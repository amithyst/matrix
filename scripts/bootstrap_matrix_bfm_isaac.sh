#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-bfm-isaac.lock.json"
PATH_GUARD="$SCRIPT_DIR/matrix_bfm_isaac_path_guard.py"

PROFILE="${MATRIX_PROFILE:-}"
RUNTIME_ROOT_OVERRIDE=""
TEACHER_PROFILE_OVERRIDE=""
COLLISION_ROOT_OVERRIDE=""
VISUAL_WHEELHOUSE_OVERRIDE=""
VISUAL_VENV_OVERRIDE=""
VISUAL_BOOTSTRAP_PYTHON_OVERRIDE=""
VERIFY_ONLY=0
REFRESH=0
VISUAL_STAGE=""

cleanup_visual_stage() {
    if [[ -n "$VISUAL_STAGE" && -e "$VISUAL_STAGE" ]]; then
        rm -rf -- "$VISUAL_STAGE"
    fi
}
trap cleanup_visual_stage EXIT

usage() {
    printf '%s\n' \
        "Usage: bash scripts/bootstrap_matrix_bfm_isaac.sh [options]" \
        "" \
        "Options:" \
        "  --profile NAME          Load Matrix host defaults" \
        "  --runtime-root PATH     Frozen BFM runtime checkout destination" \
        "  --teacher-profile PATH Verify a promoted world16 teacher profile" \
        "  --collision-root PATH  Verify the frozen Moon collision closure" \
        "  --visual-wheelhouse PATH Locked offline visual-import wheels" \
        "  --visual-venv PATH       Dedicated visual-import venv destination" \
        "  --visual-python PATH     CPython 3.10 used to create the visual venv" \
        "  --verify-only           Do not clone or replace a checkout" \
        "  --refresh               Atomically replace a mismatched checkout" \
        "  -h, --help              Show this help"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --runtime-root) RUNTIME_ROOT_OVERRIDE="$2"; shift 2 ;;
        --teacher-profile) TEACHER_PROFILE_OVERRIDE="$2"; shift 2 ;;
        --collision-root) COLLISION_ROOT_OVERRIDE="$2"; shift 2 ;;
        --visual-wheelhouse) VISUAL_WHEELHOUSE_OVERRIDE="$2"; shift 2 ;;
        --visual-venv) VISUAL_VENV_OVERRIDE="$2"; shift 2 ;;
        --visual-python) VISUAL_BOOTSTRAP_PYTHON_OVERRIDE="$2"; shift 2 ;;
        --verify-only) VERIFY_ONLY=1; shift ;;
        --refresh) REFRESH=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

for command_name in bwrap git python3 realpath; do
    command -v "$command_name" >/dev/null || {
        echo "[ERROR] Required command is unavailable: $command_name" >&2
        exit 1
    }
done
if ! bwrap --die-with-parent --unshare-net --bind / / --dev-bind /dev /dev \
    -- python3 -c \
        'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0))'; then
    echo "[ERROR] Bubblewrap private-network preflight failed" >&2
    exit 8
fi

# shellcheck disable=SC1091
source "$SCRIPT_DIR/matrix_local_env.sh"
if ! load_matrix_local_env "$PROJECT_ROOT"; then
    exit 2
fi
if [[ -n "$PROFILE" ]]; then
    PROFILE_FILE="$PROJECT_ROOT/config/hosts/$PROFILE.env"
    if [[ ! -f "$PROFILE_FILE" ]]; then
        echo "[ERROR] Unknown host profile: $PROFILE" >&2
        exit 2
    fi
    # shellcheck disable=SC1090
    source "$PROFILE_FILE"
fi

RUNTIME_ROOT="${RUNTIME_ROOT_OVERRIDE:-${MATRIX_BFM_ISAAC_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac-sync-world16-v1/bfm_runtime}}"
if ! RUNTIME_ROOT="$(python3 "$PATH_GUARD" --mode overlap "$RUNTIME_ROOT" 2>&1)"; then
    echo "[ERROR] Unsafe runtime destination: $RUNTIME_ROOT" >&2
    exit 2
fi
TEACHER_PROFILE="${TEACHER_PROFILE_OVERRIDE:-${MATRIX_BFM_ISAAC_TEACHER_PROFILE:-}}"
if [[ -z "$TEACHER_PROFILE" ]]; then
    echo "[ERROR] Teacher profile is unset; use --profile or --teacher-profile" >&2
    exit 2
fi
if ! TEACHER_PROFILE="$(python3 "$PATH_GUARD" --mode subtree "$TEACHER_PROFILE" 2>&1)"; then
    echo "[ERROR] Unsafe teacher profile: $TEACHER_PROFILE" >&2
    exit 2
fi
COLLISION_ROOT="${COLLISION_ROOT_OVERRIDE:-${MATRIX_BFM_ISAAC_COLLISION_ROOT:-}}"
if [[ -z "$COLLISION_ROOT" ]]; then
    echo "[ERROR] Moon collision root is unset; use --profile or --collision-root" >&2
    exit 2
fi
if ! COLLISION_ROOT="$(python3 "$PATH_GUARD" --mode subtree "$COLLISION_ROOT" 2>&1)"; then
    echo "[ERROR] Unsafe Moon collision root: $COLLISION_ROOT" >&2
    exit 2
fi
VISUAL_WHEELHOUSE="${VISUAL_WHEELHOUSE_OVERRIDE:-${MATRIX_BFM_ISAAC_VISUAL_WHEELHOUSE:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac/visual-import-wheelhouse}}"
if ! VISUAL_WHEELHOUSE="$(python3 "$PATH_GUARD" --mode subtree "$VISUAL_WHEELHOUSE" 2>&1)"; then
    echo "[ERROR] Unsafe visual wheelhouse: $VISUAL_WHEELHOUSE" >&2
    exit 2
fi
VISUAL_VENV_CANDIDATE="${VISUAL_VENV_OVERRIDE:-${MATRIX_BFM_ISAAC_VISUAL_VENV:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac/visual-import-venv}}"
if [[ -L "$VISUAL_VENV_CANDIDATE" ]]; then
    echo "[ERROR] Visual-import venv root must not be a symlink: $VISUAL_VENV_CANDIDATE" >&2
    exit 2
fi
if ! VISUAL_VENV="$(python3 "$PATH_GUARD" --mode overlap "$VISUAL_VENV_CANDIDATE" 2>&1)"; then
    echo "[ERROR] Unsafe visual-import venv: $VISUAL_VENV" >&2
    exit 2
fi
VISUAL_BOOTSTRAP_PYTHON="${VISUAL_BOOTSTRAP_PYTHON_OVERRIDE:-${MATRIX_BFM_ISAAC_VISUAL_BOOTSTRAP_PYTHON:-python3}}"
if ! command -v "$VISUAL_BOOTSTRAP_PYTHON" >/dev/null 2>&1; then
    echo "[ERROR] Visual bootstrap Python is unavailable: $VISUAL_BOOTSTRAP_PYTHON" >&2
    exit 1
fi

readarray -t LOCK_VALUES < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys

payload = json.load(open(sys.argv[1], encoding="utf-8"))
print(payload["bfm_runtime"]["repository"])
print(payload["bfm_runtime"]["commit"])
bridge = payload["ue_material_bridge"]
print(bridge["relative_path"])
print(bridge["sha256"])
print(bridge["ue_binary_relative_path"])
print(bridge["ue_binary_build_id"])
PY
)
if [[ "${#LOCK_VALUES[@]}" != "6" ]]; then
    echo "[ERROR] Could not read BFM runtime/material closure from lock" >&2
    exit 1
fi
RUNTIME_REPOSITORY="${LOCK_VALUES[0]}"
RUNTIME_COMMIT="${LOCK_VALUES[1]}"
MATERIAL_BRIDGE="$PROJECT_ROOT/${LOCK_VALUES[2]}"
MATERIAL_BRIDGE_SHA256="${LOCK_VALUES[3]}"
MATERIAL_UE_BINARY="$PROJECT_ROOT/${LOCK_VALUES[4]}"
MATERIAL_UE_BUILD_ID="${LOCK_VALUES[5]}"

python3 "$SCRIPT_DIR/verify_matrix_bfm_isaac_runtime.py" \
    --lock "$LOCK_FILE" --schema-only

install_frozen_checkout() {
    local destination="$1"
    local parent stage
    if ! destination="$(python3 "$PATH_GUARD" --mode overlap "$destination" 2>&1)"; then
        echo "[ERROR] Unsafe runtime destination before install: $destination" >&2
        return 2
    fi
    parent="$(dirname "$destination")"
    mkdir -p "$parent"
    stage="$(mktemp -d "$parent/.bfm-runtime.stage.XXXXXX")"
    trap 'rm -rf -- "$stage"' RETURN
    rmdir "$stage"
    echo "[INFO] Cloning frozen BFM runtime $RUNTIME_COMMIT"
    git clone --filter=blob:none --no-checkout "$RUNTIME_REPOSITORY" "$stage"
    git -C "$stage" fetch --depth=1 origin "$RUNTIME_COMMIT"
    git -C "$stage" checkout --detach "$RUNTIME_COMMIT"
    if [[ -e "$destination" ]]; then
        local backup="${destination}.backup.$(date -u +%Y%m%dT%H%M%SZ)"
        mv -- "$destination" "$backup"
        echo "[INFO] Preserved previous runtime checkout at $backup"
    fi
    mv -- "$stage" "$destination"
    trap - RETURN
}

if [[ "$VERIFY_ONLY" != "1" ]]; then
    if ! git -C "$RUNTIME_ROOT" rev-parse --is-inside-work-tree \
        >/dev/null 2>&1; then
        if [[ -e "$RUNTIME_ROOT" && "$REFRESH" != "1" ]]; then
            echo "[ERROR] Runtime destination exists but is not a Git checkout: $RUNTIME_ROOT" >&2
            echo "[ERROR] Use --refresh to preserve and replace it" >&2
            exit 1
        fi
        install_frozen_checkout "$RUNTIME_ROOT"
    else
        ACTUAL_COMMIT="$(git -C "$RUNTIME_ROOT" rev-parse HEAD)"
        if [[ "$ACTUAL_COMMIT" != "$RUNTIME_COMMIT" ]]; then
            if [[ "$REFRESH" != "1" ]]; then
                echo "[ERROR] Existing BFM runtime is not the locked commit:" >&2
                echo "[ERROR] expected=$RUNTIME_COMMIT actual=$ACTUAL_COMMIT" >&2
                echo "[ERROR] Use --refresh for an atomic preserved replacement" >&2
                exit 1
            fi
            install_frozen_checkout "$RUNTIME_ROOT"
        fi
    fi
fi

VERIFY_ARGS=(
    --lock "$LOCK_FILE"
    --runtime-root "$RUNTIME_ROOT"
    --collision-root "$COLLISION_ROOT"
)
if [[ -n "$TEACHER_PROFILE" && -f "$TEACHER_PROFILE" ]]; then
    VERIFY_ARGS+=(--teacher-profile "$TEACHER_PROFILE")
elif [[ -n "$TEACHER_PROFILE_OVERRIDE" ]]; then
    echo "[ERROR] Requested teacher profile is missing: $TEACHER_PROFILE" >&2
    exit 1
else
    echo "[WARN] Promoted world16 profile is not present on this host: $TEACHER_PROFILE" >&2
    echo "[WARN] Runtime code is verified; launch remains fail-closed until the profile is installed" >&2
fi
python3 "$SCRIPT_DIR/verify_matrix_bfm_isaac_runtime.py" "${VERIFY_ARGS[@]}"

verify_visual_wheelhouse() {
    python3 "$SCRIPT_DIR/verify_matrix_bfm_isaac_runtime.py" \
        --lock "$LOCK_FILE" --visual-wheelhouse "$VISUAL_WHEELHOUSE"
}

verify_visual_venv() {
    local root="$1"
    python3 "$SCRIPT_DIR/verify_matrix_bfm_isaac_runtime.py" \
        --lock "$LOCK_FILE" --visual-venv "$root"
}

install_visual_venv() {
    local identity parent stage backup marker
    verify_visual_wheelhouse
    if [[ -d "$VISUAL_VENV" ]] && verify_visual_venv "$VISUAL_VENV"; then
        echo "[PASS] Locked visual-import venv is already ready: $VISUAL_VENV"
        return 0
    fi

    identity="$(
        env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 \
            "$VISUAL_BOOTSTRAP_PYTHON" -I - <<'PY'
import platform
import sys
print(platform.python_implementation())
print(f"{sys.version_info.major}.{sys.version_info.minor}")
print(platform.machine())
PY
    )"
    if [[ "$identity" != $'CPython\n3.10\nx86_64' ]]; then
        echo "[ERROR] Visual bootstrap interpreter must be CPython 3.10 x86_64" >&2
        echo "[ERROR] actual=$identity" >&2
        return 1
    fi
    if ! env -u PYTHONHOME -u PYTHONPATH \
        "$VISUAL_BOOTSTRAP_PYTHON" -I -c 'import ensurepip, venv'; then
        echo "[ERROR] Visual bootstrap Python requires ensurepip and venv" >&2
        return 1
    fi

    parent="$(dirname "$VISUAL_VENV")"
    mkdir -p "$parent"
    stage="$(mktemp -d "$parent/.visual-import-venv.stage.XXXXXX")"
    VISUAL_STAGE="$stage"
    rmdir "$stage"
    echo "[INFO] Creating locked visual-import venv offline: $stage"
    env -u PYTHONHOME -u PYTHONPATH PYTHONNOUSERSITE=1 \
        "$VISUAL_BOOTSTRAP_PYTHON" -I -m venv "$stage"
    bwrap --die-with-parent --unshare-net --bind / / --dev-bind /dev /dev \
        -- env -u PYTHONHOME -u PYTHONPATH \
        PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
        PIP_CONFIG_FILE=/dev/null PIP_DISABLE_PIP_VERSION_CHECK=1 \
        "$stage/bin/python" -I -m pip install \
        --no-index --only-binary=:all: --no-compile \
        --find-links "$VISUAL_WHEELHOUSE" \
        --requirement \
            "$PROJECT_ROOT/config/runtime/matrix-bfm-isaac-visual-requirements.txt"
    bwrap --die-with-parent --unshare-net --bind / / --dev-bind /dev /dev \
        -- env -u PYTHONHOME -u PYTHONPATH \
        PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
        PIP_CONFIG_FILE=/dev/null PIP_DISABLE_PIP_VERSION_CHECK=1 \
        "$stage/bin/python" -I -m pip check

    marker="$stage/.matrix-bfm-visual-lock.json"
    python3 -I - "$LOCK_FILE" "$marker" <<'PY'
import json
import os
from pathlib import Path
import sys

lock = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
visual = lock["visual_import"]
payload = {
    "schema": "matrix_bfm_isaac_visual_venv.v1",
    "python_implementation": visual["python_implementation"],
    "python_version": visual["python_version"],
    "platform_machine": visual["platform_machine"],
    "requirements_sha256": visual["requirements_sha256"],
    "wheelhouse_manifest_sha256": visual["wheelhouse_manifest_sha256"],
}
path = Path(sys.argv[2])
temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
os.replace(temporary, path)
PY
    verify_visual_venv "$stage"

    backup=""
    if [[ -e "$VISUAL_VENV" ]]; then
        backup="${VISUAL_VENV}.backup.$(date -u +%Y%m%dT%H%M%SZ)"
        mv -- "$VISUAL_VENV" "$backup"
    fi
    mv -- "$stage" "$VISUAL_VENV"
    VISUAL_STAGE=""
    if ! verify_visual_venv "$VISUAL_VENV"; then
        local failed="${VISUAL_VENV}.failed.$(date -u +%Y%m%dT%H%M%SZ)"
        mv -- "$VISUAL_VENV" "$failed"
        if [[ -n "$backup" ]]; then
            mv -- "$backup" "$VISUAL_VENV"
        fi
        echo "[ERROR] Final visual-import venv verification failed; preserved $failed" >&2
        return 1
    fi
    if [[ -n "$backup" ]]; then
        echo "[INFO] Preserved previous visual-import venv at $backup"
    fi
    echo "[PASS] Locked visual-import venv is ready: $VISUAL_VENV"
}

MATERIAL_BRIDGE_ARGS=(
    --output "$MATERIAL_BRIDGE"
    --expected-sha256 "$MATERIAL_BRIDGE_SHA256"
    --expected-ue-build-id "$MATERIAL_UE_BUILD_ID"
)
if [[ "$VERIFY_ONLY" == "1" ]]; then
    verify_visual_wheelhouse
    verify_visual_venv "$VISUAL_VENV"
    MATRIX_UE_BINARY="$MATERIAL_UE_BINARY" \
        bash "$SCRIPT_DIR/build_matrix_ue_material_fix.sh" \
        "${MATERIAL_BRIDGE_ARGS[@]}" --verify-only
else
    install_visual_venv
    if ! MATRIX_UE_BINARY="$MATERIAL_UE_BINARY" \
        bash "$SCRIPT_DIR/build_matrix_ue_material_fix.sh" \
        "${MATERIAL_BRIDGE_ARGS[@]}" --verify-only; then
        MATRIX_UE_BINARY="$MATERIAL_UE_BINARY" \
            bash "$SCRIPT_DIR/build_matrix_ue_material_fix.sh" \
            "${MATERIAL_BRIDGE_ARGS[@]}"
    fi
fi

echo "[PASS] Frozen Leo BFM runtime is ready: $RUNTIME_ROOT"
echo "[PASS] Frozen Matrix visual-import environment is ready: $VISUAL_VENV"
echo "[PASS] Audited Matrix UE material bridge is ready"
echo "[INFO] Next: bash scripts/run_matrix_bfm_isaac_guarded.sh smoke --profile ${PROFILE:-trna}"
