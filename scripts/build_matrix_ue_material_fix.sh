#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SOURCE="$PROJECT_ROOT/src/ue_shims/matrix_ue_material_fix.c"
UE_BINARY="${MATRIX_UE_BINARY:-$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue}"
OUTPUT="${MATRIX_UE_MATERIAL_FIX_OUTPUT:-$PROJECT_ROOT/outputs/runtime/matrix-ue-material-fix/libmatrix_ue_material_fix.so}"
EXPECTED_BUILD_ID="056e17b8675b1006"

usage() {
    cat <<'EOF'
Usage: scripts/build_matrix_ue_material_fix.sh [--output ABSOLUTE_PATH]

Build the guarded Matrix 0.1.2 UE material bridge used through
MATRIX_UE_MATERIAL_FIX_PRELOAD.  The build refuses unknown UE executables.
EOF
}

while (($#)); do
    case "$1" in
        --output)
            [[ $# -ge 2 ]] || { echo "[ERROR] --output requires a path" >&2; exit 2; }
            OUTPUT="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ "$OUTPUT" == /* ]] || {
    echo "[ERROR] material-fix output must be absolute: $OUTPUT" >&2
    exit 2
}
[[ -f "$SOURCE" ]] || {
    echo "[ERROR] material-fix source is missing: $SOURCE" >&2
    exit 1
}
[[ -f "$UE_BINARY" ]] || {
    echo "[ERROR] Matrix UE executable is missing: $UE_BINARY" >&2
    exit 1
}
command -v readelf >/dev/null || {
    echo "[ERROR] readelf is required to verify the Matrix UE build" >&2
    exit 1
}
CC="${CC:-cc}"
command -v "$CC" >/dev/null || {
    echo "[ERROR] C compiler is required: $CC" >&2
    exit 1
}

build_id="$(readelf -n "$UE_BINARY" | awk '/Build ID:/ {print $3; exit}')"
if [[ "$build_id" != "$EXPECTED_BUILD_ID" ]]; then
    echo "[ERROR] unsupported Matrix UE Build ID: ${build_id:-missing}" >&2
    echo "[ERROR] expected: $EXPECTED_BUILD_ID" >&2
    exit 1
fi

mkdir -p -- "$(dirname -- "$OUTPUT")"
temporary="$OUTPUT.tmp.$$"
cleanup() {
    rm -f -- "$temporary"
}
trap cleanup EXIT

"$CC" \
    -std=c11 \
    -O2 \
    -fPIC \
    -fvisibility=hidden \
    -fcf-protection=branch \
    -Wall \
    -Wextra \
    -Werror \
    -shared \
    -Wl,-z,defs \
    -Wl,-z,relro \
    -Wl,-z,now \
    -o "$temporary" \
    "$SOURCE"
chmod 0755 -- "$temporary"
mv -f -- "$temporary" "$OUTPUT"
trap - EXIT

echo "[PASS] built Matrix UE material fix: $OUTPUT"
echo "[INFO] enable only for UE with MATRIX_UE_MATERIAL_FIX_PRELOAD=$OUTPUT"
