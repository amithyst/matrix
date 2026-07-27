#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SOURCE="$PROJECT_ROOT/src/ue_shims/matrix_ue_material_fix.c"
UE_BINARY="${MATRIX_UE_BINARY:-$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/zsibot_mujoco_ue}"
OUTPUT="${MATRIX_UE_MATERIAL_FIX_OUTPUT:-$PROJECT_ROOT/outputs/runtime/matrix-ue-material-fix/libmatrix_ue_material_fix.so}"
EXPECTED_BUILD_ID="056e17b8675b1006"
EXPECTED_MARKER="matrix-ue-material-fix: installed audited Matrix 0.1.2 material bridge"
EXPECTED_SHA256=""
VERIFY_ONLY=0

usage() {
    cat <<'EOF'
Usage: scripts/build_matrix_ue_material_fix.sh [--output ABSOLUTE_PATH]
       [--expected-sha256 SHA256] [--expected-ue-build-id BUILD_ID]
       [--verify-only]

Build the guarded Matrix 0.1.2 UE material bridge used through
MATRIX_UE_MATERIAL_FIX_PRELOAD.  Launch through run_matrix_sonic.sh so the
registered G1 skin palette and scope tag are supplied.  The build refuses
unknown UE executables.  --expected-sha256 pins the exact output bytes for both
build and verify-only modes.  --verify-only never invokes the compiler.
EOF
}

while (($#)); do
    case "$1" in
        --output)
            [[ $# -ge 2 ]] || { echo "[ERROR] --output requires a path" >&2; exit 2; }
            OUTPUT="$2"
            shift 2
            ;;
        --verify-only)
            VERIFY_ONLY=1
            shift
            ;;
        --expected-sha256)
            [[ $# -ge 2 ]] || { echo "[ERROR] --expected-sha256 requires a digest" >&2; exit 2; }
            EXPECTED_SHA256="$2"
            shift 2
            ;;
        --expected-ue-build-id)
            [[ $# -ge 2 ]] || { echo "[ERROR] --expected-ue-build-id requires an id" >&2; exit 2; }
            EXPECTED_BUILD_ID="$2"
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
if [[ -n "$EXPECTED_SHA256" \
    && ! "$EXPECTED_SHA256" =~ ^[0-9a-f]{64}$ ]]; then
    echo "[ERROR] --expected-sha256 must be a lowercase SHA256" >&2
    exit 2
fi
if [[ ! "$EXPECTED_BUILD_ID" =~ ^[0-9a-f]{16,64}$ ]]; then
    echo "[ERROR] --expected-ue-build-id must be a lowercase ELF Build ID" >&2
    exit 2
fi
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
command -v strings >/dev/null || {
    echo "[ERROR] strings is required to verify the Matrix UE build" >&2
    exit 1
}
command -v sha256sum >/dev/null || {
    echo "[ERROR] sha256sum is required to verify the Matrix UE bridge" >&2
    exit 1
}

build_id="$(readelf -n "$UE_BINARY" | awk '/Build ID:/ {print $3; exit}')"
if [[ "$build_id" != "$EXPECTED_BUILD_ID" ]]; then
    echo "[ERROR] unsupported Matrix UE Build ID: ${build_id:-missing}" >&2
    echo "[ERROR] expected: $EXPECTED_BUILD_ID" >&2
    exit 1
fi

verify_output() {
    local candidate="$1"
    local actual_sha256 header
    if [[ ! -f "$candidate" || -L "$candidate" || ! -x "$candidate" ]]; then
        echo "[ERROR] material-fix output must be a regular executable file: $candidate" >&2
        return 1
    fi
    if ! header="$(LC_ALL=C readelf -h "$candidate" 2>/dev/null)"; then
        echo "[ERROR] material-fix output is not a readable ELF file: $candidate" >&2
        return 1
    fi
    if [[ "$header" != *"Class:                             ELF64"* \
        || "$header" != *"Data:                              2's complement, little endian"* \
        || "$header" != *"Type:                              DYN (Shared object file)"* \
        || "$header" != *"Machine:                           Advanced Micro Devices X86-64"* ]]; then
        echo "[ERROR] material-fix output is not the required x86_64 ELF shared object" >&2
        return 1
    fi
    if ! LC_ALL=C strings "$candidate" | grep -Fqx -- "$EXPECTED_MARKER"; then
        echo "[ERROR] material-fix output is missing the audited install marker" >&2
        return 1
    fi
    actual_sha256="$(sha256sum -- "$candidate" | awk '{print $1}')"
    if [[ -n "$EXPECTED_SHA256" && "$actual_sha256" != "$EXPECTED_SHA256" ]]; then
        echo "[ERROR] material-fix SHA256 mismatch: $candidate" >&2
        echo "[ERROR] expected=$EXPECTED_SHA256 actual=$actual_sha256" >&2
        return 1
    fi
    echo "[PASS] verified Matrix UE material fix: $candidate"
    echo "[PASS] Matrix UE material fix SHA256: $actual_sha256"
}

if [[ "$VERIFY_ONLY" == "1" ]]; then
    verify_output "$OUTPUT"
    exit 0
fi

CC="${CC:-cc}"
command -v "$CC" >/dev/null || {
    echo "[ERROR] C compiler is required: $CC" >&2
    exit 1
}

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
verify_output "$temporary"
mv -f -- "$temporary" "$OUTPUT"
trap - EXIT

echo "[PASS] built Matrix UE material fix: $OUTPUT"
echo "[INFO] use run_matrix_sonic.sh with MATRIX_UE_MATERIAL_FIX_PRELOAD=$OUTPUT"
echo "[INFO] the launcher supplies the selected registered G1 skin contract"
