#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"

for command_name in bwrap flock python3 realpath; do
    command -v "$command_name" >/dev/null || {
        echo "[ERROR] Required guarded-launch command is unavailable: $command_name" >&2
        exit 1
    }
done

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    exec bash "$SCRIPT_DIR/run_matrix_bfm_isaac.sh" --help
fi

PROFILE="${MATRIX_PROFILE:-}"
arguments=("$@")
for ((index = 0; index < ${#arguments[@]}; index++)); do
    if [[ "${arguments[$index]}" == "--profile" ]]; then
        if ((index + 1 >= ${#arguments[@]})); then
            echo "[ERROR] --profile requires a value" >&2
            exit 2
        fi
        PROFILE="${arguments[$((index + 1))]}"
    fi
done

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

# Share the host-wide Matrix lock with the native SONIC launcher and desktop
# entry point.  The renderer runs in a private network namespace, so UDP 9999
# cannot provide mutual exclusion, and the resource guard intentionally
# ignores processes rooted in this checkout.  Keep this descriptor open in the
# guard process for the entire launch lifetime; its supervised child closes
# inherited descriptors before starting the stack.
MATRIX_SONIC_HOST_LOCK="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"
exec 8>"$MATRIX_SONIC_HOST_LOCK"
if ! flock -n 8; then
    echo "[ERROR] Another Matrix launcher owns this host: $MATRIX_SONIC_HOST_LOCK" >&2
    exit 75
fi

INSTANCE_ID="${MATRIX_INSTANCE_ID:-matrix-bfm-isaac}"
STATE_ROOT="${MATRIX_BFM_ISAAC_STATE_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac}"
RUN_ROOT="${MATRIX_BFM_ISAAC_RUN_ROOT:-$PROJECT_ROOT/outputs/runs/matrix-bfm-isaac}"
if [[ ! "$INSTANCE_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "[ERROR] MATRIX_INSTANCE_ID must match [A-Za-z0-9._-]+" >&2
    exit 2
fi

deny_colleague_path() {
    local label="$1"
    local candidate="$2"
    local resolved
    if ! resolved="$(python3 "$SCRIPT_DIR/matrix_bfm_isaac_path_guard.py" \
        --mode subtree "$candidate" 2>&1)"; then
        echo "[ERROR] Unsafe $label: $resolved" >&2
        exit 2
    fi
    printf '%s\n' "$resolved"
}

STATE_ROOT="$(deny_colleague_path state-root "$STATE_ROOT")"
RUN_ROOT="$(deny_colleague_path run-root "$RUN_ROOT")"
INSTANCE_DIR="$STATE_ROOT/instances/$INSTANCE_ID"
STATUS_FILE="$INSTANCE_DIR/resource-guard-status.json"
GPU_LOCK="$STATE_ROOT/matrix-bfm-isaac-gpu.lock"
mkdir -p "$INSTANCE_DIR" "$RUN_ROOT"
chmod 700 "$INSTANCE_DIR"

if ! bwrap --die-with-parent --unshare-net --bind / / --dev-bind /dev /dev \
    -- python3 -c \
        'import socket; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0))'; then
    echo "[ERROR] Bubblewrap private-network preflight failed; no Matrix/Isaac process was started" >&2
    exit 8
fi

export MATRIX_INSTANCE_ID="$INSTANCE_ID"
export MATRIX_BFM_ISAAC_STATE_ROOT="$STATE_ROOT"
export MATRIX_BFM_ISAAC_RUN_ROOT="$RUN_ROOT"
export MATRIX_BFM_ISAAC_GUARDED=1

exec python3 "$SCRIPT_DIR/matrix_bfm_isaac_resource_guard.py" \
    --own-root "$PROJECT_ROOT" \
    --min-free-vram-mib "${MATRIX_BFM_ISAAC_MIN_FREE_VRAM_MIB:-12288}" \
    --min-available-ram-mib "${MATRIX_BFM_ISAAC_MIN_AVAILABLE_RAM_MIB:-12288}" \
    run \
    --status "$STATUS_FILE" \
    --lock "$GPU_LOCK" \
    --cleanup-script "$SCRIPT_DIR/stop_matrix_bfm_isaac_instance.sh" \
    --runtime-floor-vram-mib "${MATRIX_BFM_ISAAC_RUNTIME_FLOOR_VRAM_MIB:-1536}" \
    --runtime-floor-ram-mib "${MATRIX_BFM_ISAAC_RUNTIME_FLOOR_RAM_MIB:-3072}" \
    --interval "${MATRIX_BFM_ISAAC_RESOURCE_POLL_S:-0.25}" \
    --shutdown-grace "${MATRIX_BFM_ISAAC_GUARD_SHUTDOWN_GRACE_S:-40}" \
    -- bash "$SCRIPT_DIR/run_matrix_bfm_isaac.sh" "$@"
