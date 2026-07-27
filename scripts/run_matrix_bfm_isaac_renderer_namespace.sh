#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

required_environment=(
    MATRIX_BFM_ISAAC_STATE_SOCKET
    MATRIX_BFM_ISAAC_RELAY_STATUS
    MATRIX_BFM_ISAAC_RELAY_LOG
    MATRIX_BFM_ISAAC_BOOTSTRAP_STATE
    MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE
    MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR
)
for variable_name in "${required_environment[@]}"; do
    if [[ -z "${!variable_name:-}" ]]; then
        echo "[ERROR] Missing renderer namespace variable: $variable_name" >&2
        exit 2
    fi
done

STATE_SOCKET="$MATRIX_BFM_ISAAC_STATE_SOCKET"
RELAY_STATUS="$MATRIX_BFM_ISAAC_RELAY_STATUS"
RELAY_LOG="$MATRIX_BFM_ISAAC_RELAY_LOG"
BOOTSTRAP_STATE="$MATRIX_BFM_ISAAC_BOOTSTRAP_STATE"
NAMESPACE_PID_FILE="$MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE"
CHECKOUT_SNAPSHOT_DIR="$MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR"
CHECKOUT_SNAPSHOT_HELPER="$SCRIPT_DIR/matrix_bfm_isaac_checkout_snapshot.py"
KEYBOARD_SOCKET="${MATRIX_BFM_ISAAC_KEYBOARD_SOCKET:-}"
RUN_SIM_PID=""
RELAY_PID=""
CHECKOUT_SNAPSHOT_ACTIVE=0
TRACKED_MUTATION_PATHS=(
    config/config.json
    src/robot_mc/run_mc.sh
    src/robot_mujoco/simulate/config.yaml
)

stop_exact_child() {
    local pid="$1"
    local grace_s="$2"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi
    kill -TERM "$pid" 2>/dev/null || true
    local attempts=$((grace_s * 10))
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local exit_status="$?"
    local cleanup_status=0
    trap - EXIT INT TERM HUP
    stop_exact_child "$RELAY_PID" 3
    stop_exact_child "$RUN_SIM_PID" 20
    if [[ "$CHECKOUT_SNAPSHOT_ACTIVE" == "1" ]]; then
        if python3 "$CHECKOUT_SNAPSHOT_HELPER" \
            --root "$PROJECT_ROOT" \
            --snapshot "$CHECKOUT_SNAPSHOT_DIR" \
            restore --remove-snapshot; then
            CHECKOUT_SNAPSHOT_ACTIVE=0
        else
            cleanup_status=70
            echo "[ERROR] Matrix tracked-source restoration failed;" \
                "preserving $CHECKOUT_SNAPSHOT_DIR" >&2
        fi
    fi
    rm -f -- "$STATE_SOCKET"
    rm -f -- "$NAMESPACE_PID_FILE"
    if [[ "$cleanup_status" != "0" ]]; then
        exit "$cleanup_status"
    fi
    exit "$exit_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

mkdir -p "$(dirname "$NAMESPACE_PID_FILE")"
NAMESPACE_PID_TMP="${NAMESPACE_PID_FILE}.tmp.$$"
printf '%s\n' "$$" > "$NAMESPACE_PID_TMP"
chmod 600 "$NAMESPACE_PID_TMP"
mv -- "$NAMESPACE_PID_TMP" "$NAMESPACE_PID_FILE"

SNAPSHOT_ARGS=(
    --root "$PROJECT_ROOT"
    --snapshot "$CHECKOUT_SNAPSHOT_DIR"
    capture
)
for tracked_path in "${TRACKED_MUTATION_PATHS[@]}"; do
    SNAPSHOT_ARGS+=(--path "$tracked_path")
done
python3 "$CHECKOUT_SNAPSHOT_HELPER" "${SNAPSHOT_ARGS[@]}"
CHECKOUT_SNAPSHOT_ACTIVE=1

export MATRIX_EXTERNAL_STATE=1
export MATRIX_DISABLE_MC=1
export MATRIX_SONIC=0
export MATRIX_ITEM_INVENTORY_CATALOG=
export MATRIX_CREATIVE_INVENTORY_CATALOG=

bash "$PROJECT_ROOT/scripts/run_sim.sh" "$@" &
RUN_SIM_PID=$!

UDP_READY=0
for _attempt in $(seq 1 900); do
    if ! kill -0 "$RUN_SIM_PID" 2>/dev/null; then
        echo "[ERROR] Matrix renderer exited before namespace UDP 9999 was ready" >&2
        exit 4
    fi
    if ss -H -lun 'sport = :9999' 2>/dev/null | grep -q ':9999'; then
        UDP_READY=1
        break
    fi
    sleep 0.1
done
if [[ "$UDP_READY" != "1" ]]; then
    echo "[ERROR] Namespace-local Matrix UDP 9999 readiness timed out" >&2
    exit 4
fi

RELAY_ARGS=(
    --unix-socket "$STATE_SOCKET"
    --status-file "$RELAY_STATUS"
    --udp-port 9999
    --input-nq 36
    --input-nv 35
    --input-nu 0
    --output-nu 29
    --ctrl-fill 0
    --control-hz 50
    --output-hz "${MATRIX_BFM_ISAAC_RENDER_HZ:-50}"
    --interpolation-buffer-s "${MATRIX_BFM_ISAAC_RENDER_BUFFER_S:-0.10}"
    --bootstrap-state "$BOOTSTRAP_STATE"
    --bootstrap-hz 50
)
if [[ "${MATRIX_BFM_ISAAC_NO_INTERPOLATE:-0}" == "1" ]]; then
    RELAY_ARGS+=(--no-interpolate)
fi
if [[ -n "$KEYBOARD_SOCKET" ]]; then
    RELAY_ARGS+=(--safety-keyboard-socket "$KEYBOARD_SOCKET")
fi
if [[ -n "${MATRIX_BFM_ISAAC_COLLISION_BOUNDS:-}" ]]; then
    read -r -a COLLISION_BOUNDS <<< "$MATRIX_BFM_ISAAC_COLLISION_BOUNDS"
    if [[ "${#COLLISION_BOUNDS[@]}" != "4" ]]; then
        echo "[ERROR] MATRIX_BFM_ISAAC_COLLISION_BOUNDS must contain four numbers" >&2
        exit 2
    fi
    RELAY_ARGS+=(
        --collision-x-min "${COLLISION_BOUNDS[0]}"
        --collision-x-max "${COLLISION_BOUNDS[1]}"
        --collision-y-min "${COLLISION_BOUNDS[2]}"
        --collision-y-max "${COLLISION_BOUNDS[3]}"
        --collision-warning-margin "${MATRIX_BFM_ISAAC_COLLISION_WARNING_MARGIN:-20}"
        --collision-stop-margin "${MATRIX_BFM_ISAAC_COLLISION_STOP_MARGIN:-10}"
    )
fi

python3 -u "$SCRIPT_DIR/matrix_external_state_relay.py" "${RELAY_ARGS[@]}" \
    > "$RELAY_LOG" 2>&1 &
RELAY_PID=$!
for _attempt in $(seq 1 100); do
    [[ -S "$STATE_SOCKET" ]] && break
    if ! kill -0 "$RELAY_PID" 2>/dev/null; then
        echo "[ERROR] Matrix state relay exited during startup" >&2
        exit 4
    fi
    sleep 0.05
done
if [[ ! -S "$STATE_SOCKET" ]]; then
    echo "[ERROR] Matrix state relay did not create $STATE_SOCKET" >&2
    exit 4
fi

set +e
COMPLETED_PID=""
wait -n -p COMPLETED_PID "$RUN_SIM_PID" "$RELAY_PID"
RESULT=$?
set -e
if [[ "$COMPLETED_PID" == "$RELAY_PID" ]]; then
    RELAY_PID=""
    echo "[ERROR] Matrix state relay exited before the renderer (code $RESULT)" >&2
    exit "$RESULT"
fi
RUN_SIM_PID=""
exit "$RESULT"
