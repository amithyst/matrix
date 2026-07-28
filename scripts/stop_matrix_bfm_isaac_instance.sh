#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
INSTANCE_ID="${MATRIX_INSTANCE_ID:-matrix-bfm-isaac}"
STATE_ROOT="${MATRIX_BFM_ISAAC_STATE_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac}"

if [[ ! "$INSTANCE_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "[ERROR] MATRIX_INSTANCE_ID must match [A-Za-z0-9._-]+" >&2
    exit 2
fi
if ! STATE_ROOT="$(python3 "$SCRIPT_DIR/matrix_bfm_isaac_path_guard.py" \
    --mode subtree "$STATE_ROOT" 2>&1)"; then
    echo "[ERROR] Unsafe runtime state root: $STATE_ROOT" >&2
    exit 2
fi

INSTANCE_DIR="$STATE_ROOT/instances/$INSTANCE_ID"
LEDGER_FILE="$INSTANCE_DIR/owner-ledger.json"
LEDGER_TOOL="$SCRIPT_DIR/matrix_bfm_isaac_instance_ledger.py"
if [[ ! -f "$LEDGER_FILE" ]]; then
    echo "[INFO] Matrix BFM/Isaac instance '$INSTANCE_ID' has no owner ledger"
    exit 0
fi

EXPECTED_ARGS=()
if [[ -n "${MATRIX_EXPECTED_NONCE:-}" ]]; then
    EXPECTED_ARGS=(--nonce "$MATRIX_EXPECTED_NONCE")
fi
python3 "$LEDGER_TOOL" --path "$LEDGER_FILE" "${EXPECTED_ARGS[@]}" \
    signal --signal TERM

deadline=$((SECONDS + 3))
while ((SECONDS < deadline)); do
    if python3 "$LEDGER_TOOL" --path "$LEDGER_FILE" "${EXPECTED_ARGS[@]}" \
        verify-empty >/dev/null 2>&1; then
        break
    fi
    sleep 0.1
done
python3 "$LEDGER_TOOL" --path "$LEDGER_FILE" "${EXPECTED_ARGS[@]}" \
    signal --signal KILL

if python3 "$LEDGER_TOOL" --path "$LEDGER_FILE" "${EXPECTED_ARGS[@]}" \
    verify-empty >/dev/null; then
    rm -f -- "$LEDGER_FILE"
    echo "[INFO] Matrix BFM/Isaac instance '$INSTANCE_ID' stopped"
else
    echo "[ERROR] Owned processes remain; preserving $LEDGER_FILE" >&2
    exit 5
fi
