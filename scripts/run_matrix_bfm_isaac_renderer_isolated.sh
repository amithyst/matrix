#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for command_name in bwrap ss; do
    command -v "$command_name" >/dev/null || {
        echo "[ERROR] $command_name is required for isolated Matrix UDP 9999" >&2
        exit 1
    }
done

# The cooked UE consumer is fixed to UDP 9999. Keep both UE and the relay in a
# private network namespace; the host-side Isaac process crosses the boundary
# only through the instance-owned Unix datagram socket.
exec bwrap \
    --die-with-parent \
    --unshare-net \
    --bind / / \
    --dev-bind /dev /dev \
    -- bash "$SCRIPT_DIR/run_matrix_bfm_isaac_renderer_namespace.sh" "$@"
