#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [[ -n "${MATRIX_VIDEO_PYTHON:-}" ]]; then
    PYTHON="$MATRIX_VIDEO_PYTHON"
elif [[ -n "${MATRIX_SONIC_PYTHON:-}" ]]; then
    PYTHON="$MATRIX_SONIC_PYTHON"
elif [[ -x "$PROJECT_ROOT/.venv-audit/bin/python" ]]; then
    PYTHON="$PROJECT_ROOT/.venv-audit/bin/python"
else
    PYTHON="$(command -v python3)"
fi

exec "$PYTHON" "$SCRIPT_DIR/record_matrix_sonic_video.py" "$@"
