#!/usr/bin/env bash
set -euo pipefail

VENV_ROOT="${MATRIX_BFM_ISAAC_VISUAL_VENV:?MATRIX_BFM_ISAAC_VISUAL_VENV is required}"
PYTHON="$VENV_ROOT/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "[ERROR] Locked Matrix visual-import Python is missing: $PYTHON" >&2
    exit 1
fi

unset PYTHONHOME
unset PYTHONPATH
export PYTHONNOUSERSITE=1
export PYTHONDONTWRITEBYTECODE=1
exec "$PYTHON" -I "$@"
