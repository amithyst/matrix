#!/usr/bin/env bash
set -euo pipefail

BFM_GPU_PYTHON="${MATRIX_BFM_SONIC_GPU_PYTHON:-$HOME/matrix-artifacts/bfm-teacher50k-matrix-v1/venv-gpu/bin/python}"
BFM_BASE_ENV_LIB="${MATRIX_BFM_SONIC_BASE_ENV_LIB:-$HOME/miniconda3/envs/sonic-h2-sim/lib}"

if [[ ! -x "$BFM_GPU_PYTHON" ]]; then
  echo "[ERROR] BFM GPU Python is not executable: $BFM_GPU_PYTHON" >&2
  exit 1
fi
if [[ ! -d "$BFM_BASE_ENV_LIB" ]]; then
  echo "[ERROR] BFM base environment library directory is missing: $BFM_BASE_ENV_LIB" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$BFM_BASE_ENV_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$BFM_GPU_PYTHON" "$@"
