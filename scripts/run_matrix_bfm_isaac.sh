#!/usr/bin/env bash
set -euo pipefail

if ((BASH_VERSINFO[0] < 5 \
    || (BASH_VERSINFO[0] == 5 && BASH_VERSINFO[1] < 1))); then
    echo "[ERROR] Matrix BFM/Isaac supervision requires Bash 5.1 or newer" >&2
    exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONDONTWRITEBYTECODE=1

MODE="${1:-interactive}"
if [[ $# -gt 0 ]]; then
    shift
fi
case "$MODE" in
    smoke|interactive) ;;
    -h|--help) MODE="help" ;;
    *) echo "[ERROR] Mode must be smoke or interactive: $MODE" >&2; exit 2 ;;
esac

PROFILE="${MATRIX_PROFILE:-}"
SCENE_ID="15"
DURATION="20"
SCHEDULE="stand:2,walk:12,stand:2"
OFFSCREEN="auto"
CORRECTNESS_ONLY=0
NO_INTERPOLATE=0
RUNTIME_ROOT_OVERRIDE=""
RUNTIME_PYTHON_OVERRIDE=""
RUNTIME_CONFIG_OVERRIDE=""
TEACHER_PROFILE_OVERRIDE=""
PHYSICS_ASSET_ROOT_OVERRIDE=""
COLLISION_ROOT_OVERRIDE=""
VISUAL_URDF_OVERRIDE=""
RUN_DIR_OVERRIDE=""

usage() {
    printf '%s\n' \
        "Usage: bash scripts/run_matrix_bfm_isaac_guarded.sh [smoke|interactive] [options]" \
        "" \
        "Options:" \
        "  --profile NAME              Load Matrix host defaults" \
        "  --scene ID                  Matrix scene (default: 15 MoonWorld)" \
        "  --duration SEC              Interactive simulated duration" \
        "  --schedule SPEC             Smoke schedule (default: stand:2,walk:12,stand:2)" \
        "  --offscreen                 Force Matrix renderer offscreen" \
        "  --onscreen                  Force Matrix renderer onscreen" \
        "  --runtime-root PATH         Frozen bfm-sonic-realscan-play checkout" \
        "  --runtime-python PATH       Leo Isaac runtime Python" \
        "  --runtime-config PATH       Frozen runtime TOML" \
        "  --teacher-profile PATH      Promoted world16 step079000 profile" \
        "  --physics-asset-root PATH   Frozen seven-file G1 USD closure" \
        "  --collision-root PATH       Frozen MoonWorld collision closure" \
        "  --visual-urdf PATH          Existing canonical Matrix G1 visual" \
        "  --run-dir PATH              Evidence output directory" \
        "  --no-interpolate            Disable presentation interpolation" \
        "  --correctness-only          Do not fail on the separate real-time gate" \
        "  -h, --help                  Show this help" \
        "" \
        "Interactive mode must run inside tmux. Example:" \
        "  tmux new -s matrix-bfm-isaac" \
        "  bash scripts/run_matrix_bfm_isaac_guarded.sh interactive --profile trna"
}

if [[ "$MODE" == "help" ]]; then
    usage
    exit 0
fi

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --scene) SCENE_ID="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --schedule) SCHEDULE="$2"; shift 2 ;;
        --offscreen) OFFSCREEN=1; shift ;;
        --onscreen) OFFSCREEN=0; shift ;;
        --runtime-root) RUNTIME_ROOT_OVERRIDE="$2"; shift 2 ;;
        --runtime-python) RUNTIME_PYTHON_OVERRIDE="$2"; shift 2 ;;
        --runtime-config) RUNTIME_CONFIG_OVERRIDE="$2"; shift 2 ;;
        --teacher-profile) TEACHER_PROFILE_OVERRIDE="$2"; shift 2 ;;
        --physics-asset-root) PHYSICS_ASSET_ROOT_OVERRIDE="$2"; shift 2 ;;
        --collision-root) COLLISION_ROOT_OVERRIDE="$2"; shift 2 ;;
        --visual-urdf) VISUAL_URDF_OVERRIDE="$2"; shift 2 ;;
        --run-dir) RUN_DIR_OVERRIDE="$2"; shift 2 ;;
        --no-interpolate) NO_INTERPOLATE=1; shift ;;
        --correctness-only) CORRECTNESS_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "${MATRIX_BFM_ISAAC_GUARDED:-0}" != "1" \
    || ! "${MATRIX_BFM_CLEAN_RUN_NONCE:-}" =~ ^[A-Za-z0-9._-]{16,128}$ ]]; then
    echo "[ERROR] Direct launch is disabled; use the guarded entry point:" >&2
    echo "[ERROR] bash scripts/run_matrix_bfm_isaac_guarded.sh $MODE ..." >&2
    exit 2
fi

if [[ "$MODE" == "interactive" && -z "${TMUX:-}" ]]; then
    echo "[ERROR] Interactive Matrix BFM/Isaac must run inside tmux" >&2
    echo "[ERROR] tmux new -s matrix-bfm-isaac" >&2
    exit 2
fi
if [[ ! "$SCENE_ID" =~ ^[0-9]+$ ]]; then
    echo "[ERROR] --scene must be an integer" >&2
    exit 2
fi
if ! python3 - "$DURATION" <<'PY'
import math
import sys
try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(value) and value > 0.0 else 1)
PY
then
    echo "[ERROR] --duration must be positive and finite" >&2
    exit 2
fi
if [[ -z "$SCHEDULE" || "$SCHEDULE" == *$'\n'* ]]; then
    echo "[ERROR] --schedule must be a non-empty single line" >&2
    exit 2
fi
if [[ "$OFFSCREEN" == "auto" ]]; then
    if [[ "$MODE" == "smoke" ]]; then OFFSCREEN=1; else OFFSCREEN=0; fi
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
if [[ -n "${MATRIX_UE_EXTRA_EXEC_CMDS:-}" ]]; then
    echo "[ERROR] Qualified BFM/Isaac rejects MATRIX_UE_EXTRA_EXEC_CMDS" >&2
    echo "[ERROR] Use only the bounded MATRIX_BFM_ISAAC_* video overrides" >&2
    exit 2
fi

LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-bfm-isaac.lock.json"
PATH_GUARD="$SCRIPT_DIR/matrix_bfm_isaac_path_guard.py"
guard_path() {
    local label="$1"
    local mode="$2"
    local candidate="$3"
    local resolved
    if ! resolved="$(python3 "$PATH_GUARD" --mode "$mode" "$candidate" 2>&1)"; then
        echo "[ERROR] Unsafe $label: $resolved" >&2
        exit 2
    fi
    printf '%s\n' "$resolved"
}

readarray -t MATERIAL_BRIDGE_CONTRACT < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys

bridge = json.load(open(sys.argv[1], encoding="utf-8"))["ue_material_bridge"]
for key in (
    "relative_path",
    "sha256",
    "ue_binary_relative_path",
    "ue_binary_build_id",
):
    print(bridge[key])
PY
)
if [[ "${#MATERIAL_BRIDGE_CONTRACT[@]}" != "4" ]]; then
    echo "[ERROR] Could not read the UE material bridge contract" >&2
    exit 1
fi
MATERIAL_BRIDGE_PATH="$PROJECT_ROOT/${MATERIAL_BRIDGE_CONTRACT[0]}"
MATERIAL_BRIDGE_SHA256="${MATERIAL_BRIDGE_CONTRACT[1]}"
MATERIAL_UE_BINARY="$PROJECT_ROOT/${MATERIAL_BRIDGE_CONTRACT[2]}"
MATERIAL_UE_BUILD_ID="${MATERIAL_BRIDGE_CONTRACT[3]}"
MATERIAL_BRIDGE_PATH="$(guard_path material-bridge subtree "$MATERIAL_BRIDGE_PATH")"
MATERIAL_UE_BINARY="$(guard_path matrix-ue-binary subtree "$MATERIAL_UE_BINARY")"
if [[ -v MATRIX_UE_MATERIAL_FIX_PRELOAD \
    && "$MATRIX_UE_MATERIAL_FIX_PRELOAD" != "$MATERIAL_BRIDGE_PATH" ]]; then
    echo "[ERROR] Qualified BFM/Isaac rejects material bridge overrides" >&2
    echo "[ERROR] expected=$MATERIAL_BRIDGE_PATH actual=$MATRIX_UE_MATERIAL_FIX_PRELOAD" >&2
    exit 2
fi
MATRIX_UE_BINARY="$MATERIAL_UE_BINARY" \
    bash "$SCRIPT_DIR/build_matrix_ue_material_fix.sh" \
    --output "$MATERIAL_BRIDGE_PATH" \
    --expected-sha256 "$MATERIAL_BRIDGE_SHA256" \
    --expected-ue-build-id "$MATERIAL_UE_BUILD_ID" \
    --verify-only

RUNTIME_ROOT="${RUNTIME_ROOT_OVERRIDE:-${MATRIX_BFM_ISAAC_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac-sync-world16-v1/bfm_runtime}}"
RUNTIME_PYTHON="${RUNTIME_PYTHON_OVERRIDE:-${MATRIX_BFM_ISAAC_PYTHON:-}}"
RUNTIME_CONFIG="${RUNTIME_CONFIG_OVERRIDE:-${MATRIX_BFM_ISAAC_CONFIG:-$RUNTIME_ROOT/configs/alienware/moon-matrix.toml}}"
TEACHER_PROFILE="${TEACHER_PROFILE_OVERRIDE:-${MATRIX_BFM_ISAAC_TEACHER_PROFILE:-}}"
PHYSICS_ASSET_ROOT="${PHYSICS_ASSET_ROOT_OVERRIDE:-${MATRIX_BFM_ISAAC_PHYSICS_ASSET_ROOT:-}}"
COLLISION_ROOT="${COLLISION_ROOT_OVERRIDE:-${MATRIX_BFM_ISAAC_COLLISION_ROOT:-}}"
BFM_SOURCE_ROOT="${MATRIX_BFM_ISAAC_SOURCE_ROOT:-}"
MATRIX_NATIVE_RUNTIME="${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2}"
VISUAL_ROOT="${MATRIX_BFM_ISAAC_VISUAL_ROOT:-$MATRIX_NATIVE_RUNTIME/g1-visual}"
VISUAL_URDF="${VISUAL_URDF_OVERRIDE:-${MATRIX_BFM_ISAAC_VISUAL_URDF:-$VISUAL_ROOT/g1_29dof.urdf}}"
for required_setting in \
    RUNTIME_PYTHON TEACHER_PROFILE PHYSICS_ASSET_ROOT COLLISION_ROOT \
    BFM_SOURCE_ROOT VISUAL_ROOT; do
    if [[ -z "${!required_setting}" ]]; then
        echo "[ERROR] $required_setting is unset; use --profile or an explicit override" >&2
        exit 2
    fi
done
RUNTIME_ROOT="$(guard_path runtime-root subtree "$RUNTIME_ROOT")"
RUNTIME_CONFIG="$(guard_path runtime-config subtree "$RUNTIME_CONFIG")"
TEACHER_PROFILE="$(guard_path teacher-profile subtree "$TEACHER_PROFILE")"
PHYSICS_ASSET_ROOT="$(guard_path physics-asset-root subtree "$PHYSICS_ASSET_ROOT")"
COLLISION_ROOT="$(guard_path collision-root subtree "$COLLISION_ROOT")"
BFM_SOURCE_ROOT="$(guard_path bfm-source-root subtree "$BFM_SOURCE_ROOT")"
VISUAL_ROOT="$(guard_path visual-root subtree "$VISUAL_ROOT")"
VISUAL_URDF="$(guard_path visual-urdf subtree "$VISUAL_URDF")"

for command_name in bwrap flock git realpath setsid ss stdbuf; do
    command -v "$command_name" >/dev/null || {
        echo "[ERROR] Required launch command is unavailable: $command_name" >&2
        exit 1
    }
done
[[ -x "$RUNTIME_PYTHON" ]] || {
    echo "[ERROR] Leo Isaac runtime Python is missing: $RUNTIME_PYTHON" >&2
    exit 1
}
git -C "$RUNTIME_ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1 || {
    echo "[ERROR] Frozen BFM runtime is missing: $RUNTIME_ROOT" >&2
    echo "[ERROR] Run scripts/bootstrap_matrix_bfm_isaac.sh first" >&2
    exit 1
}
for required_file in "$RUNTIME_CONFIG" "$TEACHER_PROFILE" "$VISUAL_URDF"; do
    [[ -f "$required_file" && ! -L "$required_file" ]] || {
        echo "[ERROR] Required launch file is missing or a symlink: $required_file" >&2
        exit 1
    }
done
if [[ "$VISUAL_URDF" != "$VISUAL_ROOT/g1_29dof.urdf" ]]; then
    echo "[ERROR] Matrix visual URDF is outside the verified visual closure:" >&2
    echo "[ERROR] expected=$VISUAL_ROOT/g1_29dof.urdf actual=$VISUAL_URDF" >&2
    exit 2
fi
[[ -d "$PHYSICS_ASSET_ROOT" ]] || {
    echo "[ERROR] Frozen G1 PhysX USD closure is missing: $PHYSICS_ASSET_ROOT" >&2
    exit 1
}
[[ -d "$COLLISION_ROOT" && ! -L "$COLLISION_ROOT" ]] || {
    echo "[ERROR] Frozen Moon collision closure is missing: $COLLISION_ROOT" >&2
    exit 1
}
readarray -t SCENE_CONTRACT < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys

scene = json.load(open(sys.argv[1], encoding="utf-8"))["scene_collision_contract"]
for key in (
    "scene_id",
    "runtime_config_suffix",
    "x_min_m",
    "x_max_m",
    "y_min_m",
    "y_max_m",
    "warning_margin_m",
    "stop_margin_m",
):
    print(scene[key])
PY
)
if [[ "${#SCENE_CONTRACT[@]}" != "8" ]]; then
    echo "[ERROR] Could not read the frozen Moon collision contract" >&2
    exit 1
fi
LOCKED_SCENE_ID="${SCENE_CONTRACT[0]}"
LOCKED_CONFIG="$RUNTIME_ROOT/${SCENE_CONTRACT[1]}"
if [[ "$SCENE_ID" != "$LOCKED_SCENE_ID" || "$RUNTIME_CONFIG" != "$LOCKED_CONFIG" ]]; then
    echo "[ERROR] This release is locked to Matrix scene $LOCKED_SCENE_ID and:" >&2
    echo "[ERROR] $LOCKED_CONFIG" >&2
    exit 2
fi
if ! "$RUNTIME_PYTHON" -I - \
    "$LOCK_FILE" "$RUNTIME_CONFIG" "$PHYSICS_ASSET_ROOT" "$COLLISION_ROOT" \
    "$BFM_SOURCE_ROOT" <<'PY'
import json
from pathlib import Path
import re
import sys

try:
    import tomllib
except ModuleNotFoundError:
    tomllib = None


def load_paths(text):
    if tomllib is not None:
        config = tomllib.loads(text)
        return config.get("paths")

    # Python 3.10 has no stdlib TOML reader.  This frozen closure check only
    # needs the string-valued [paths] table, so reject anything ambiguous in
    # that table and ignore unrelated runtime sections.
    paths = None
    in_paths = False
    seen_paths = False
    decoder = json.JSONDecoder()
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("["):
            match = re.fullmatch(
                r"\[\s*([A-Za-z0-9_-]+)\s*\]\s*(?:#.*)?",
                stripped,
            )
            if match is None:
                raise SystemExit(f"unsupported TOML table at line {line_number}")
            section = match.group(1)
            in_paths = section == "paths"
            if in_paths:
                if seen_paths:
                    raise SystemExit(f"duplicate paths table at line {line_number}")
                paths = {}
                seen_paths = True
            continue
        if not in_paths:
            continue
        if "=" not in stripped:
            raise SystemExit(f"invalid paths assignment at line {line_number}")
        key, encoded = (part.strip() for part in stripped.split("=", 1))
        if re.fullmatch(r"[A-Za-z0-9_-]+", key) is None or key in paths:
            raise SystemExit(f"invalid or duplicate paths key at line {line_number}")
        try:
            value, end = decoder.raw_decode(encoded)
        except json.JSONDecodeError as exc:
            raise SystemExit(f"invalid paths string at line {line_number}") from exc
        remainder = encoded[end:].strip()
        if not isinstance(value, str) or (remainder and not remainder.startswith("#")):
            raise SystemExit(f"unsupported paths value at line {line_number}")
        paths[key] = value
    return paths

lock = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
paths = load_paths(Path(sys.argv[2]).read_text(encoding="utf-8"))
physics_root = Path(sys.argv[3]).resolve(strict=True)
collision_root = Path(sys.argv[4]).resolve(strict=True)
bfm_source_root = Path(sys.argv[5]).resolve(strict=True)
if not isinstance(paths, dict):
    raise SystemExit("runtime config has no [paths] table")

expected = {
    "bfm_sonic_repo": bfm_source_root,
    "g1_usd": (physics_root / lock["physics_assets"]["main_usd"]).resolve(
        strict=True
    ),
    "scene_root": collision_root,
    "collision_usd": (
        collision_root / lock["scene_assets"]["collision_usd"]
    ).resolve(strict=True),
}
for key, expected_path in expected.items():
    value = paths.get(key)
    if not isinstance(value, str) or not value:
        raise SystemExit(f"runtime config paths.{key} is missing")
    actual_path = Path(value).resolve(strict=True)
    if actual_path != expected_path:
        raise SystemExit(
            f"runtime config paths.{key} does not use the verified closure: "
            f"expected={expected_path} actual={actual_path}"
        )
print("[PASS] Frozen runtime config is bound to the verified source/assets")
PY
then
    echo "[ERROR] Refusing to verify one source/asset closure and run another" >&2
    exit 2
fi
export MATRIX_BFM_ISAAC_COLLISION_BOUNDS="${SCENE_CONTRACT[2]} ${SCENE_CONTRACT[3]} ${SCENE_CONTRACT[4]} ${SCENE_CONTRACT[5]}"
export MATRIX_BFM_ISAAC_COLLISION_WARNING_MARGIN="${SCENE_CONTRACT[6]}"
export MATRIX_BFM_ISAAC_COLLISION_STOP_MARGIN="${SCENE_CONTRACT[7]}"

INSTANCE_ID="${MATRIX_INSTANCE_ID:-matrix-bfm-isaac}"
if [[ ! "$INSTANCE_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "[ERROR] MATRIX_INSTANCE_ID must match [A-Za-z0-9._-]+" >&2
    exit 2
fi
STATE_ROOT="$(guard_path state-root subtree "${MATRIX_BFM_ISAAC_STATE_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-bfm-isaac}")"
RUN_ROOT="$(guard_path run-root subtree "${MATRIX_BFM_ISAAC_RUN_ROOT:-$PROJECT_ROOT/outputs/runs/matrix-bfm-isaac}")"
VISUAL_VENV_CANDIDATE="${MATRIX_BFM_ISAAC_VISUAL_VENV:-$STATE_ROOT/visual-import-venv}"
if [[ -L "$VISUAL_VENV_CANDIDATE" ]]; then
    echo "[ERROR] Visual-import venv root must not be a symlink: $VISUAL_VENV_CANDIDATE" >&2
    exit 2
fi
VISUAL_VENV_ROOT="$(guard_path visual-import-venv subtree "$VISUAL_VENV_CANDIDATE")"
[[ -d "$VISUAL_VENV_ROOT" ]] || {
    echo "[ERROR] Locked visual-import venv is missing: $VISUAL_VENV_ROOT" >&2
    echo "[ERROR] Run scripts/bootstrap_matrix_bfm_isaac.sh first" >&2
    exit 1
}
INSTANCE_DIR="$STATE_ROOT/instances/$INSTANCE_ID"
INSTANCE_DIR="$(guard_path instance-dir subtree "$INSTANCE_DIR")"
mkdir -p "$INSTANCE_DIR"
chmod 700 "$INSTANCE_DIR"
INSTANCE_LOCK="$INSTANCE_DIR/instance.lock"
OWNER_LEDGER="$INSTANCE_DIR/owner-ledger.json"
LEDGER_TOOL="$SCRIPT_DIR/matrix_bfm_isaac_instance_ledger.py"
exec 9>"$INSTANCE_LOCK"
if ! flock -n 9; then
    echo "[ERROR] Matrix BFM/Isaac instance '$INSTANCE_ID' is already running" >&2
    exit 75
fi
python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
    --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" init --launcher-pid "$$"

RUN_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
RUN_DIR="${RUN_DIR_OVERRIDE:-$RUN_ROOT/${MODE}_${RUN_STAMP}_$$}"
RUN_DIR="$(guard_path run-dir subtree "$RUN_DIR")"
if [[ -e "$RUN_DIR" ]]; then
    echo "[ERROR] Refusing to reuse an existing evidence directory: $RUN_DIR" >&2
    exit 2
fi
IPC_ROOT="$(guard_path ipc-root subtree "${MATRIX_BFM_ISAAC_IPC_ROOT:-$STATE_ROOT/ipc}")"
IPC_DIR="$(guard_path ipc-dir subtree "$IPC_ROOT/$$")"
guard_socket_path() {
    local label="$1"
    local candidate="$2"
    local resolved
    if ! resolved="$(python3 "$PATH_GUARD" --mode subtree --unix-socket \
        "$candidate" 2>&1)"; then
        echo "[ERROR] Unsafe $label: $resolved" >&2
        exit 2
    fi
    printf '%s\n' "$resolved"
}
STATE_SOCKET="$(guard_socket_path state-socket "$IPC_DIR/s")"
KEYBOARD_SOCKET="$(guard_socket_path keyboard-socket "$IPC_DIR/k")"
RELAY_STATUS="$RUN_DIR/relay-status.json"
RENDERER_NAMESPACE_PID_FILE="$RUN_DIR/renderer-namespace.pid"
CHECKOUT_SNAPSHOT_DIR="$RUN_DIR/checkout-source-snapshot"
RUNTIME_REPORT="$RUN_DIR/runtime-report.json"
TRAJECTORY="$RUN_DIR/trajectory.npz"
ACCEPTANCE_REPORT="$RUN_DIR/acceptance.json"
PREFLIGHT_REPORT="$RUN_DIR/preflight.json"
FINALIZER_STATUS="$RUN_DIR/finalizer-status.json"
RESOLVED_VIDEO_SETTINGS="$RUN_DIR/resolved-video-settings.json"
mkdir -p "$RUN_DIR" "$IPC_DIR"
chmod 700 "$RUN_DIR" "$IPC_DIR"

VIDEO_SETTINGS_TEMP="$RUN_DIR/.resolved-video-settings.json.tmp.$$"
if ! python3 -I "$SCRIPT_DIR/matrix_bfm_isaac_video_settings.py" \
    --file "$PROJECT_ROOT/config/runtime/matrix-bfm-isaac-video-settings.json" \
    --format json > "$VIDEO_SETTINGS_TEMP"; then
    rm -f -- "$VIDEO_SETTINGS_TEMP"
    echo "[ERROR] Could not resolve the locked BFM/Isaac video settings" >&2
    exit 2
fi
chmod 600 "$VIDEO_SETTINGS_TEMP"
mv -- "$VIDEO_SETTINGS_TEMP" "$RESOLVED_VIDEO_SETTINGS"

VERIFY_BASE=(
    "$SCRIPT_DIR/verify_matrix_bfm_isaac_runtime.py"
    --lock "$LOCK_FILE"
    --matrix-root "$PROJECT_ROOT"
    --runtime-root "$RUNTIME_ROOT"
    --runtime-python "$RUNTIME_PYTHON"
    --physics-asset-root "$PHYSICS_ASSET_ROOT"
    --collision-root "$COLLISION_ROOT"
    --teacher-profile "$TEACHER_PROFILE"
    --visual-venv "$VISUAL_VENV_ROOT"
    --matrix-visual-root "$VISUAL_ROOT"
    --material-bridge "$MATERIAL_BRIDGE_PATH"
)
python3 "${VERIFY_BASE[@]}"

export PYTHONPATH="$RUNTIME_ROOT/src:$BFM_SOURCE_ROOT/imports/SONIC:$BFM_SOURCE_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OMNI_KIT_ACCEPT_EULA=YES
export ACCEPT_EULA=Y
export MATRIX_BFM_TEACHER_PROFILE_FILE="$TEACHER_PROFILE"
export MATRIX_BFM_ISAAC_VISUAL_VENV="$VISUAL_VENV_ROOT"
export MATRIX_CUSTOM_URDF_PYTHON="$SCRIPT_DIR/run_matrix_bfm_visual_python.sh"
TMPDIR="$(guard_path temporary-dir subtree "$RUN_DIR/tmp")"
XDG_CACHE_HOME="$(guard_path cache-root subtree "${MATRIX_BFM_ISAAC_CACHE_ROOT:-$PROJECT_ROOT/outputs/cache/matrix-bfm-isaac}")"
XDG_CONFIG_HOME="$(guard_path config-root subtree "${MATRIX_BFM_ISAAC_CONFIG_ROOT:-$PROJECT_ROOT/outputs/config/matrix-bfm-isaac}")"
export TMPDIR XDG_CACHE_HOME XDG_CONFIG_HOME
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME"

echo "[INFO] Running frozen BFM preflight"
"$RUNTIME_PYTHON" -m bfm_sonic_realscan_play.cli \
    --config "$RUNTIME_CONFIG" preflight --report "$PREFLIGHT_REPORT" \
    > "$RUN_DIR/preflight.log" 2>&1

RENDERER_PID=""
PHYSICS_PID=""
KEYBOARD_PID=""
WAIT_CHILD_EXIT_CODE=""
RENDERER_FINAL_EXIT_CODE=""
CLEANUP_STARTED=0
SHUTDOWN_SIGNAL=""
FORCE_REQUESTED=0

register_owned_group() {
    local pid="$1"
    local label="$2"
    if python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
        --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" add --pid "$pid" >/dev/null; then
        return 0
    fi
    echo "[ERROR] Could not register $label PID $pid; terminating fail-safe" >&2
    python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
        --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" terminate-unregistered \
        --pid "$pid" >/dev/null 2>&1 || true
    wait "$pid" 2>/dev/null || true
    return 1
}

managed_child_running() {
    local expected_pid="$1"
    local child_pid
    while IFS= read -r child_pid; do
        [[ "$child_pid" == "$expected_pid" ]] && return 0
    done < <(jobs -pr)
    return 1
}

stop_child() {
    local pid="$1"
    local label="$2"
    local grace_s="$3"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
    if ! managed_child_running "$pid"; then
        wait "$pid" 2>/dev/null || true
        return 0
    fi
    kill -TERM "$pid" 2>/dev/null || true
    local attempts
    attempts="$(python3 - "$grace_s" <<'PY'
import math
import sys
print(max(1, math.ceil(float(sys.argv[1]) * 10)))
PY
)"
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        managed_child_running "$pid" || break
        sleep 0.1
    done
    if managed_child_running "$pid"; then
        echo "[WARN] $label did not stop after ${grace_s}s; sending exact KILL" >&2
        kill -KILL "$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
}

wait_child_exit() {
    local pid="$1"
    local grace_s="$2"
    local attempts
    attempts="$(python3 - "$grace_s" <<'PY'
import math
import sys
print(max(1, math.ceil(float(sys.argv[1]) * 10)))
PY
)"
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        managed_child_running "$pid" || {
            if wait "$pid" 2>/dev/null; then
                WAIT_CHILD_EXIT_CODE=0
            else
                WAIT_CHILD_EXIT_CODE=$?
            fi
            return 0
        }
        sleep 0.1
    done
    return 1
}

stop_renderer() {
    local pid="$1"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
    RENDERER_FINAL_EXIT_CODE=""
    local namespace_pid=""
    if [[ -f "$RENDERER_NAMESPACE_PID_FILE" \
        && ! -L "$RENDERER_NAMESPACE_PID_FILE" ]]; then
        namespace_pid="$(<"$RENDERER_NAMESPACE_PID_FILE")"
    fi
    if [[ "$namespace_pid" =~ ^[1-9][0-9]*$ ]] \
        && python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
            --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" signal-pid \
            --pid "$namespace_pid" --signal TERM >/dev/null 2>&1; then
        if wait_child_exit "$pid" 25; then
            RENDERER_FINAL_EXIT_CODE="$WAIT_CHILD_EXIT_CODE"
            return 0
        fi
        echo "[WARN] Renderer namespace did not finish graceful cleanup" >&2
    fi
    stop_child "$pid" renderer 20
}

wait_for_nonempty_file() {
    local path="$1"
    local attempts="$2"
    local attempt
    for ((attempt = 0; attempt < attempts; attempt++)); do
        [[ -s "$path" ]] && return 0
        sleep 0.1
    done
    return 1
}

cleanup() {
    trap '' INT TERM HUP
    if [[ "$CLEANUP_STARTED" == "1" ]]; then
        return
    fi
    CLEANUP_STARTED=1
    stop_child "$KEYBOARD_PID" keyboard 3
    stop_child "$PHYSICS_PID" physics 5
    # run_sim owns UE through its exact supervisor and may need its bounded
    # child cleanup window; no process-name-wide cleanup is used in this mode.
    stop_renderer "$RENDERER_PID"
    rm -f -- "$STATE_SOCKET" "$KEYBOARD_SOCKET" \
        "$RENDERER_NAMESPACE_PID_FILE"
    rmdir -- "$IPC_DIR" 2>/dev/null || true
    if [[ -f "$OWNER_LEDGER" ]]; then
        python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
            --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" signal --signal TERM \
            >/dev/null 2>&1 || true
        sleep 0.2
        python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
            --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" signal --signal KILL \
            >/dev/null 2>&1 || true
        if python3 "$LEDGER_TOOL" --path "$OWNER_LEDGER" \
            --nonce "$MATRIX_BFM_CLEAN_RUN_NONCE" verify-empty \
            >/dev/null 2>&1; then
            rm -f -- "$OWNER_LEDGER"
        else
            echo "[ERROR] Owned processes remain; preserving $OWNER_LEDGER" >&2
        fi
    fi
}
trap cleanup EXIT
request_shutdown() {
    local signal_name="$1"
    if [[ -n "$SHUTDOWN_SIGNAL" ]]; then
        FORCE_REQUESTED=1
        return
    fi
    SHUTDOWN_SIGNAL="$signal_name"
    echo "[WARN] $signal_name received; requesting BFM finalizer" >&2
}
trap 'request_shutdown INT' INT
trap 'request_shutdown TERM' TERM
trap 'request_shutdown HUP' HUP

abort_startup_if_shutdown_requested() {
    [[ -n "$SHUTDOWN_SIGNAL" ]] || return 0
    local exit_code=128
    case "$SHUTDOWN_SIGNAL" in
        INT) exit_code=130 ;;
        TERM) exit_code=143 ;;
        HUP) exit_code=129 ;;
    esac
    echo "[WARN] Aborting startup after $SHUTDOWN_SIGNAL; Isaac was not started" >&2
    exit "$exit_code"
}

echo "[INFO] Starting canonical Matrix G1 renderer in external-state mode"
export MATRIX_BFM_ISAAC_STATE_SOCKET="$STATE_SOCKET"
export MATRIX_BFM_ISAAC_KEYBOARD_SOCKET="$KEYBOARD_SOCKET"
export MATRIX_BFM_ISAAC_RELAY_STATUS="$RELAY_STATUS"
export MATRIX_BFM_ISAAC_RELAY_LOG="$RUN_DIR/relay.log"
export MATRIX_BFM_ISAAC_BOOTSTRAP_STATE="$PROJECT_ROOT/config/runtime/matrix-bfm-isaac-bootstrap-state.json"
export MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE="$RENDERER_NAMESPACE_PID_FILE"
export MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR="$CHECKOUT_SNAPSHOT_DIR"
export MATRIX_BFM_ISAAC_NO_INTERPOLATE="$NO_INTERPOLATE"
if ! BFM_VIDEO_LINES="$(
    python3 -I - "$RESOLVED_VIDEO_SETTINGS" <<'PY'
import json
from pathlib import Path
import sys

value = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
for field in (
    "resolution_width",
    "resolution_height",
    "window_mode",
    "fps_limit",
    "quality",
    "camera_smoothing",
    "screen_percentage",
):
    print(value[field])
PY
)"; then
    echo "[ERROR] Could not resolve the locked BFM/Isaac video settings" >&2
    exit 2
fi
mapfile -t BFM_VIDEO_FIELDS <<<"$BFM_VIDEO_LINES"
if [[ "${#BFM_VIDEO_FIELDS[@]}" != "7" ]]; then
    echo "[ERROR] Invalid BFM/Isaac video settings helper output" >&2
    exit 2
fi
BFM_VIDEO_WIDTH="${BFM_VIDEO_FIELDS[0]}"
BFM_VIDEO_HEIGHT="${BFM_VIDEO_FIELDS[1]}"
BFM_VIDEO_WINDOW_MODE="${BFM_VIDEO_FIELDS[2]}"
BFM_VIDEO_FPS_LIMIT="${BFM_VIDEO_FIELDS[3]}"
BFM_VIDEO_QUALITY="${BFM_VIDEO_FIELDS[4]}"
BFM_VIDEO_CAMERA_SMOOTHING="${BFM_VIDEO_FIELDS[5]}"
BFM_VIDEO_SCREEN_PERCENTAGE="${BFM_VIDEO_FIELDS[6]}"
echo "[INFO] BFM renderer video: ${BFM_VIDEO_WIDTH}x${BFM_VIDEO_HEIGHT}" \
    "mode=$BFM_VIDEO_WINDOW_MODE fps=$BFM_VIDEO_FPS_LIMIT" \
    "quality=$BFM_VIDEO_QUALITY smoothing=$BFM_VIDEO_CAMERA_SMOOTHING" \
    "screen_percentage=$BFM_VIDEO_SCREEN_PERCENTAGE"

# The BFM renderer has a tracked video contract independent from the ordinary
# Matrix settings panel.  Strip every generic applied value at the process
# boundary, then pass only validated BFM-specific fields to run_sim.sh.
setsid env \
    -u MATRIX_VIDEO_SETTINGS_FILE \
    -u MATRIX_VIDEO_APPLIED_WIDTH \
    -u MATRIX_VIDEO_APPLIED_HEIGHT \
    -u MATRIX_VIDEO_APPLIED_WINDOW_MODE \
    -u MATRIX_VIDEO_APPLIED_FPS_LIMIT \
    -u MATRIX_VIDEO_APPLIED_QUALITY \
    -u MATRIX_VIDEO_APPLIED_CAMERA_SMOOTHING \
    -u MATRIX_VIDEO_APPLIED_REVISION \
    -u MATRIX_VIDEO_APPLIED_JSON \
    -u MATRIX_UE_EXTRA_EXEC_CMDS \
    -u MATRIX_UE_MATERIAL_FIX_PRELOAD \
    MATRIX_BFM_ISAAC_RENDERER_VIDEO_LOCKED=1 \
    MATRIX_BFM_ISAAC_RENDER_WIDTH="$BFM_VIDEO_WIDTH" \
    MATRIX_BFM_ISAAC_RENDER_HEIGHT="$BFM_VIDEO_HEIGHT" \
    MATRIX_BFM_ISAAC_RENDER_WINDOW_MODE="$BFM_VIDEO_WINDOW_MODE" \
    MATRIX_BFM_ISAAC_RENDER_FPS_LIMIT="$BFM_VIDEO_FPS_LIMIT" \
    MATRIX_BFM_ISAAC_RENDER_QUALITY="$BFM_VIDEO_QUALITY" \
    MATRIX_BFM_ISAAC_RENDER_CAMERA_SMOOTHING="$BFM_VIDEO_CAMERA_SMOOTHING" \
    MATRIX_BFM_ISAAC_RENDER_SCREEN_PERCENTAGE="$BFM_VIDEO_SCREEN_PERCENTAGE" \
    MATRIX_UE_MATERIAL_FIX_PRELOAD="$MATERIAL_BRIDGE_PATH" \
    bash "$SCRIPT_DIR/run_matrix_bfm_isaac_renderer_isolated.sh" \
    custom "$SCENE_ID" "$OFFSCREEN" 0 0 "$VISUAL_URDF" g1_29dof \
    > "$RUN_DIR/renderer.log" 2>&1 &
RENDERER_PID=$!
register_owned_group "$RENDERER_PID" renderer-namespace
for _attempt in $(seq 1 1200); do
    abort_startup_if_shutdown_requested
    [[ -S "$STATE_SOCKET" ]] && break
    managed_child_running "$RENDERER_PID" || {
        echo "[ERROR] Isolated Matrix renderer/relay exited during startup" >&2
        exit 4
    }
    sleep 0.1
done
[[ -S "$STATE_SOCKET" ]] || {
    echo "[ERROR] Isolated Matrix state relay did not create $STATE_SOCKET" >&2
    exit 4
}
if [[ ! -s "$RENDERER_NAMESPACE_PID_FILE" \
    || -L "$RENDERER_NAMESPACE_PID_FILE" ]]; then
    echo "[ERROR] Renderer namespace did not publish its owned PID" >&2
    exit 4
fi
abort_startup_if_shutdown_requested

PHYSICS_ARGS=(
    --config "$RUNTIME_CONFIG"
    --report "$RUNTIME_REPORT"
    --trajectory "$TRAJECTORY"
    --matrix-state-socket "$STATE_SOCKET"
    --no-nurec
)
if [[ "$MODE" == "interactive" ]]; then
    PHYSICS_ARGS+=(
        --interactive
        --control-only
        --keyboard-unix-socket "$KEYBOARD_SOCKET"
        --duration "$DURATION"
    )
else
    PHYSICS_ARGS+=(--schedule "$SCHEDULE")
fi

echo "[INFO] Starting frozen Leo BFM/Isaac control loop"
setsid "$RUNTIME_PYTHON" "$RUNTIME_ROOT/scripts/run_g1_teacher_closed_loop.py" \
    "${PHYSICS_ARGS[@]}" > "$RUN_DIR/physics.log" 2>&1 &
PHYSICS_PID=$!
register_owned_group "$PHYSICS_PID" physics
if [[ "$MODE" == "interactive" ]]; then
    setsid python3 -u "$SCRIPT_DIR/matrix_bfm_isaac_keyboard.py" \
        --socket "$KEYBOARD_SOCKET" \
        --display "${DISPLAY:-:0}" \
        --xauthority "${XAUTHORITY:-/run/user/${UID}/gdm/Xauthority}" \
        --allowed-process-root \
            "$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux" \
        --camera-look-backend "${MATRIX_CAMERA_ARROW_BACKEND:-xtest}" \
        --camera-look-pixels-per-second \
            "${MATRIX_CAMERA_ARROW_PIXELS_PER_SECOND:-600}" \
        > "$RUN_DIR/keyboard.log" 2>&1 &
    KEYBOARD_PID=$!
    register_owned_group "$KEYBOARD_PID" keyboard
fi

request_physics_finalizer() {
    if [[ "$MODE" != "interactive" || ! -S "$KEYBOARD_SOCKET" ]]; then
        return 1
    fi
    python3 "$SCRIPT_DIR/matrix_bfm_isaac_command.py" \
        --socket "$KEYBOARD_SOCKET" --key SPACE --key ESCAPE
}

wait_for_physics_finalizer() {
    local timeout_s="${MATRIX_BFM_ISAAC_FINALIZER_TIMEOUT_S:-30}"
    local deadline
    deadline="$(python3 - "$timeout_s" <<'PY'
import math
import sys
import time

timeout = float(sys.argv[1])
if not math.isfinite(timeout) or timeout <= 0.0:
    raise SystemExit(2)
print(time.monotonic() + timeout)
PY
)" || {
        echo "[ERROR] MATRIX_BFM_ISAAC_FINALIZER_TIMEOUT_S must be positive and finite" >&2
        return 2
    }
    while managed_child_running "$PHYSICS_PID"; do
        if [[ "$FORCE_REQUESTED" == "1" ]] \
            || ! python3 - "$deadline" <<'PY'
import sys
import time
raise SystemExit(0 if time.monotonic() < float(sys.argv[1]) else 1)
PY
        then
            return 124
        fi
        sleep 0.1
    done
    return 0
}

PHYSICS_EXIT_CODE=127
STACK_FAILURE_CODE=0
FINALIZER_TRIGGER="natural"
WAIT_PIDS=("$PHYSICS_PID" "$RENDERER_PID")
if [[ -n "$KEYBOARD_PID" ]]; then
    WAIT_PIDS+=("$KEYBOARD_PID")
fi

set +e
COMPLETED_PID=""
if [[ -n "$SHUTDOWN_SIGNAL" ]]; then
    # A signal can be latched after the last readiness check but before wait
    # starts.  Do not enter an otherwise indefinite wait for a second signal.
    FIRST_EXIT_CODE=128
else
    wait -n -p COMPLETED_PID "${WAIT_PIDS[@]}"
    FIRST_EXIT_CODE=$?
fi
set -e

if [[ -n "$SHUTDOWN_SIGNAL" && -z "$COMPLETED_PID" ]]; then
    FINALIZER_TRIGGER="signal_${SHUTDOWN_SIGNAL,,}"
    if request_physics_finalizer && wait_for_physics_finalizer; then
        set +e
        wait "$PHYSICS_PID"
        PHYSICS_EXIT_CODE=$?
        set -e
        PHYSICS_PID=""
    else
        echo "[ERROR] BFM finalizer did not complete after $SHUTDOWN_SIGNAL" >&2
        stop_child "$PHYSICS_PID" physics 5
        PHYSICS_PID=""
        PHYSICS_EXIT_CODE=124
    fi
    STACK_FAILURE_CODE=128
    case "$SHUTDOWN_SIGNAL" in
        INT) STACK_FAILURE_CODE=130 ;;
        TERM) STACK_FAILURE_CODE=143 ;;
        HUP) STACK_FAILURE_CODE=129 ;;
    esac
elif [[ "$COMPLETED_PID" == "$PHYSICS_PID" ]]; then
    PHYSICS_EXIT_CODE="$FIRST_EXIT_CODE"
    PHYSICS_PID=""
elif [[ "$COMPLETED_PID" == "$RENDERER_PID" ]]; then
    RENDERER_PID=""
    FINALIZER_TRIGGER="renderer_exit"
    STACK_FAILURE_CODE=4
    echo "[ERROR] Matrix renderer exited before the BFM runtime (code $FIRST_EXIT_CODE)" >&2
    if request_physics_finalizer && wait_for_physics_finalizer; then
        set +e
        wait "$PHYSICS_PID"
        PHYSICS_EXIT_CODE=$?
        set -e
    else
        stop_child "$PHYSICS_PID" physics 5
        PHYSICS_EXIT_CODE=124
    fi
    PHYSICS_PID=""
elif [[ -n "$KEYBOARD_PID" && "$COMPLETED_PID" == "$KEYBOARD_PID" ]]; then
    KEYBOARD_PID=""
    FINALIZER_TRIGGER="keyboard_exit"
    STACK_FAILURE_CODE=4
    echo "[ERROR] Keyboard bridge exited before the BFM runtime (code $FIRST_EXIT_CODE)" >&2
    if request_physics_finalizer && wait_for_physics_finalizer; then
        set +e
        wait "$PHYSICS_PID"
        PHYSICS_EXIT_CODE=$?
        set -e
    else
        stop_child "$PHYSICS_PID" physics 5
        PHYSICS_EXIT_CODE=124
    fi
    PHYSICS_PID=""
else
    echo "[ERROR] Could not identify the first completed Matrix child" >&2
    STACK_FAILURE_CODE=4
    stop_child "$PHYSICS_PID" physics 5
    PHYSICS_PID=""
fi

# Let the relay drain the last Unix datagram before writing its final status.
sleep 0.25
stop_child "$KEYBOARD_PID" keyboard 3
KEYBOARD_PID=""
stop_renderer "$RENDERER_PID"
RENDERER_PID=""
# The namespace normally exits 143 after the exact TERM used for ordered UE and
# relay teardown. Exit 70 is reserved for source-snapshot restoration failure;
# retain it even when the helper already removed the snapshot and only its
# final durability fsync failed.
if [[ "$RENDERER_FINAL_EXIT_CODE" == "70" ]]; then
    echo "[ERROR] Renderer namespace reported Matrix source restoration failure" >&2
    STACK_FAILURE_CODE=70
fi
# wait_child_exit intentionally tolerates the renderer's non-zero status after
# an operator-driven TERM, so the namespace's restore failure must also have a
# durable outer-process witness. A dangling symlink does not satisfy -e; reject
# both existing paths and symlinks before an otherwise successful acceptance.
if [[ -e "$CHECKOUT_SNAPSHOT_DIR" || -L "$CHECKOUT_SNAPSHOT_DIR" ]]; then
    echo "[ERROR] Renderer left an unretired Matrix source snapshot:" \
        "$CHECKOUT_SNAPSHOT_DIR" >&2
    STACK_FAILURE_CODE=70
fi
# The namespace performs ordered relay then UE cleanup. Bound the relay's
# atomic final-status handoff before evaluating the evidence set.
wait_for_nonempty_file "$RELAY_STATUS" 50 || true

REPORT_PRESENT=0
TRAJECTORY_PRESENT=0
RELAY_PRESENT=0
[[ -s "$RUNTIME_REPORT" ]] && REPORT_PRESENT=1
[[ -s "$TRAJECTORY" ]] && TRAJECTORY_PRESENT=1
[[ -s "$RELAY_STATUS" ]] && RELAY_PRESENT=1
FINALIZER_COMPLETE=0
if [[ "$REPORT_PRESENT" == "1" \
    && "$TRAJECTORY_PRESENT" == "1" \
    && "$RELAY_PRESENT" == "1" ]]; then
    FINALIZER_COMPLETE=1
fi
python3 - "$FINALIZER_STATUS" "$FINALIZER_TRIGGER" \
    "$PHYSICS_EXIT_CODE" "$STACK_FAILURE_CODE" "$FINALIZER_COMPLETE" \
    "$REPORT_PRESENT" "$TRAJECTORY_PRESENT" "$RELAY_PRESENT" <<'PY'
import json
import os
from pathlib import Path
import sys

path = Path(sys.argv[1])
payload = {
    "schema": "matrix_bfm_isaac_finalizer_status.v1",
    "trigger": sys.argv[2],
    "physics_exit_code": int(sys.argv[3]),
    "stack_failure_code": int(sys.argv[4]),
    "complete": sys.argv[5] == "1",
    "report_present": sys.argv[6] == "1",
    "trajectory_present": sys.argv[7] == "1",
    "relay_status_present": sys.argv[8] == "1",
}
temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
os.replace(temporary, path)
PY
if [[ "$FINALIZER_COMPLETE" != "1" ]]; then
    echo "[ERROR] Runtime did not produce report/trajectory/relay evidence" >&2
    exit 1
fi
ACCEPTANCE_ARGS=(
    "${VERIFY_BASE[@]}"
    --report "$RUNTIME_REPORT"
    --relay-status "$RELAY_STATUS"
    --video-settings "$RESOLVED_VIDEO_SETTINGS"
    --output "$ACCEPTANCE_REPORT"
)
if [[ "$CORRECTNESS_ONLY" == "1" ]]; then
    ACCEPTANCE_ARGS+=(--correctness-only)
fi
set +e
python3 "${ACCEPTANCE_ARGS[@]}" | tee "$RUN_DIR/acceptance.log"
ACCEPTANCE_EXIT_CODE=${PIPESTATUS[0]}
set -e

echo "[INFO] Matrix BFM/Isaac evidence: $RUN_DIR"
echo "[INFO] Correctness and real-time gates: $ACCEPTANCE_REPORT"
if [[ "$STACK_FAILURE_CODE" != "0" ]]; then
    echo "[ERROR] Matrix stack ended through $FINALIZER_TRIGGER" >&2
    exit "$STACK_FAILURE_CODE"
fi
if [[ "$PHYSICS_EXIT_CODE" != "0" ]]; then
    echo "[ERROR] Frozen BFM runtime exited with code $PHYSICS_EXIT_CODE" >&2
    exit "$PHYSICS_EXIT_CODE"
fi
exit "$ACCEPTANCE_EXIT_CODE"
