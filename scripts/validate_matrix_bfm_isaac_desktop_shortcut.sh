#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
DESKTOP_FILE="${MATRIX_BFM_ISAAC_DESKTOP_FILE:-${HOME:?}/Desktop/matrix-sonic.desktop}"
SESSION_NAME="${MATRIX_BFM_ISAAC_DESKTOP_SESSION_NAME:-matrix-bfm-isaac-mainline-${UID}}"
READY_TIMEOUT_SECONDS="${MATRIX_BFM_ISAAC_DESKTOP_READY_TIMEOUT_S:-240}"
SETTLE_SECONDS="${MATRIX_BFM_ISAAC_DESKTOP_SETTLE_S:-10}"
RECORDER_GUARD=1
PROFILE_EXPECTED=""

usage() {
    cat <<'EOF'
Usage: bash scripts/validate_matrix_bfm_isaac_desktop_shortcut.sh [options]

Launches the installed Matrix desktop shortcut through the Desktop Entry path,
then waits until the live tmux/run artifacts prove the game is ready for use.

Options:
  --desktop-file PATH       Desktop shortcut (default: ~/Desktop/matrix-sonic.desktop)
  --session-name NAME       Expected desktop tmux session
  --profile PROFILE         Require X-Matrix-Profile from the shortcut
  --ready-timeout SEC       Seconds to wait for live readiness (default: 240)
  --settle-seconds SEC      Extra live hold after readiness (default: 10)
  --skip-recorder-guard     Allow launch even if a known recorder is active
  -h, --help                Show this help
EOF
}

die() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 2
}

fail() {
    printf '[FAIL] %s\n' "$*" >&2
    exit 1
}

require_value() {
    local option="$1"
    local count="$2"
    ((count >= 2)) || die "$option requires a value"
}

validate_identifier() {
    local label="$1"
    local value="$2"
    [[ "$value" =~ ^[A-Za-z0-9._-]+$ ]] \
        || die "$label must match [A-Za-z0-9._-]+"
}

validate_positive_integer() {
    local label="$1"
    local value="$2"
    [[ "$value" =~ ^[1-9][0-9]*$ ]] || die "$label must be a positive integer"
}

while (($# > 0)); do
    case "$1" in
        --desktop-file)
            require_value "$1" "$#"
            DESKTOP_FILE="$2"
            shift 2
            ;;
        --session-name)
            require_value "$1" "$#"
            SESSION_NAME="$2"
            shift 2
            ;;
        --profile)
            require_value "$1" "$#"
            PROFILE_EXPECTED="$2"
            shift 2
            ;;
        --ready-timeout)
            require_value "$1" "$#"
            READY_TIMEOUT_SECONDS="$2"
            shift 2
            ;;
        --settle-seconds)
            require_value "$1" "$#"
            SETTLE_SECONDS="$2"
            shift 2
            ;;
        --skip-recorder-guard)
            RECORDER_GUARD=0
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            die "unsupported argument: $1"
            ;;
    esac
done

validate_identifier "session name" "$SESSION_NAME"
validate_positive_integer "ready timeout" "$READY_TIMEOUT_SECONDS"
validate_positive_integer "settle seconds" "$SETTLE_SECONDS"
[[ -f "$DESKTOP_FILE" && ! -L "$DESKTOP_FILE" ]] \
    || die "desktop shortcut must be a regular non-symlink file: $DESKTOP_FILE"
command -v gio >/dev/null || die "gio is required to launch the desktop shortcut"
command -v python3 >/dev/null || die "python3 is required"
command -v tmux >/dev/null || die "tmux is required"

eval "$(
    python3 - "$DESKTOP_FILE" <<'PY'
import configparser
from pathlib import Path
import shlex
import sys

path = Path(sys.argv[1])
parser = configparser.ConfigParser(interpolation=None, strict=False)
parser.optionxform = str
with path.open(encoding="utf-8") as stream:
    parser.read_file(stream)
if "Desktop Entry" not in parser:
    raise SystemExit("desktop file is missing [Desktop Entry]")
entry = parser["Desktop Entry"]
exec_value = entry.get("Exec", "")
profile = entry.get("X-Matrix-Profile", "")
active_root = entry.get("X-Matrix-Active-Root", "")
if not exec_value:
    raise SystemExit("desktop file is missing Exec")
if not active_root:
    try:
        parts = shlex.split(exec_value)
    except ValueError:
        parts = []
    for item in parts:
        if item.endswith("/scripts/launch_matrix_bfm_isaac_desktop.sh"):
            active_root = str(Path(item).parents[1])
            break
for name, value in (
    ("DESKTOP_EXEC", exec_value),
    ("DESKTOP_PROFILE", profile),
    ("DESKTOP_ACTIVE_ROOT", active_root),
):
    print(f"{name}={shlex.quote(value)}")
PY
)" || die "failed to parse desktop shortcut: $DESKTOP_FILE"

[[ -n "$DESKTOP_ACTIVE_ROOT" ]] \
    || die "desktop shortcut does not identify an active Matrix root"
[[ -r "$DESKTOP_ACTIVE_ROOT/scripts/launch_matrix_bfm_isaac_desktop.sh" ]] \
    || die "desktop active root is missing the BFM/Isaac launcher: $DESKTOP_ACTIVE_ROOT"
if [[ -n "$PROFILE_EXPECTED" && "$DESKTOP_PROFILE" != "$PROFILE_EXPECTED" ]]; then
    die "desktop profile mismatch: expected=$PROFILE_EXPECTED actual=${DESKTOP_PROFILE:-unset}"
fi

check_recorder_idle() {
    python3 - <<'PY'
from pathlib import Path
import json
import os
import sys

default_paths = [
    "/tmp/g1_wuji_operator_inspire_record_status.json",
    "/tmp/g1_wuji_operator_record_status.json",
    "/tmp/g1_wuji_cloud_record_status.json",
    "/tmp/isaac_sim_record_status.json",
]
raw = os.environ.get("MATRIX_BFM_ISAAC_RECORDER_STATUS_FILES", "")
paths = [Path(p) for p in (raw.split(":") if raw else default_paths) if p]
active = []
for path in paths:
    if not path.exists():
        continue
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        continue
    states = [
        str(value.get(name, "")).upper()
        for name in ("state", "status", "record_state", "recording_state")
    ]
    bool_active = any(
        value.get(name) is True
        for name in ("recording", "recording_active", "is_recording")
    )
    run_active = bool(value.get("active_run_dir")) and "IDLE" not in states
    if bool_active or run_active or any(state in {"REC", "RECORDING"} for state in states):
        active.append(
            f"{path}: state={value.get('state', value.get('status', 'unknown'))} "
            f"episode={value.get('episode', value.get('record_episode', 'unknown'))}"
        )
if active:
    print("\n".join(active), file=sys.stderr)
    raise SystemExit(75)
PY
}

if [[ "$RECORDER_GUARD" == "1" ]]; then
    set +e
    check_recorder_idle
    status=$?
    set -e
    if [[ "$status" != "0" ]]; then
        if [[ "$status" == "75" ]]; then
            printf '[BLOCKED] A known recorder is active; refusing Matrix desktop launch.\n' >&2
            exit 75
        fi
        exit "$status"
    fi
fi

session_exists() {
    tmux has-session -t "=$SESSION_NAME" >/dev/null 2>&1
}

session_is_live() {
    local pane_dead
    session_exists || return 1
    while IFS= read -r pane_dead; do
        [[ "$pane_dead" == "0" ]] && return 0
    done < <(tmux list-panes -t "=$SESSION_NAME" -F '#{pane_dead}' 2>/dev/null)
    return 1
}

session_option() {
    local option="$1"
    tmux show-options -v -t "$SESSION_NAME" "$option" 2>/dev/null || true
}

health_state() {
    local run_dir="$1"
    local console_log="$2"
    python3 - "$run_dir" "$console_log" <<'PY'
from pathlib import Path
import sys

run_dir = Path(sys.argv[1])
console = Path(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2] else None

def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""

keyboard = read(run_dir / "keyboard.log")
physics = read(run_dir / "physics.log")
renderer = read(run_dir / "renderer.log")
console_text = read(console) if console is not None else ""

fatal_patterns = [
    "Traceback",
    "Keyboard bridge exited",
    "Matrix renderer exited",
    "Matrix stack ended",
    "Direct launch is disabled",
    "Desktop sim-only topology rejected",
]
combined = "\n".join((keyboard, physics, renderer, console_text))
for pattern in fatal_patterns:
    if pattern in combined:
        print(f"FAILED fatal_pattern={pattern}")
        raise SystemExit(2)
if (run_dir / "finalizer-status.json").exists():
    print("FAILED finalizer_status_present")
    raise SystemExit(2)
if (run_dir / "runtime-report.json").exists():
    print("FAILED runtime_report_present")
    raise SystemExit(2)

ready_line = ""
for line in keyboard.splitlines():
    if line.startswith("matrix-bfm-isaac-keyboard ready "):
        ready_line = line
if ready_line and "ESCAPE" in ready_line:
    print("FAILED desktop_keyboard_still_forwards_escape")
    raise SystemExit(2)

keyboard_ready = bool(ready_line)
physics_ready = "Interactive controls:" in physics
renderer_ready = (
    "Starting UE" in renderer
    or "Starting canonical Matrix G1 renderer" in console_text
)
if keyboard_ready and physics_ready and renderer_ready:
    print("READY")
    raise SystemExit(0)
missing = []
if not keyboard_ready:
    missing.append("keyboard_ready")
if not physics_ready:
    missing.append("physics_interactive_controls")
if not renderer_ready:
    missing.append("renderer_started")
print("PENDING missing=" + ",".join(missing))
raise SystemExit(1)
PY
}

print_failure_context() {
    local run_dir="$1"
    local console_log="$2"
    printf 'Session: %s\n' "$SESSION_NAME" >&2
    [[ -z "$run_dir" ]] || printf 'Evidence: %s\n' "$run_dir" >&2
    [[ -z "$console_log" ]] || printf 'Console log: %s\n' "$console_log" >&2
    if [[ -n "$console_log" && -r "$console_log" ]]; then
        printf '%s\n' '--- console tail ---' >&2
        tail -80 -- "$console_log" >&2 || true
    fi
    if [[ -n "$run_dir" && -r "$run_dir/keyboard.log" ]]; then
        printf '%s\n' '--- keyboard tail ---' >&2
        tail -40 -- "$run_dir/keyboard.log" >&2 || true
    fi
}

printf 'Launching Matrix through desktop shortcut: %s\n' "$DESKTOP_FILE"
printf 'Desktop active root: %s\n' "$DESKTOP_ACTIVE_ROOT"
printf 'Desktop Exec: %s\n' "$DESKTOP_EXEC"
if ! gio launch "$DESKTOP_FILE"; then
    fail "gio failed to launch the desktop shortcut"
fi

deadline=$((SECONDS + READY_TIMEOUT_SECONDS))
last_state=""
run_dir=""
console_log=""
while ((SECONDS < deadline)); do
    if session_exists; then
        run_dir="$(session_option @matrix_run_dir)"
        console_log="$(session_option @matrix_console_log)"
        if ! session_is_live; then
            print_failure_context "$run_dir" "$console_log"
            fail "Matrix desktop session exited before readiness"
        fi
        if [[ -n "$run_dir" && -d "$run_dir" ]]; then
            set +e
            last_state="$(health_state "$run_dir" "$console_log")"
            health_code=$?
            set -e
            case "$health_code" in
                0)
                    if ((SETTLE_SECONDS > 0)); then
                        sleep "$SETTLE_SECONDS"
                    fi
                    if ! session_is_live; then
                        print_failure_context "$run_dir" "$console_log"
                        fail "Matrix desktop session exited during settle window"
                    fi
                    set +e
                    last_state="$(health_state "$run_dir" "$console_log")"
                    health_code=$?
                    set -e
                    if [[ "$health_code" != "0" ]]; then
                        print_failure_context "$run_dir" "$console_log"
                        fail "Matrix desktop health regressed after settle: $last_state"
                    fi
                    printf 'READY_FOR_PLAY\n'
                    printf 'Matrix desktop shortcut launch is healthy.\n'
                    printf 'Session: %s\n' "$SESSION_NAME"
                    printf 'Attach: tmux attach-session -t =%s\n' "$SESSION_NAME"
                    printf 'Evidence: %s\n' "$run_dir"
                    printf 'Console log: %s\n' "$console_log"
                    exit 0
                    ;;
                1)
                    sleep 1
                    ;;
                *)
                    print_failure_context "$run_dir" "$console_log"
                    fail "Matrix desktop health check failed: $last_state"
                    ;;
            esac
        fi
    fi
    sleep 1
done

print_failure_context "$run_dir" "$console_log"
fail "Matrix desktop shortcut did not become ready within ${READY_TIMEOUT_SECONDS}s; last_state=${last_state:-none}"
