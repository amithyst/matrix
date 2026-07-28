#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
SESSION_NAME="${MATRIX_BFM_ISAAC_DESKTOP_SESSION_NAME:-matrix-bfm-isaac-mainline-${UID}}"
INSTANCE_ID="${MATRIX_BFM_ISAAC_DESKTOP_INSTANCE_ID:-matrix-bfm-isaac-mainline-${UID}}"
HOST_LOCK_PATH="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"
PROFILE="trna"
DURATION="${MATRIX_BFM_ISAAC_DESKTOP_DURATION:-7200}"
STOP_GRACE_SECONDS="${MATRIX_BFM_ISAAC_DESKTOP_STOP_GRACE_S:-120}"
ACTION="start"
STATE_DIR="${MATRIX_BFM_ISAAC_DESKTOP_STATE_DIR:-${XDG_STATE_HOME:-${HOME:?}/.local/state}/matrix-bfm-isaac}"
LOG_DIR="${MATRIX_BFM_ISAAC_DESKTOP_LOG_DIR:-$STATE_DIR}"
LOG_FILE="$LOG_DIR/mainline-desktop-launcher.log"
SESSION_LOCK_HELD="${MATRIX_BFM_ISAAC_DESKTOP_LOCKED:-0}"
unset MATRIX_BFM_ISAAC_DESKTOP_LOCKED

usage() {
    cat <<'EOF'
Usage: bash scripts/launch_matrix_bfm_isaac_desktop.sh [ACTION] [options]

Actions:
  start    Start the qualified Matrix BFM/Isaac mainline in tmux (default)
  status   Report whether the desktop tmux session is live or stale
  stop     Request a graceful stop and wait for finalization
  attach   Attach this terminal to the desktop tmux session
  dismiss  Remove a dead failed session after its persistent logs were reviewed

Options:
  --profile PROFILE  Matrix host profile (default: trna)
  --duration SEC     Positive simulated duration (default: 7200)
  -h, --help         Show this help
EOF
}

die() {
    log_event "ERROR $*"
    printf '[ERROR] %s\n' "$*" >&2
    exit 2
}

log_event() {
    local message="$1"
    mkdir -p -m 0700 -- "$LOG_DIR" 2>/dev/null || return 0
    printf '%s pid=%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$$" \
        "$message" >> "$LOG_FILE" 2>/dev/null || true
}

prepare_state_directory() {
    local mode
    local owner
    mkdir -p -m 0700 -- "$STATE_DIR"
    [[ -d "$STATE_DIR" && ! -L "$STATE_DIR" ]] \
        || die "state directory must be a real directory: $STATE_DIR"
    owner="$(stat -c '%u' -- "$STATE_DIR")"
    [[ "$owner" == "$UID" ]] \
        || die "state directory must be owned by uid $UID: $STATE_DIR"
    mode="$(stat -c '%a' -- "$STATE_DIR")"
    (( (8#$mode & 0077) == 0 )) \
        || die "state directory must not be accessible by group/world: $STATE_DIR"
}

validate_identifier() {
    local label="$1"
    local value="$2"
    [[ "$value" =~ ^[A-Za-z0-9._-]+$ ]] \
        || die "$label must match [A-Za-z0-9._-]+"
}

validate_profile() {
    case "$1" in
        heyuan | trna | zza) ;;
        *) die "unsupported profile: $1 (expected heyuan, trna, or zza)" ;;
    esac
}

validate_positive_integer() {
    local label="$1"
    local value="$2"
    [[ "$value" =~ ^[1-9][0-9]*$ ]] \
        || die "$label must be a positive integer"
}

require_value() {
    local option="$1"
    local count="$2"
    ((count >= 2)) || die "$option requires a value"
}

notify_user() {
    local message="$1"
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] \
        && command -v notify-send >/dev/null 2>&1; then
        notify-send --app-name="Matrix 主线" "Matrix 主线" "$message" \
            >/dev/null 2>&1 || true
    fi
}

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

remove_stale_session() {
    if session_exists && ! session_is_live; then
        tmux kill-session -t "=$SESSION_NAME" >/dev/null 2>&1 || true
    fi
    ! session_exists
}

session_option() {
    local option="$1"
    tmux show-options -v -t "$SESSION_NAME" "$option" 2>/dev/null || true
}

print_evidence_hint() {
    local run_dir="${1:-}"
    local console_log="${2:-}"
    [[ -z "$run_dir" ]] || printf 'Evidence: %s\n' "$run_dir"
    [[ -z "$console_log" ]] || printf 'Console log: %s\n' "$console_log"
}

finalizer_is_clean() {
    local run_dir="$1"
    [[ -n "$run_dir" ]] || return 1
    python3 - "$run_dir/finalizer-status.json" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
try:
    value = json.loads(path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    raise SystemExit(1)
required = {
    "complete": True,
    "physics_exit_code": 0,
    "report_present": True,
    "trajectory_present": True,
    "relay_status_present": True,
}
base_ok = all(value.get(k) == v for k, v in required.items())
termination_ok = (value.get("trigger"), value.get("stack_failure_code")) in {
    ("natural", 0),
    ("signal_term", 143),
}
raise SystemExit(0 if base_ok and termination_ok else 1)
PY
}

keyboard_socket_for_run() {
    local run_dir="$1"
    python3 - "$run_dir/keyboard.log" "$PROJECT_ROOT" <<'PY'
from pathlib import Path
import sys

log = Path(sys.argv[1])
project = Path(sys.argv[2]).resolve()
allowed = (project / "outputs/runtime/matrix-bfm-isaac/ipc").resolve()
try:
    content = log.read_text(encoding="utf-8")
except OSError:
    raise SystemExit(1)
matches = []
for line in content.splitlines():
    if " socket=" not in line or " keys=" not in line:
        continue
    matches.append(line.split(" socket=", 1)[1].rsplit(" keys=", 1)[0])
if not matches:
    raise SystemExit(1)
candidate = Path(matches[-1])
try:
    resolved = candidate.resolve(strict=True)
    resolved.relative_to(allowed)
except (OSError, ValueError):
    raise SystemExit(1)
if not resolved.is_socket():
    raise SystemExit(1)
print(resolved)
PY
}

request_runtime_finalizer() {
    local run_dir="$1"
    local keyboard_socket
    if ! keyboard_socket="$(keyboard_socket_for_run "$run_dir")"; then
        return 1
    fi
    python3 "$SCRIPT_DIR/matrix_bfm_isaac_command.py" \
        --socket "$keyboard_socket" --key SPACE --key ESCAPE
}

print_attach_hint() {
    printf 'Attach with: tmux attach-session -t =%s\n' "$SESSION_NAME"
}

host_runtime_is_locked() {
    local fd
    local parent
    [[ "$HOST_LOCK_PATH" == /* ]] \
        || die "host runtime lock path must be absolute: $HOST_LOCK_PATH"
    parent="$(dirname -- "$HOST_LOCK_PATH")"
    [[ -d "$parent" && ! -L "$parent" ]] \
        || die "host runtime lock directory is invalid: $parent"
    exec {fd}>"$HOST_LOCK_PATH" \
        || die "cannot open host runtime lock: $HOST_LOCK_PATH"
    if flock --exclusive --nonblock "$fd"; then
        flock --unlock "$fd" >/dev/null 2>&1 || true
        exec {fd}>&-
        return 1
    fi
    exec {fd}>&-
    return 0
}

start_session() {
    local display_value="${DISPLAY:-:0}"
    local xauthority_value="${XAUTHORITY:-/run/user/${UID}/gdm/Xauthority}"
    local run_script="$SCRIPT_DIR/run_matrix_bfm_isaac_guarded.sh"
    local run_stamp
    local run_dir
    local console_log
    local -a command

    if session_is_live; then
        printf 'Matrix mainline is already running in tmux session %s.\n' \
            "$SESSION_NAME"
        print_attach_hint
        notify_user "已经在 $SESSION_NAME 中运行"
        log_event "START idempotent session=$SESSION_NAME"
        return 0
    fi
    if session_exists; then
        local stale_run_dir
        local stale_console_log
        stale_run_dir="$(session_option @matrix_run_dir)"
        stale_console_log="$(session_option @matrix_console_log)"
        if ! finalizer_is_clean "$stale_run_dir"; then
            printf '[ERROR] Previous Matrix session failed finalizer verification and was retained: %s\n' \
                "$SESSION_NAME" >&2
            print_evidence_hint "$stale_run_dir" "$stale_console_log" >&2
            notify_user "上次启动失败，已保留现场；请先检查日志"
            log_event "START blocked failed_stale session=$SESSION_NAME run_dir=$stale_run_dir console=$stale_console_log"
            return 1
        fi
        remove_stale_session \
            || die "failed to remove finalized stale tmux session: $SESSION_NAME"
        printf 'Removed finalized Matrix mainline tmux session %s.\n' \
            "$SESSION_NAME"
        log_event "START removed_finalized_stale session=$SESSION_NAME run_dir=$stale_run_dir console=$stale_console_log"
    fi
    if host_runtime_is_locked; then
        printf '[ERROR] Another Matrix launcher owns this host: %s\n' \
            "$HOST_LOCK_PATH" >&2
        notify_user "已有 Matrix 实例占用主机，请先停止后再启动"
        log_event "START blocked host_lock=$HOST_LOCK_PATH"
        return 1
    fi
    [[ -f "$run_script" && -r "$run_script" ]] \
        || die "qualified runtime launcher is missing: $run_script"

    run_stamp="$(date -u +%Y%m%dT%H%M%SZ)"
    run_dir="$PROJECT_ROOT/outputs/runs/matrix-bfm-isaac/desktop_${run_stamp}_$$"
    console_log="$STATE_DIR/desktop_${run_stamp}_$$.console.log"

    command=(
        /usr/bin/env
        -u LD_LIBRARY_PATH
        -u PYTHONPATH
        -u MATRIX_UE_EXTRA_EXEC_CMDS
        -u MATRIX_UE_MATERIAL_FIX_PRELOAD
        -u MATRIX_BFM_ISAAC_VIDEO_RESOLUTION
        -u MATRIX_BFM_ISAAC_VIDEO_WINDOW_MODE
        -u MATRIX_BFM_ISAAC_UE_MAX_FPS
        -u MATRIX_BFM_ISAAC_VIDEO_QUALITY
        -u MATRIX_BFM_ISAAC_VIDEO_CAMERA_SMOOTHING
        -u MATRIX_BFM_ISAAC_SCREEN_PERCENTAGE
        -u MATRIX_PICO_PYTHON
        -u MATRIX_PICO_WHEEL
        -u FASTRTPS_DEFAULT_PROFILES_FILE
        -u CYCLONEDDS_URI
        -u RMW_IMPLEMENTATION
        -u ROS_DOMAIN_ID
        -u ROS_LOCALHOST_ONLY
        -u XR_RUNTIME_JSON
        -u XR_API_LAYER_PATH
        "DISPLAY=$display_value"
        "XAUTHORITY=$xauthority_value"
        "MATRIX_INSTANCE_ID=$INSTANCE_ID"
        "MATRIX_BFM_ISAAC_DESKTOP_SIM_ONLY=1"
        "MATRIX_PICO_INPUT_ENABLED=0"
        "MATRIX_EXTERNAL_STATE=1"
        "MATRIX_DISABLE_MC=1"
        "MATRIX_SONIC=0"
        "MATRIX_SONIC_CONTROL_SOURCE=external"
        "MATRIX_GAME_INPUT_SOURCE=keyboard"
        "MATRIX_GAME_NO_INPUT_PROVIDER=1"
        "MATRIX_BFM_ISAAC_KEYBOARD_ESCAPE_EXIT=0"
        /usr/bin/bash "$run_script" interactive
        --profile "$PROFILE"
        --onscreen
        --duration "$DURATION"
        --correctness-only
        --run-dir "$run_dir"
    )

    if ! tmux new-session -d -s "$SESSION_NAME" -c "$PROJECT_ROOT" -- \
        /usr/bin/bash -o pipefail -c \
        'log="$1"; shift; "$@" 2>&1 | /usr/bin/tee --ignore-interrupts -a -- "$log"' \
        matrix-mainline "$console_log" "${command[@]}"; then
        if session_is_live; then
            printf 'Matrix mainline is already running in tmux session %s.\n' \
                "$SESSION_NAME"
            return 0
        fi
        printf '[ERROR] failed to create tmux session %s\n' \
            "$SESSION_NAME" >&2
        notify_user "启动 tmux 会话失败"
        log_event "START failed tmux_create session=$SESSION_NAME"
        return 1
    fi
    tmux set-window-option -t "=$SESSION_NAME:" remain-on-exit on >/dev/null \
        || die "could not retain desktop tmux session"
    tmux set-option -t "$SESSION_NAME" @matrix_run_dir "$run_dir" >/dev/null \
        || die "could not record Matrix evidence directory"
    tmux set-option -t "$SESSION_NAME" @matrix_console_log \
        "$console_log" >/dev/null \
        || die "could not record Matrix console log"
    sleep 0.30
    if ! session_is_live; then
        printf '[ERROR] Matrix mainline exited during startup; inspect tmux session %s\n' \
            "$SESSION_NAME" >&2
        notify_user "启动失败，请检查 tmux 会话 $SESSION_NAME"
        print_evidence_hint "$run_dir" "$console_log" >&2
        log_event "START failed early_exit session=$SESSION_NAME run_dir=$run_dir console=$console_log"
        return 1
    fi

    printf 'Started qualified Matrix BFM/Isaac mainline in tmux session %s.\n' \
        "$SESSION_NAME"
    printf 'Repository: %s\nProfile: %s\n' "$PROJECT_ROOT" "$PROFILE"
    print_attach_hint
    print_evidence_hint "$run_dir" "$console_log"
    notify_user "已启动当前验收主线；会话 $SESSION_NAME"
    log_event "START ok session=$SESSION_NAME profile=$PROFILE duration=$DURATION root=$PROJECT_ROOT run_dir=$run_dir console=$console_log"
}

status_session() {
    if session_is_live; then
        printf 'Matrix mainline is running in tmux session %s.\n' "$SESSION_NAME"
        print_attach_hint
        notify_user "正在运行：$SESSION_NAME"
        log_event "STATUS running session=$SESSION_NAME"
        return 0
    fi
    if session_exists; then
        local run_dir
        local console_log
        run_dir="$(session_option @matrix_run_dir)"
        console_log="$(session_option @matrix_console_log)"
        printf 'Matrix mainline is stopped; tmux session %s is stale.\n' \
            "$SESSION_NAME"
        print_evidence_hint "$run_dir" "$console_log"
        notify_user "已停止；会话 $SESSION_NAME 保留了退出信息"
        log_event "STATUS stale session=$SESSION_NAME run_dir=$run_dir console=$console_log"
        return 1
    fi
    printf 'Matrix mainline is stopped (tmux session %s does not exist).\n' \
        "$SESSION_NAME"
    notify_user "Matrix 主线未运行"
    log_event "STATUS stopped session=$SESSION_NAME"
    return 1
}

stop_session() {
    local deadline
    local pane_status=""
    local run_dir=""
    local console_log=""
    if ! session_exists; then
        printf 'Matrix mainline is already stopped.\n'
        notify_user "已经停止"
        log_event "STOP already_stopped session=$SESSION_NAME"
        return 0
    fi
    if ! session_is_live; then
        run_dir="$(session_option @matrix_run_dir)"
        console_log="$(session_option @matrix_console_log)"
        if finalizer_is_clean "$run_dir"; then
            remove_stale_session \
                || die "failed to remove stale tmux session: $SESSION_NAME"
            printf 'Removed finalized Matrix mainline tmux session %s.\n' \
                "$SESSION_NAME"
            notify_user "已核验 finalizer 并清理停止的会话"
            log_event "STOP removed_finalized_stale session=$SESSION_NAME run_dir=$run_dir console=$console_log"
            return 0
        fi
        printf '[ERROR] Matrix pane is dead but finalizer is missing or failed; session retained: %s\n' \
            "$SESSION_NAME" >&2
        print_evidence_hint "$run_dir" "$console_log" >&2
        notify_user "停止结果未通过 finalizer 核验，已保留现场"
        log_event "STOP finalizer_failed_stale session=$SESSION_NAME run_dir=$run_dir console=$console_log"
        return 1
    fi

    run_dir="$(session_option @matrix_run_dir)"
    console_log="$(session_option @matrix_console_log)"
    if ! request_runtime_finalizer "$run_dir"; then
        printf '[ERROR] Could not request the Matrix runtime finalizer; live session retained: %s\n' \
            "$SESSION_NAME" >&2
        print_evidence_hint "$run_dir" "$console_log" >&2
        notify_user "无法请求自然 finalizer，运行会话保持不变"
        log_event "STOP request_failed session=$SESSION_NAME run_dir=$run_dir console=$console_log"
        return 1
    fi
    log_event "STOP finalizer_requested session=$SESSION_NAME run_dir=$run_dir"
    deadline=$((SECONDS + STOP_GRACE_SECONDS))
    while ((SECONDS < deadline)); do
        if ! session_is_live; then
            pane_status="$(
                tmux list-panes -t "=$SESSION_NAME" \
                    -F '#{pane_dead_status}' 2>/dev/null | head -n 1
            )"
            if finalizer_is_clean "$run_dir"; then
                remove_stale_session \
                    || die "failed to remove stopped tmux session: $SESSION_NAME"
                printf 'Stopped Matrix mainline safely; finalizer verified (pane status %s).\n' \
                    "${pane_status:-unknown}"
                notify_user "Matrix 主线已安全停止，finalizer 验证通过"
                log_event "STOP finalizer_ok session=$SESSION_NAME pane_status=${pane_status:-unknown} run_dir=$run_dir console=$console_log"
                return 0
            fi
            printf '[ERROR] Matrix process exited but finalizer did not pass; session retained: %s\n' \
                "$SESSION_NAME" >&2
            print_evidence_hint "$run_dir" "$console_log" >&2
            notify_user "进程已退出但 finalizer 未通过，已保留现场"
            log_event "STOP finalizer_failed session=$SESSION_NAME pane_status=${pane_status:-unknown} run_dir=$run_dir console=$console_log"
            return 1
        fi
        sleep 0.25
    done

    printf '[ERROR] Matrix mainline cleanup exceeded %s seconds; session was left for inspection: %s\n' \
        "$STOP_GRACE_SECONDS" "$SESSION_NAME" >&2
    notify_user "停止超时，已保留会话供检查，没有强杀进程"
    log_event "STOP timeout session=$SESSION_NAME grace=$STOP_GRACE_SECONDS"
    return 1
}

attach_session() {
    session_exists || {
        printf '[ERROR] Matrix mainline tmux session does not exist: %s\n' \
            "$SESSION_NAME" >&2
        return 1
    }
    exec tmux attach-session -t "=$SESSION_NAME"
}

dismiss_stale_session() {
    local run_dir=""
    local console_log=""
    if ! session_exists; then
        printf 'Matrix mainline tmux session does not exist; nothing to dismiss.\n'
        return 0
    fi
    if session_is_live; then
        printf '[ERROR] refusing to dismiss a live Matrix session: %s\n' \
            "$SESSION_NAME" >&2
        return 1
    fi
    run_dir="$(session_option @matrix_run_dir)"
    console_log="$(session_option @matrix_console_log)"
    tmux kill-session -t "=$SESSION_NAME"
    printf 'Dismissed dead Matrix session %s; persistent evidence was kept.\n' \
        "$SESSION_NAME"
    print_evidence_hint "$run_dir" "$console_log"
    log_event "DISMISS dead_session session=$SESSION_NAME run_dir=$run_dir console=$console_log"
}

if (($# > 0)); then
    case "$1" in
        start | status | stop | attach | dismiss)
            ACTION="$1"
            shift
            ;;
        -h | --help)
            usage
            exit 0
            ;;
    esac
fi

while (($# > 0)); do
    case "$1" in
        --profile)
            require_value "$1" "$#"
            PROFILE="$2"
            shift 2
            ;;
        --duration)
            require_value "$1" "$#"
            DURATION="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *) die "unsupported argument: $1" ;;
    esac
done

validate_identifier "session name" "$SESSION_NAME"
validate_identifier "instance id" "$INSTANCE_ID"
validate_profile "$PROFILE"
validate_positive_integer "duration" "$DURATION"
validate_positive_integer "stop grace" "$STOP_GRACE_SECONDS"
for command_name in flock python3 tmux; do
    command -v "$command_name" >/dev/null \
        || die "$command_name is required"
done

prepare_state_directory
if [[ "$ACTION" != "attach" && "$SESSION_LOCK_HELD" != "1" ]]; then
    export MATRIX_BFM_ISAAC_DESKTOP_LOCKED=1
    exec flock --exclusive --close "$STATE_DIR/desktop-session.lock" \
        /usr/bin/bash "$SCRIPT_DIR/$(basename -- "${BASH_SOURCE[0]}")" \
        "$ACTION" --profile "$PROFILE" --duration "$DURATION"
fi

log_event "INVOKE action=$ACTION profile=$PROFILE duration=$DURATION session=$SESSION_NAME"

case "$ACTION" in
    start) start_session ;;
    status) status_session ;;
    stop) stop_session ;;
    attach) attach_session ;;
    dismiss) dismiss_stale_session ;;
    *) die "unsupported action: $ACTION" ;;
esac
