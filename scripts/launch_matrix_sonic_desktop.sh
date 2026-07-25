#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
RUN_SCRIPT="$SCRIPT_DIR/run_matrix_sonic.sh"
SESSION_NAME="matrix-sonic-desktop-${UID}"
PROFILE="heyuan"
ACTION="start"
SESSION_LOCK_HELD="${MATRIX_DESKTOP_LAUNCHER_LOCKED:-0}"
STOP_GRACE_SECONDS=60
unset MATRIX_DESKTOP_LAUNCHER_LOCKED

usage() {
    cat <<'EOF'
Usage: bash scripts/launch_matrix_sonic_desktop.sh [ACTION] [--profile PROFILE]

Actions:
  start    Start Matrix SONIC unless its tmux session is still live (default)
  status   Report whether the Matrix SONIC tmux session is live or stale
  stop     Stop the Matrix SONIC tmux session
  attach   Attach this terminal to the Matrix SONIC tmux session

Profiles: heyuan (default), trna, zza
EOF
}

die() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 2
}

validate_profile() {
    case "$1" in
        heyuan | trna | zza)
            ;;
        *)
            die "unsupported profile: $1 (expected heyuan, trna, or zza)"
            ;;
    esac
}

session_lock_directory() {
    local directory="${XDG_RUNTIME_DIR:-}"
    local mode
    local owner

    if [[ -z "$directory" || "$directory" != /* || ! -d "$directory" \
        || -L "$directory" ]]; then
        directory="${HOME:?}/.cache/matrix-sonic"
        mkdir -p -m 0700 -- "$directory"
    fi
    [[ -d "$directory" && ! -L "$directory" ]] \
        || die "session lock path is not a real directory: $directory"
    owner="$(stat -c '%u' -- "$directory")"
    [[ "$owner" == "$UID" ]] \
        || die "session lock directory must be owned by uid $UID: $directory"
    mode="$(stat -c '%a' -- "$directory")"
    (( (8#$mode & 0022) == 0 )) \
        || die "session lock directory must not be group/world writable: $directory"
    printf '%s\n' "$directory"
}

run_with_session_lock() {
    local directory
    local flock_bin

    flock_bin="$(command -v flock)" || die "flock is required"
    directory="$(session_lock_directory)"
    umask 077
    MATRIX_DESKTOP_LAUNCHER_LOCKED=1 "$flock_bin" --exclusive --close \
        "$directory/matrix-sonic-desktop-${UID}.lock" \
        /usr/bin/bash "$SCRIPT_DIR/$(basename -- "${BASH_SOURCE[0]}")" \
        "$ACTION" --profile "$PROFILE"
}

notify_user() {
    local message="$1"
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] \
        && command -v notify-send >/dev/null 2>&1; then
        notify-send --app-name="Matrix SONIC" "Matrix SONIC" "$message" \
            >/dev/null 2>&1 || true
    fi
}

print_attach_hint() {
    printf 'Attach with: tmux attach-session -t =%s\n' "$SESSION_NAME"
}

session_exists() {
    tmux has-session -t "=$SESSION_NAME" >/dev/null 2>&1
}

session_is_live() {
    local pane_dead

    session_exists || return 1
    while IFS= read -r pane_dead; do
        if [[ "$pane_dead" == "0" ]]; then
            return 0
        fi
    done < <(tmux list-panes -t "=$SESSION_NAME" -F '#{pane_dead}' 2>/dev/null)
    return 1
}

remove_stale_session() {
    if session_exists && ! session_is_live; then
        tmux kill-session -t "=$SESSION_NAME" >/dev/null 2>&1 || true
    fi
    ! session_exists
}

report_running() {
    printf 'Matrix SONIC is already running in tmux session %s.\n' "$SESSION_NAME"
    print_attach_hint
    notify_user "Already running in $SESSION_NAME. Attach: tmux attach-session -t =$SESSION_NAME"
}

start_session() {
    if [[ ! -f "$RUN_SCRIPT" || ! -r "$RUN_SCRIPT" ]]; then
        die "runtime launcher is missing: $RUN_SCRIPT"
    fi

    if session_is_live; then
        report_running
        return 0
    fi
    if session_exists; then
        remove_stale_session \
            || die "failed to remove stale tmux session: $SESSION_NAME"
        printf 'Removed stale Matrix SONIC tmux session %s before startup.\n' \
            "$SESSION_NAME"
    fi

    if tmux new-session -d -s "$SESSION_NAME" -c "$PROJECT_ROOT" -- \
        /usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
        /usr/bin/bash "$RUN_SCRIPT" \
        --profile "$PROFILE" \
        --scene 2 \
        --control-source game; then
        sleep 0.20
        if ! session_is_live; then
            remove_stale_session || true
            notify_user "Matrix SONIC exited during startup. Run the launcher from a terminal for details."
            printf '[ERROR] Matrix SONIC exited during startup\n' >&2
            return 1
        fi
        printf 'Started Matrix SONIC in tmux session %s (profile %s).\n' \
            "$SESSION_NAME" "$PROFILE"
        print_attach_hint
        notify_user "Started profile $PROFILE in $SESSION_NAME. Attach: tmux attach-session -t =$SESSION_NAME"
        return 0
    fi

    # A simultaneous desktop click can win the create race after our first check.
    if session_is_live; then
        report_running
        return 0
    fi
    remove_stale_session || true

    notify_user "Failed to start Matrix SONIC. Run the launcher from a terminal for details."
    printf '[ERROR] failed to create tmux session %s\n' "$SESSION_NAME" >&2
    return 1
}

status_session() {
    if session_is_live; then
        printf 'Matrix SONIC is running in tmux session %s.\n' "$SESSION_NAME"
        print_attach_hint
        notify_user "Running in $SESSION_NAME. Attach: tmux attach-session -t =$SESSION_NAME"
        return 0
    fi

    if session_exists; then
        printf 'Matrix SONIC is stopped; tmux session %s is stale (all panes exited).\n' \
            "$SESSION_NAME"
        notify_user "Stopped: tmux session $SESSION_NAME is stale. Start again to clean it."
        return 1
    fi

    printf 'Matrix SONIC is stopped (tmux session %s does not exist).\n' \
        "$SESSION_NAME"
    notify_user "Stopped: tmux session $SESSION_NAME does not exist."
    return 1
}

stop_session() {
    local deadline
    local pane_pid=""
    local pane_uid=""
    if ! session_exists; then
        printf 'Matrix SONIC is already stopped (tmux session %s does not exist).\n' \
            "$SESSION_NAME"
        notify_user "Already stopped: $SESSION_NAME does not exist."
        return 0
    fi

    if ! session_is_live; then
        remove_stale_session \
            || die "failed to remove stale tmux session: $SESSION_NAME"
        printf 'Removed stopped Matrix SONIC tmux session %s.\n' "$SESSION_NAME"
        notify_user "Removed stopped session $SESSION_NAME."
        return 0
    fi

    tmux send-keys -t "=$SESSION_NAME:0.0" C-c
    sleep 0.25
    if session_is_live; then
        pane_pid="$(
            while read -r pane_dead candidate_pid; do
                if [[ "$pane_dead" == "0" ]]; then
                    printf '%s\n' "$candidate_pid"
                    break
                fi
            done < <(
                tmux list-panes -t "=$SESSION_NAME" \
                    -F '#{pane_dead} #{pane_pid}' 2>/dev/null
            )
        )"
        if [[ "$pane_pid" =~ ^[1-9][0-9]*$ ]]; then
            pane_uid="$(ps -o uid= -p "$pane_pid" 2>/dev/null | tr -d ' ')"
            if [[ "$pane_uid" == "$UID" ]]; then
                kill -INT "$pane_pid" 2>/dev/null || true
            fi
        fi
    fi
    # run_matrix_sonic.sh allows its run_sim child 25 seconds for normal TERM
    # cleanup before restoring tracked config, so the desktop wrapper must wait
    # longer than that inner contract before declaring the whole stack stuck.
    deadline=$((SECONDS + STOP_GRACE_SECONDS))
    while ((SECONDS < deadline)); do
        if ! session_exists; then
            printf 'Stopped Matrix SONIC tmux session %s cleanly.\n' \
                "$SESSION_NAME"
            notify_user "Stopped $SESSION_NAME cleanly."
            return 0
        fi
        if ! session_is_live; then
            remove_stale_session \
                || die "failed to remove stopped tmux session: $SESSION_NAME"
            printf 'Stopped Matrix SONIC and removed tmux session %s cleanly.\n' \
                "$SESSION_NAME"
            notify_user "Stopped $SESSION_NAME cleanly."
            return 0
        fi
        sleep 0.25
    done

    tmux kill-session -t "=$SESSION_NAME" >/dev/null 2>&1 || true
    notify_user "Matrix SONIC cleanup timed out; tmux session was forced closed."
    printf '[ERROR] Matrix SONIC cleanup timed out; forced tmux session %s closed\n' \
        "$SESSION_NAME" >&2
    return 1
}

attach_session() {
    if ! session_is_live; then
        printf '[ERROR] Matrix SONIC is stopped; tmux session %s is missing or stale.\n' \
            "$SESSION_NAME" >&2
        return 1
    fi
    exec tmux attach-session -t "=$SESSION_NAME"
}

if (($# > 0)) && [[ "$1" != --* ]]; then
    ACTION="$1"
    shift
fi

while (($# > 0)); do
    case "$1" in
        --profile)
            (($# >= 2)) || die "--profile requires a value"
            PROFILE="$2"
            shift 2
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

validate_profile "$PROFILE"
command -v tmux >/dev/null 2>&1 || die "tmux is required"

if [[ "$SESSION_LOCK_HELD" != "1" \
    && ( "$ACTION" == "start" || "$ACTION" == "stop" ) ]]; then
    run_with_session_lock
    exit $?
fi

case "$ACTION" in
    start)
        start_session
        ;;
    status)
        status_session
        ;;
    stop)
        stop_session
        ;;
    attach)
        attach_session
        ;;
    *)
        die "unsupported action: $ACTION (expected start, status, stop, or attach)"
        ;;
esac
