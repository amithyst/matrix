#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
SESSION_NAME="matrix-sonic-desktop-${UID}"
HOST_LOCK_PATH="${MATRIX_DESKTOP_HOST_LOCK_PATH:-/tmp/matrix-sonic-${UID}.lock}"
PROFILE="heyuan"
SCENE_ID="15"
INITIAL_LOCOMOTION_POLICY="${MATRIX_INITIAL_LOCOMOTION_POLICY:-}"
ACTION="start"
SESSION_LOCK_HELD="${MATRIX_DESKTOP_LAUNCHER_LOCKED:-0}"
STOP_GRACE_SECONDS=60
unset MATRIX_DESKTOP_LAUNCHER_LOCKED

usage() {
    cat <<'EOF'
Usage: bash scripts/launch_matrix_sonic_desktop.sh [ACTION] [options]

Actions:
  start    Start Matrix SONIC unless its tmux session is still live (default)
  status   Report whether the Matrix SONIC tmux session is live or stale
  stop     Stop the Matrix SONIC tmux session
  attach   Attach this terminal to the Matrix SONIC tmux session

Profiles: heyuan (default), trna, zza

Options:
  --profile PROFILE  Host profile (default: heyuan)
  --scene ID         Matrix native scene id (default: 15 / MoonWorld)
  --initial-locomotion-policy POLICY
                     sonic or bfm-sonic-teacher50k
                     (default: selected host profile; tRNA uses BFM Teacher50k)

Scene 15 starts MoonWorld with the selected host profile's locomotion policy.
Other scenes use the generic Matrix SONIC game-control launcher.
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

validate_scene() {
    [[ "$1" =~ ^(0|[1-9][0-9]?)$ ]] \
        || die "invalid scene id: $1 (expected 0-99)"
}

validate_initial_locomotion_policy() {
    case "$1" in
        sonic | bfm-sonic-teacher50k)
            ;;
        *)
            die "invalid initial locomotion policy: $1 (expected sonic or bfm-sonic-teacher50k)"
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
    local -a launcher_args

    flock_bin="$(command -v flock)" || die "flock is required"
    directory="$(session_lock_directory)"
    launcher_args=(--profile "$PROFILE" --scene "$SCENE_ID")
    if [[ -n "$INITIAL_LOCOMOTION_POLICY" ]]; then
        launcher_args+=(
            --initial-locomotion-policy "$INITIAL_LOCOMOTION_POLICY"
        )
    fi
    umask 077
    MATRIX_DESKTOP_LAUNCHER_LOCKED=1 "$flock_bin" --exclusive --close \
        "$directory/matrix-sonic-desktop-${UID}.lock" \
        /usr/bin/bash "$SCRIPT_DIR/$(basename -- "${BASH_SOURCE[0]}")" \
        "$ACTION" "${launcher_args[@]}"
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

host_runtime_is_locked() {
    local fd
    local flock_bin
    local parent

    [[ "$HOST_LOCK_PATH" == /* ]] \
        || die "host runtime lock path must be absolute: $HOST_LOCK_PATH"
    parent="$(dirname -- "$HOST_LOCK_PATH")"
    [[ -d "$parent" && ! -L "$parent" ]] \
        || die "host runtime lock directory is invalid: $parent"
    flock_bin="$(command -v flock)" || die "flock is required"
    umask 077
    exec {fd}>"$HOST_LOCK_PATH" \
        || die "cannot open host runtime lock: $HOST_LOCK_PATH"
    if "$flock_bin" --exclusive --nonblock "$fd"; then
        "$flock_bin" --unlock "$fd" >/dev/null 2>&1 || true
        exec {fd}>&-
        return 1
    fi
    exec {fd}>&-
    return 0
}

describe_host_runtime_owner() {
    local owner_cwd
    local owner_name
    local owner_pid

    if command -v lsof >/dev/null 2>&1; then
        while IFS= read -r owner_pid; do
            [[ "$owner_pid" =~ ^[1-9][0-9]*$ ]] || continue
            owner_cwd="$(readlink -f -- "/proc/$owner_pid/cwd" 2>/dev/null || true)"
            if [[ "$owner_cwd" == /* ]]; then
                owner_name="$(basename -- "$owner_cwd")"
                printf 'pid=%s, source=%s\n' "$owner_pid" "$owner_name"
                return 0
            fi
        done < <(lsof -t -- "$HOST_LOCK_PATH" 2>/dev/null | sort -n -u)
    fi
    printf 'lock=%s\n' "$HOST_LOCK_PATH"
}

start_session() {
    local owner
    local message
    local run_script
    local runtime_args
    local -a env_overrides

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

    if host_runtime_is_locked; then
        owner="$(describe_host_runtime_owner)"
        message="Another Matrix SONIC instance owns this host ($owner). Wait for it to finish or stop it before starting profile $PROFILE."
        notify_user "$message"
        printf '[ERROR] %s\n' "$message" >&2
        return 1
    fi

    if [[ "$SCENE_ID" == "15" ]]; then
        run_script="$SCRIPT_DIR/run_matrix_sonic_moon_v1.sh"
        runtime_args=(
            --profile "$PROFILE"
            --control-source game
            --game-fall-recovery auto
        )
    else
        run_script="$SCRIPT_DIR/run_matrix_sonic.sh"
        runtime_args=(
            --profile "$PROFILE"
            --scene "$SCENE_ID"
            --control-source game
        )
    fi
    if [[ -n "$INITIAL_LOCOMOTION_POLICY" ]]; then
        runtime_args+=(
            --initial-locomotion-policy "$INITIAL_LOCOMOTION_POLICY"
        )
    fi
    if [[ "$SCENE_ID" == "15" \
        && "$INITIAL_LOCOMOTION_POLICY" == "bfm-sonic-teacher50k" ]]; then
        runtime_args+=(
            --game-world-persistence on
            --game-auto-respawn on
        )
    fi
    if [[ ! -f "$run_script" || ! -r "$run_script" ]]; then
        die "runtime launcher is missing: $run_script"
    fi

    env_overrides=(
        MATRIX_GAME_GRAB_ESCAPE=1
        MATRIX_ESC_OVERLAY_MODAL_SHIELD=0
        MATRIX_ESC_OVERLAY_RECENTER_POINTER=0
        MATRIX_ESC_OVERLAY_KEEP_RAISED=0
        MATRIX_ESC_OVERLAY_CLOSE_ON_FOCUS_LOSS=1
    )
    if [[ -n "${MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER:-}" ]]; then
        case "$MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER" in
            raw | flat-local | flat_anchor | flat-anchor | anchor)
                ;;
            *)
                die "invalid MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER: $MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER"
                ;;
        esac
        env_overrides+=(
            "MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER=$MATRIX_MOON_DYNAMIC_GROUND_HEIGHT_FILTER"
        )
    fi
    if [[ -n "${MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE:-}" ]]; then
        case "$MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE" in
            stable | default | hfield | heightfield | continuous | continuous-hfield | rolling-hfield | rolling-heightfield | rolling-heightfield-v2 | tiles | tile | mocap-tiles | rolling-tiles | rolling-mocap-tiles | rolling-mocap-tiles-v1 | leo | official)
                ;;
            *)
                die "invalid MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE: $MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE"
                ;;
        esac
        env_overrides+=(
            "MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE=$MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE"
        )
    fi

    if tmux new-session -d -s "$SESSION_NAME" -c "$PROJECT_ROOT" -- \
        /usr/bin/env -u LD_LIBRARY_PATH -u PYTHONPATH \
        -u MATRIX_MOON_DYNAMIC_GROUND_COLLISION_MODE \
        "${env_overrides[@]}" \
        /usr/bin/bash "$run_script" \
        "${runtime_args[@]}"; then
        sleep 0.20
        if ! session_is_live; then
            remove_stale_session || true
            notify_user "Matrix SONIC exited during startup. Run the launcher from a terminal for details."
            printf '[ERROR] Matrix SONIC exited during startup\n' >&2
            return 1
        fi
        printf 'Started Matrix SONIC in tmux session %s (profile %s, scene %s).\n' \
            "$SESSION_NAME" "$PROFILE" "$SCENE_ID"
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
        --scene)
            (($# >= 2)) || die "--scene requires a value"
            SCENE_ID="$2"
            shift 2
            ;;
        --initial-locomotion-policy)
            (($# >= 2)) || die "--initial-locomotion-policy requires a value"
            INITIAL_LOCOMOTION_POLICY="$2"
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
validate_scene "$SCENE_ID"
if [[ -n "$INITIAL_LOCOMOTION_POLICY" ]]; then
    validate_initial_locomotion_policy "$INITIAL_LOCOMOTION_POLICY"
elif [[ "$PROFILE" == "trna" && "$SCENE_ID" == "15" ]]; then
    INITIAL_LOCOMOTION_POLICY="bfm-sonic-teacher50k"
fi
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
