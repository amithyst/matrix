#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
TEMPLATE="$PROJECT_ROOT/packaging/matrix-bfm-isaac-mainline.desktop.in"
PROFILE="trna"
ACTIVE_ROOT="${MATRIX_MAINLINE_ACTIVE_ROOT:-${HOME:?}/matrix-mainline}"
DESKTOP_DIR=""
ICON_PATH=""
TARGET_BASENAME="matrix-sonic.desktop"

usage() {
    cat <<'EOF'
Usage: bash scripts/install_matrix_bfm_isaac_desktop_launcher.sh [options]

Options:
  --profile PROFILE   heyuan, trna (default), or zza
  --active-root DIR   Stable active-release path (default: ~/matrix-mainline)
  --desktop-dir DIR   Existing private Desktop directory
  --icon PATH         Existing icon; defaults below active root
  -h, --help          Show this help
EOF
}

die() {
    printf '[ERROR] %s\n' "$*" >&2
    exit 2
}

require_value() {
    local option="$1"
    local count="$2"
    ((count >= 2)) || die "$option requires a value"
}

reject_control_characters() {
    local label="$1"
    local value="$2"
    if [[ -z "$value" || "$value" == *$'\n'* || "$value" == *$'\r'* \
        || "$value" == *$'\t'* ]]; then
        die "$label contains an empty value or control characters"
    fi
}

validate_profile() {
    case "$1" in
        heyuan | trna | zza) ;;
        *) die "unsupported profile: $1 (expected heyuan, trna, or zza)" ;;
    esac
}

validate_desktop_exec_path() {
    local label="$1"
    local value="$2"
    reject_control_characters "$label" "$value"
    case "$value" in
        *'"'* | *'`'* | *'$'* | *'\'* | *'%'*)
            die "$label contains a character reserved by Desktop Entry Exec: $value"
            ;;
    esac
}

default_desktop_dir() {
    local candidate=""
    if command -v xdg-user-dir >/dev/null 2>&1; then
        candidate="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
    fi
    if [[ -z "$candidate" || "$candidate" == "${HOME:?}" ]]; then
        candidate="$HOME/Desktop"
    fi
    printf '%s\n' "$candidate"
}

canonical_private_directory() {
    local input="$1"
    local logical_path
    local physical_path
    local mode
    local owner
    reject_control_characters "desktop directory" "$input"
    logical_path="$(realpath -sm -- "$input")"
    physical_path="$(realpath -e -- "$input")" \
        || die "desktop directory does not exist: $input"
    [[ -d "$physical_path" ]] \
        || die "desktop path is not a directory: $input"
    [[ "$logical_path" == "$physical_path" ]] \
        || die "desktop directory must not contain symlink components: $input"
    case "$physical_path" in
        / | /bin | /boot | /dev | /etc | /home | /lib | /lib64 | /opt | \
            /proc | /root | /run | /sbin | /srv | /sys | /tmp | /usr | /var | \
            /var/tmp)
            die "refusing dangerous desktop directory: $physical_path"
            ;;
    esac
    owner="$(stat -c '%u' -- "$physical_path")"
    [[ "$owner" == "$UID" ]] \
        || die "desktop directory must be owned by uid $UID: $physical_path"
    mode="$(stat -c '%a' -- "$physical_path")"
    (( (8#$mode & 0022) == 0 )) \
        || die "desktop directory must not be group/world writable: $physical_path"
    [[ -w "$physical_path" && -x "$physical_path" ]] \
        || die "desktop directory is not writable: $physical_path"
    printf '%s\n' "$physical_path"
}

logical_active_root() {
    local input="$1"
    local logical_path
    local physical_path
    reject_control_characters "active root" "$input"
    logical_path="$(realpath -sm -- "$input")"
    physical_path="$(realpath -e -- "$input")" \
        || die "active root does not exist: $input"
    [[ -d "$physical_path" ]] || die "active root is not a directory: $input"
    validate_desktop_exec_path "active root" "$logical_path"
    [[ -r "$logical_path/scripts/launch_matrix_bfm_isaac_desktop.sh" ]] \
        || die "desktop launcher is missing below active root: $logical_path"
    printf '%s\n' "$logical_path"
}

while (($# > 0)); do
    case "$1" in
        --profile)
            require_value "$1" "$#"
            PROFILE="$2"
            shift 2
            ;;
        --active-root)
            require_value "$1" "$#"
            ACTIVE_ROOT="$2"
            shift 2
            ;;
        --desktop-dir)
            require_value "$1" "$#"
            DESKTOP_DIR="$2"
            shift 2
            ;;
        --icon)
            require_value "$1" "$#"
            ICON_PATH="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *) die "unsupported argument: $1" ;;
    esac
done

validate_profile "$PROFILE"
[[ -r "$TEMPLATE" ]] || die "desktop template is missing: $TEMPLATE"
ACTIVE_ROOT="$(logical_active_root "$ACTIVE_ROOT")"
if [[ -z "$ICON_PATH" ]]; then
    ICON_PATH="$ACTIVE_ROOT/demo_gif/Launcher.png"
fi
ICON_PATH="$(realpath -sm -- "$ICON_PATH")"
validate_desktop_exec_path "icon path" "$ICON_PATH"
[[ -f "$ICON_PATH" && -r "$ICON_PATH" ]] \
    || die "icon must be a readable regular file: $ICON_PATH"
if [[ -z "$DESKTOP_DIR" ]]; then
    DESKTOP_DIR="$(default_desktop_dir)"
fi
DESKTOP_DIR="$(canonical_private_directory "$DESKTOP_DIR")"

LAUNCHER="$ACTIVE_ROOT/scripts/launch_matrix_bfm_isaac_desktop.sh"
START_EXEC="/usr/bin/bash \"$LAUNCHER\" start --profile $PROFILE"
STATUS_EXEC="/usr/bin/bash \"$LAUNCHER\" status --profile $PROFILE"
STOP_EXEC="/usr/bin/bash \"$LAUNCHER\" stop --profile $PROFILE"
TARGET="$DESKTOP_DIR/$TARGET_BASENAME"
if [[ -L "$TARGET" ]]; then
    die "refusing symlink desktop target: $TARGET"
fi
if [[ -e "$TARGET" && ! -f "$TARGET" ]]; then
    die "desktop target must be a regular file: $TARGET"
fi
if [[ -e "$TARGET" && "$(stat -c '%u' -- "$TARGET")" != "$UID" ]]; then
    die "existing desktop target is not owned by uid $UID: $TARGET"
fi

umask 077
TEMPORARY="$(mktemp "$DESKTOP_DIR/.$TARGET_BASENAME.tmp.XXXXXX")"
cleanup() {
    [[ -n "${TEMPORARY:-}" && -e "$TEMPORARY" ]] \
        && rm -f -- "$TEMPORARY"
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

while IFS= read -r line || [[ -n "$line" ]]; do
    case "$line" in
        'Exec=@MATRIX_START_EXEC@') printf 'Exec=%s\n' "$START_EXEC" ;;
        'Exec=@MATRIX_STATUS_EXEC@') printf 'Exec=%s\n' "$STATUS_EXEC" ;;
        'Exec=@MATRIX_STOP_EXEC@') printf 'Exec=%s\n' "$STOP_EXEC" ;;
        'Icon=@MATRIX_ICON@') printf 'Icon=%s\n' "$ICON_PATH" ;;
        'X-Matrix-Active-Root=@MATRIX_ACTIVE_ROOT@')
            printf 'X-Matrix-Active-Root=%s\n' "$ACTIVE_ROOT"
            ;;
        'X-Matrix-Profile=@MATRIX_PROFILE@')
            printf 'X-Matrix-Profile=%s\n' "$PROFILE"
            ;;
        *) printf '%s\n' "$line" ;;
    esac
done < "$TEMPLATE" > "$TEMPORARY"

if grep -q '@MATRIX_' "$TEMPORARY"; then
    die "desktop template contains unresolved placeholders"
fi
chmod 0755 -- "$TEMPORARY"
mv -T -- "$TEMPORARY" "$TARGET"
TEMPORARY=""
trap - EXIT HUP INT TERM
if command -v gio >/dev/null 2>&1; then
    gio set "$TARGET" metadata::trusted true >/dev/null 2>&1 || true
fi
printf 'Installed Matrix mainline desktop launcher: %s\n' "$TARGET"
