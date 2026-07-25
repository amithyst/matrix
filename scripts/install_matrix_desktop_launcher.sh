#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
TEMPLATE="$PROJECT_ROOT/packaging/matrix-sonic.desktop.in"
LAUNCHER="$SCRIPT_DIR/launch_matrix_sonic_desktop.sh"
PROFILE="heyuan"
DESKTOP_DIR=""
ICON_PATH="$PROJECT_ROOT/demo_gif/Launcher.png"

usage() {
    cat <<'EOF'
Usage: bash scripts/install_matrix_desktop_launcher.sh [options]

Options:
  --profile PROFILE     heyuan (default), trna, or zza
  --desktop-dir DIR     Existing private Desktop directory (for tests or XDG overrides)
  --icon PATH           Existing icon file (default: demo_gif/Launcher.png)
  -h, --help            Show this help
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
        heyuan | trna | zza)
            ;;
        *)
            die "unsupported profile: $1 (expected heyuan, trna, or zza)"
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
    local owner
    local mode

    reject_control_characters "desktop directory" "$input"
    logical_path="$(realpath -sm -- "$input")"
    physical_path="$(realpath -e -- "$input")" \
        || die "desktop directory does not exist: $input"
    [[ -d "$physical_path" ]] || die "desktop path is not a directory: $input"
    [[ "$logical_path" == "$physical_path" ]] \
        || die "desktop directory must not contain symlink components: $input"

    case "$physical_path" in
        / | /bin | /boot | /dev | /etc | /home | /lib | /lib64 | /opt | \
            /proc | /root | /run | /sbin | /srv | /sys | /tmp | /usr | /var | \
            /var/tmp)
            die "refusing dangerous desktop directory: $physical_path"
            ;;
    esac

    if [[ -d "${HOME:-}" ]]; then
        [[ "$physical_path" != "$(realpath -e -- "$HOME")" ]] \
            || die "refusing to install directly into HOME"
    fi
    [[ "$physical_path" != "$PROJECT_ROOT" ]] \
        || die "refusing to install into the repository root"

    owner="$(stat -c '%u' -- "$physical_path")"
    [[ "$owner" == "$UID" ]] \
        || die "desktop directory must be owned by uid $UID: $physical_path"
    mode="$(stat -c '%a' -- "$physical_path")"
    (( (8#$mode & 0002) == 0 )) \
        || die "desktop directory must not be world-writable: $physical_path"
    [[ -w "$physical_path" && -x "$physical_path" ]] \
        || die "desktop directory is not writable: $physical_path"

    printf '%s\n' "$physical_path"
}

canonical_icon() {
    local input="$1"
    local physical_path

    reject_control_characters "icon path" "$input"
    physical_path="$(realpath -e -- "$input")" \
        || die "icon does not exist: $input"
    [[ -f "$physical_path" && -r "$physical_path" ]] \
        || die "icon must be a readable regular file: $input"
    printf '%s\n' "$physical_path"
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

desktop_string() {
    local value="$1"
    reject_control_characters "desktop value" "$value"
    value="${value//\\/\\\\}"
    printf '%s' "$value"
}

desktop_exec_argument() {
    local value="$1"
    validate_desktop_exec_path "desktop executable path" "$value"
    printf '"%s"' "$value"
}

render_template() {
    local line
    local replacements=0

    while IFS= read -r line || [[ -n "$line" ]]; do
        case "$line" in
            'Name=Matrix SONIC (@MATRIX_PROFILE@)')
                printf 'Name=Matrix SONIC (%s)\n' "$PROFILE"
                ;;
            'Exec=@MATRIX_START_EXEC@')
                printf 'Exec=/usr/bin/bash %s start --profile %s\n' \
                    "$(desktop_exec_argument "$LAUNCHER")" "$PROFILE"
                ;;
            'Icon=@MATRIX_ICON@')
                printf 'Icon=%s\n' "$(desktop_string "$ICON_PATH")"
                ;;
            'X-Matrix-Repository=@MATRIX_REPOSITORY@')
                printf 'X-Matrix-Repository=%s\n' "$(desktop_string "$PROJECT_ROOT")"
                ;;
            'X-Matrix-Profile=@MATRIX_PROFILE@')
                printf 'X-Matrix-Profile=%s\n' "$PROFILE"
                ;;
            'Exec=@MATRIX_STATUS_EXEC@')
                printf 'Exec=/usr/bin/bash %s status --profile %s\n' \
                    "$(desktop_exec_argument "$LAUNCHER")" "$PROFILE"
                ;;
            'Exec=@MATRIX_STOP_EXEC@')
                printf 'Exec=/usr/bin/bash %s stop --profile %s\n' \
                    "$(desktop_exec_argument "$LAUNCHER")" "$PROFILE"
                ;;
            *)
                printf '%s\n' "$line"
                continue
                ;;
        esac
        replacements=$((replacements + 1))
    done < "$TEMPLATE"

    [[ "$replacements" == 7 ]] \
        || die "desktop template replacement count was $replacements, expected 7"
}

while (($# > 0)); do
    case "$1" in
        --profile)
            require_value "$1" "$#"
            PROFILE="$2"
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
        *)
            die "unsupported argument: $1"
            ;;
    esac
done

validate_profile "$PROFILE"
[[ -f "$TEMPLATE" && -r "$TEMPLATE" ]] \
    || die "desktop template is missing: $TEMPLATE"
[[ -f "$LAUNCHER" && -r "$LAUNCHER" ]] \
    || die "desktop launcher is missing: $LAUNCHER"
validate_desktop_exec_path "repository path" "$PROJECT_ROOT"

if [[ -z "$DESKTOP_DIR" ]]; then
    DESKTOP_DIR="$(default_desktop_dir)"
fi
DESKTOP_DIR="$(canonical_private_directory "$DESKTOP_DIR")"
ICON_PATH="$(canonical_icon "$ICON_PATH")"
validate_desktop_exec_path "icon path" "$ICON_PATH"

TARGET="$DESKTOP_DIR/matrix-sonic.desktop"
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
TEMPORARY="$(mktemp "$DESKTOP_DIR/.matrix-sonic.desktop.tmp.XXXXXX")"
cleanup() {
    if [[ -n "${TEMPORARY:-}" && -e "$TEMPORARY" ]]; then
        rm -f -- "$TEMPORARY"
    fi
}
trap cleanup EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

render_template > "$TEMPORARY"
chmod 0755 -- "$TEMPORARY"
mv -T -- "$TEMPORARY" "$TARGET"
TEMPORARY=""
trap - EXIT HUP INT TERM
if command -v gio >/dev/null 2>&1; then
    gio set "$TARGET" metadata::trusted true >/dev/null 2>&1 || true
fi

printf 'Installed Matrix SONIC desktop launcher: %s\n' "$TARGET"
printf 'Profile: %s\n' "$PROFILE"
printf 'Repository: %s\n' "$PROJECT_ROOT"
