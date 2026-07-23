#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# shellcheck disable=SC1091
source "$SCRIPT_DIR/matrix_local_env.sh"
load_matrix_local_env "$PROJECT_ROOT"

MANIFEST="$PROJECT_ROOT/config/universe/de440s-2080.lock.json"
ROOT="${MATRIX_CELESTIAL_ROOT:-${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2}/celestial}"
VERIFY_ONLY=0

usage() {
    printf '%s\n' \
        "Usage: bash scripts/bootstrap_matrix_celestial.sh [options]" \
        "  --root PATH       Celestial asset directory" \
        "  --manifest PATH   Locked asset manifest" \
        "  --verify-only     Do not download missing assets"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root) ROOT="$2"; shift 2 ;;
        --manifest) MANIFEST="$2"; shift 2 ;;
        --verify-only) VERIFY_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$ROOT" != /* || "$MANIFEST" != /* ]]; then
    echo "[ERROR] Celestial root and manifest must be absolute" >&2
    exit 2
fi
if [[ ! -f "$MANIFEST" || -L "$MANIFEST" ]]; then
    echo "[ERROR] Celestial manifest must be a regular file: $MANIFEST" >&2
    exit 2
fi
for command in python3 sha256sum stat; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "[ERROR] Missing required command: $command" >&2
        exit 1
    fi
done
if ! command -v curl >/dev/null 2>&1 && ! command -v wget >/dev/null 2>&1; then
    echo "[ERROR] curl or wget is required to provision celestial assets" >&2
    exit 1
fi

mapfile -t ASSETS < <(
    /usr/bin/python3 -I - "$MANIFEST" <<'PY'
import json
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
if set(value) != {"schema", "provider", "coverage", "assets"}:
    raise SystemExit("invalid celestial asset manifest root")
if value["schema"] != "matrix-celestial-assets/v1":
    raise SystemExit("unsupported celestial asset manifest")
if value["provider"] != "jpl-de440s-v1":
    raise SystemExit("unexpected celestial provider")
assets = value["assets"]
if not isinstance(assets, list) or len(assets) != 2:
    raise SystemExit("invalid celestial asset list")
roles = set()
for item in assets:
    if not isinstance(item, dict) or set(item) != {
        "role", "filename", "size", "sha256", "urls"
    }:
        raise SystemExit("invalid celestial asset entry")
    role = item["role"]
    filename = item["filename"]
    size = item["size"]
    digest = item["sha256"]
    urls = item["urls"]
    if (
        not isinstance(role, str)
        or not re.fullmatch(r"[a-z0-9_]{1,64}", role)
        or role in roles
        or not isinstance(filename, str)
        or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}", filename)
        or isinstance(size, bool)
        or not isinstance(size, int)
        or size <= 0
        or not isinstance(digest, str)
        or not re.fullmatch(r"[0-9a-f]{64}", digest)
        or not isinstance(urls, list)
        or not urls
        or any(
            not isinstance(url, str)
            or not url.startswith("https://")
            or "\t" in url
            or "|" in url
            for url in urls
        )
    ):
        raise SystemExit("unsafe celestial asset entry")
    roles.add(role)
    print("\t".join((role, filename, str(size), digest, "|".join(urls))))
if roles != {"de440s_spk", "jplephem_wheel"}:
    raise SystemExit("celestial asset roles do not match the provider")
PY
)

mkdir -p "$ROOT"
ROOT="$(realpath -m "$ROOT")"

verify_asset() {
    local path="$1"
    local expected_size="$2"
    local expected_sha256="$3"
    [[ -f "$path" && ! -L "$path" ]] \
        && [[ "$(stat -c '%s' "$path")" == "$expected_size" ]] \
        && [[ "$(sha256sum "$path" | awk '{print $1}')" == "$expected_sha256" ]]
}

for record in "${ASSETS[@]}"; do
    IFS=$'\t' read -r role filename expected_size expected_sha256 urls <<<"$record"
    target="$ROOT/$filename"
    if verify_asset "$target" "$expected_size" "$expected_sha256"; then
        echo "[PASS] celestial asset $role: $target"
        continue
    fi
    if [[ "$VERIFY_ONLY" == "1" ]]; then
        echo "[ERROR] Missing or invalid celestial asset $role: $target" >&2
        exit 1
    fi
    temporary="$(mktemp "$ROOT/.${filename}.XXXXXX.tmp")"
    downloaded=0
    IFS='|' read -r -a candidates <<<"$urls"
    for url in "${candidates[@]}"; do
        rm -f -- "$temporary"
        temporary="$(mktemp "$ROOT/.${filename}.XXXXXX.tmp")"
        echo "[INFO] Downloading $role from $url"
        if command -v curl >/dev/null 2>&1; then
            if curl --fail --location --retry 2 --connect-timeout 20 \
                --max-time 300 --output "$temporary" "$url"; then
                downloaded=1
            fi
        elif wget --https-only --timeout=20 --tries=3 \
            --output-document="$temporary" "$url"; then
            downloaded=1
        fi
        if [[ "$downloaded" == "1" ]] \
            && verify_asset "$temporary" "$expected_size" "$expected_sha256"; then
            break
        fi
        downloaded=0
    done
    if [[ "$downloaded" != "1" ]]; then
        rm -f -- "$temporary"
        echo "[ERROR] Could not download a verified $role" >&2
        exit 1
    fi
    chmod 0644 "$temporary"
    mv -f -- "$temporary" "$target"
    echo "[PASS] installed celestial asset $role: $target"
done

echo "[PASS] Matrix celestial DE440s runtime is ready"
printf 'export MATRIX_CELESTIAL_SPK=%q\n' "$ROOT/de440s.bsp"
printf 'export MATRIX_CELESTIAL_JPLEPHEM_WHEEL=%q\n' \
    "$ROOT/jplephem-2.23-py3-none-any.whl"
