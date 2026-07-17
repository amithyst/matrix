#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
export MATRIX_PROJECT_ROOT="$PROJECT_ROOT"

PROFILE="${MATRIX_PROFILE:-}"
ARTIFACT_SOURCE="${MATRIX_ARTIFACT_SOURCE:-}"
RELEASE_CACHE="${MATRIX_RELEASE_CACHE:-}"
RUNTIME_OVERRIDE=""
WRITE_LOCAL_ENV=0
SKIP_ASSETS=0
SKIP_PYTHON=0
VERIFY_ONLY=0

usage() {
    cat <<'EOF'
Usage: bash scripts/bootstrap_matrix_sonic.sh --profile NAME [options]

Options:
  --artifact-source PATH|HOST:PATH  Locked runtime bundle source
  --release-cache PATH              Existing Matrix 0.1.2 archives
  --runtime-root PATH               Use an existing host-local runtime directory
  --write-local-env                 Persist --runtime-root in ignored .matrix/local.env
  --skip-assets                     Do not install Matrix release/map packages
  --skip-python                     Do not create/update .venv-audit
  --verify-only                     Do not copy/install; run full verification
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile) PROFILE="$2"; shift 2 ;;
        --artifact-source) ARTIFACT_SOURCE="$2"; shift 2 ;;
        --release-cache) RELEASE_CACHE="$2"; shift 2 ;;
        --runtime-root) RUNTIME_OVERRIDE="$2"; shift 2 ;;
        --write-local-env) WRITE_LOCAL_ENV=1; shift ;;
        --skip-assets) SKIP_ASSETS=1; shift ;;
        --skip-python) SKIP_PYTHON=1; shift ;;
        --verify-only) VERIFY_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$PROFILE" ]]; then
    echo "[ERROR] --profile is required" >&2
    exit 2
fi
PROFILE_FILE="$PROJECT_ROOT/config/hosts/$PROFILE.env"
if [[ ! -f "$PROFILE_FILE" ]]; then
    echo "[ERROR] Unknown host profile: $PROFILE" >&2
    exit 2
fi
if [[ "$WRITE_LOCAL_ENV" == "1" && -z "$RUNTIME_OVERRIDE" ]]; then
    echo "[ERROR] --write-local-env requires --runtime-root" >&2
    exit 2
fi

if [[ -f "$PROJECT_ROOT/.matrix/local.env" ]]; then
    # shellcheck disable=SC1091
    source "$PROJECT_ROOT/.matrix/local.env"
fi
if [[ -n "$RUNTIME_OVERRIDE" ]]; then
    export MATRIX_RUNTIME_ROOT="$RUNTIME_OVERRIDE"
fi
# Load profile defaults after host-local overrides so paths derived from
# MATRIX_RUNTIME_ROOT always follow the selected runtime bundle.
# shellcheck disable=SC1090
source "$PROFILE_FILE"

RUNTIME_ROOT="${RUNTIME_OVERRIDE:-${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-v1}}"
RUNTIME_ROOT="$(realpath -m "$RUNTIME_ROOT")"
if [[ -n "$RELEASE_CACHE" ]]; then
    RELEASE_CACHE="$(realpath -m "$RELEASE_CACHE")"
fi
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"
mkdir -p "$PROJECT_ROOT/outputs/logs" "$PROJECT_ROOT/releases" "$(dirname "$RUNTIME_ROOT")"

if [[ "$VERIFY_ONLY" != "1" ]]; then
    if [[ -n "$ARTIFACT_SOURCE" ]]; then
        command -v rsync >/dev/null || {
            echo "[ERROR] rsync is required to copy the locked runtime bundle" >&2
            exit 1
        }
        mkdir -p "$RUNTIME_ROOT"
        echo "[INFO] Syncing locked runtime from $ARTIFACT_SOURCE"
        rsync -aL --delete "${ARTIFACT_SOURCE%/}/" "$RUNTIME_ROOT/"
    elif [[ ! -d "$RUNTIME_ROOT" ]]; then
        echo "[ERROR] Runtime is absent and --artifact-source was not provided" >&2
        exit 1
    fi

    if [[ "$SKIP_PYTHON" != "1" ]]; then
        BOOTSTRAP_PYTHON="${MATRIX_BOOTSTRAP_PYTHON:-python3}"
        AUDIT_VENV="$PROJECT_ROOT/.venv-audit"
        if ! command -v "$BOOTSTRAP_PYTHON" >/dev/null; then
            echo "[ERROR] Bootstrap interpreter is unavailable: $BOOTSTRAP_PYTHON" >&2
            exit 1
        fi
        expected_python="$($BOOTSTRAP_PYTHON -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
        if [[ -x "$AUDIT_VENV/bin/python" ]]; then
            actual_python="$($AUDIT_VENV/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
            if [[ "$actual_python" != "$expected_python" ]]; then
                echo "[INFO] Recreating .venv-audit: Python $actual_python -> $expected_python"
                rm -rf "$AUDIT_VENV"
            fi
        fi
        if [[ -x "$AUDIT_VENV/bin/python" ]] \
            && ! "$AUDIT_VENV/bin/python" -m pip --version >/dev/null 2>&1; then
            echo "[INFO] Recreating incomplete .venv-audit: pip is unavailable"
            rm -rf "$AUDIT_VENV"
        fi
        if [[ ! -x "$AUDIT_VENV/bin/python" ]]; then
            if "$BOOTSTRAP_PYTHON" -c 'import ensurepip' >/dev/null 2>&1; then
                "$BOOTSTRAP_PYTHON" -m venv "$AUDIT_VENV"
            else
                if ! "$BOOTSTRAP_PYTHON" -m pip --version >/dev/null 2>&1; then
                    echo "[ERROR] Python has neither ensurepip nor a system pip fallback" >&2
                    echo "[ERROR] Install python3-venv or make pip available to $BOOTSTRAP_PYTHON" >&2
                    exit 1
                fi
                echo "[WARN] ensurepip is unavailable; creating .venv-audit with system pip"
                "$BOOTSTRAP_PYTHON" -m venv \
                    --without-pip --system-site-packages "$AUDIT_VENV"
            fi
        fi
        WHEELHOUSE="$RUNTIME_ROOT/python-wheelhouse"
        PIP_ARGS=()
        if [[ -d "$WHEELHOUSE" ]]; then
            expected_manifest="$(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1], encoding="utf-8"))["python"]["wheelhouse_manifest_sha256"])
PY
            )"
            actual_manifest="$(sha256sum "$WHEELHOUSE/SHA256SUMS" | awk '{print $1}')"
            if [[ "$actual_manifest" != "$expected_manifest" ]]; then
                echo "[ERROR] Wheelhouse manifest SHA256 mismatch" >&2
                exit 1
            fi
            (cd "$WHEELHOUSE" && sha256sum -c SHA256SUMS)
            PIP_ARGS+=(--no-index --find-links "$WHEELHOUSE")
            if grep -Eq '^include-system-site-packages[[:space:]]*=[[:space:]]*true$' \
                "$AUDIT_VENV/pyvenv.cfg"; then
                # Do not let host packages satisfy the lock in the fallback venv.
                PIP_ARGS+=(--ignore-installed)
            fi
            echo "[INFO] Installing Python dependencies from locked offline wheelhouse"
        else
            echo "[WARN] Locked wheelhouse is absent; using configured Python package index"
        fi
        "$AUDIT_VENV/bin/python" -m pip install \
            "${PIP_ARGS[@]}" \
            -r "$PROJECT_ROOT/research/sonic_integration/requirements-trna.txt"
        "$AUDIT_VENV/bin/python" - \
            "$PROJECT_ROOT/research/sonic_integration/requirements-trna.txt" <<'PY'
import importlib.metadata
import pathlib
import sys

requirements = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
for line in requirements:
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    name, expected = line.split("==", 1)
    actual = importlib.metadata.version(name)
    if actual != expected:
        raise SystemExit(f"{name}: expected {expected}, got {actual}")
print("[PASS] Python dependency versions match requirements lock")
PY
    fi

    if [[ -n "$RELEASE_CACHE" ]]; then
        python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
            --lock "$LOCK_FILE" \
            --runtime-root "$RUNTIME_ROOT" \
            --matrix-root "$PROJECT_ROOT" \
            --release-cache "$RELEASE_CACHE" \
            --skip-dynamic \
            --skip-installed-assets \
            --fast

        materialize_release_package() {
            local source_path="$1"
            local destination_path="$2"

            if [[ "$source_path" == "$destination_path" ]]; then
                return 0
            fi
            rm -f "$destination_path" "${destination_path}.aria2"
            if ln "$source_path" "$destination_path" 2>/dev/null; then
                echo "[INFO] Reused release package by hard link: $(basename "$destination_path")"
            else
                cp --reflink=auto --preserve=mode,timestamps \
                    "$source_path" "$destination_path"
                echo "[INFO] Materialized release package locally: $(basename "$destination_path")"
            fi
        }

        while IFS= read -r package; do
            source_path="${RELEASE_CACHE%/}/$package"
            if [[ ! -f "$source_path" ]]; then
                echo "[ERROR] Release cache is missing: $source_path" >&2
                exit 1
            fi
            materialize_release_package \
                "$source_path" "$PROJECT_ROOT/releases/$package"
        done < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
for item in json.load(open(sys.argv[1], encoding="utf-8"))["matrix_release"]["packages"]:
    print(item["file"])
PY
        )

        python3 - "$LOCK_FILE" "$PROJECT_ROOT/releases" <<'PY'
import json
import os
from pathlib import Path
import sys

lock = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
release = lock["matrix_release"]
packages = {item["name"]: dict(item) for item in release["packages"]}

def package(name: str, *, required: bool) -> dict[str, object]:
    item = packages[name]
    return {
        "file": item["file"],
        "required": required,
        "size": item["size"],
        "sha256": item["sha256"],
    }

payload = {
    "version": release["version"],
    "packages": {
        "base": package("base", required=True),
        "assets": package("assets", required=True),
        "shared": {
            **package("shared", required=False),
            "is_split": False,
        },
        "maps": [
            {
                "name": "Town10World",
                **package("Town10World", required=False),
            }
        ],
    },
}
destination = Path(sys.argv[2]) / f"manifest-{release['version']}.json"
temporary = destination.with_suffix(destination.suffix + f".tmp.{os.getpid()}")
temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(temporary, destination)
PY
    fi

    if [[ "$SKIP_ASSETS" != "1" ]]; then
        INSTALL_ENV=(MATRIX_MAPS=Town10World MATRIX_ASSUME_YES=1)
        if [[ -n "$RELEASE_CACHE" ]]; then
            INSTALL_ENV+=(MATRIX_OFFLINE=1)
        fi
        /usr/bin/env "${INSTALL_ENV[@]}" \
            bash "$PROJECT_ROOT/scripts/release_manager/install_chunks.sh" 0.1.2
    fi

    deploy="$RUNTIME_ROOT/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    bridge="$RUNTIME_ROOT/bridge/g1_sonic_sim_udp_dds_bridge_accepted"
    [[ -f "$deploy" ]] && chmod +x "$deploy"
    [[ -f "$bridge" ]] && chmod +x "$bridge"

    if [[ "$MATRIX_ROS_PREFIX" == "$RUNTIME_ROOT/ros2-humble-prefix" ]]; then
        rmw_dir="$RUNTIME_ROOT/ros2-humble-prefix/lib"
        ue_rmw="$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/librmw_fastrtps_cpp.so"
        mkdir -p "$rmw_dir"
        if [[ -f "$ue_rmw" ]]; then
            ln -sfn "$ue_rmw" "$rmw_dir/librmw_fastrtps_cpp.so"
        fi
    fi
fi

VERIFY_ARGS=(
    --lock "$LOCK_FILE"
    --runtime-root "$RUNTIME_ROOT"
    --matrix-root "$PROJECT_ROOT"
    --profile "$PROFILE"
    --json-output "$PROJECT_ROOT/outputs/runtime-verification-$PROFILE.json"
)
if [[ -n "$RELEASE_CACHE" ]]; then
    VERIFY_ARGS+=(--release-cache "$RELEASE_CACHE")
fi
if [[ "$SKIP_ASSETS" == "1" ]]; then
    VERIFY_ARGS+=(--skip-installed-assets)
fi

python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" "${VERIFY_ARGS[@]}"
if [[ "$WRITE_LOCAL_ENV" == "1" ]]; then
    mkdir -p "$PROJECT_ROOT/.matrix"
    printf 'export MATRIX_RUNTIME_ROOT=%q\n' "$RUNTIME_ROOT" \
        > "$PROJECT_ROOT/.matrix/local.env"
    echo "[INFO] Wrote ignored host override: $PROJECT_ROOT/.matrix/local.env"
fi
echo "[PASS] Matrix SONIC bootstrap complete: profile=$PROFILE runtime=$RUNTIME_ROOT"
