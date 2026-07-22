#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1

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
AUDIT_VENV="$PROJECT_ROOT/.venv-audit"

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

# shellcheck disable=SC1091
source "$SCRIPT_DIR/matrix_local_env.sh"
if ! load_matrix_local_env "$PROJECT_ROOT"; then
    exit 2
fi
if [[ -n "$RUNTIME_OVERRIDE" ]]; then
    export MATRIX_RUNTIME_ROOT="$RUNTIME_OVERRIDE"
fi
# Load profile defaults after host-local overrides so paths derived from
# MATRIX_RUNTIME_ROOT always follow the selected runtime bundle.
# shellcheck disable=SC1090
source "$PROFILE_FILE"

RUNTIME_ROOT="${RUNTIME_OVERRIDE:-${MATRIX_RUNTIME_ROOT:-$PROJECT_ROOT/outputs/runtime/matrix-sonic-native-v2}}"
RUNTIME_ROOT="$(realpath -m "$RUNTIME_ROOT")"
MATRIX_SONIC_ROOT="${MATRIX_SONIC_ROOT:-$RUNTIME_ROOT/GR00T-WholeBodyControl}"
MATRIX_SONIC_ROOT="$(realpath -m "$MATRIX_SONIC_ROOT")"
if [[ -n "$RELEASE_CACHE" ]]; then
    RELEASE_CACHE="$(realpath -m "$RELEASE_CACHE")"
fi
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"
for required_command in find python3 sha256sum; do
    command -v "$required_command" >/dev/null || {
        echo "[ERROR] Required bootstrap command is unavailable: $required_command" >&2
        exit 1
    }
done
python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
    --lock "$LOCK_FILE" \
    --schema-only
mapfile -t LOCK_PYTHON_METADATA < <(python3 - "$LOCK_FILE" "$PROJECT_ROOT" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

lock_path = Path(sys.argv[1]).resolve()
project_root = Path(sys.argv[2]).resolve()
lock_bytes = lock_path.read_bytes()
lock = json.loads(lock_bytes)
python_lock = lock["python"]
requirements = (project_root / python_lock["requirements"]).resolve()
if not requirements.is_relative_to(project_root):
    raise SystemExit("python.requirements escapes the Matrix project")
requirements_bytes = requirements.read_bytes()
digest = hashlib.sha256()
digest.update(lock_bytes)
digest.update(b"\0")
digest.update(requirements_bytes)
digest.update(b"\0matrix-wheel-record-v3-no-compile")
print(requirements)
print(python_lock["requirements_sha256"])
print(digest.hexdigest())
print(python_lock["version"])
PY
)
if [[ "${#LOCK_PYTHON_METADATA[@]}" != "4" ]]; then
    echo "[ERROR] Failed to read Python metadata from runtime lock" >&2
    exit 1
fi
REQUIREMENTS_FILE="${LOCK_PYTHON_METADATA[0]}"
EXPECTED_REQUIREMENTS_SHA256="${LOCK_PYTHON_METADATA[1]}"
EXPECTED_VENV_DIGEST="${LOCK_PYTHON_METADATA[2]}"
LOCKED_PYTHON_VERSION="${LOCK_PYTHON_METADATA[3]}"
actual_requirements_sha256="$(sha256sum "$REQUIREMENTS_FILE" | awk '{print $1}')"
if [[ "$actual_requirements_sha256" != "$EXPECTED_REQUIREMENTS_SHA256" ]]; then
    echo "[ERROR] Python requirements SHA256 does not match runtime lock" >&2
    echo "[ERROR] expected=$EXPECTED_REQUIREMENTS_SHA256 actual=$actual_requirements_sha256" >&2
    exit 1
fi

select_runtime_python() {
    if [[ -n "${MATRIX_SONIC_PYTHON:-}" ]]; then
        RUNTIME_PYTHON="$MATRIX_SONIC_PYTHON"
    elif [[ -x "$AUDIT_VENV/bin/python" ]]; then
        RUNTIME_PYTHON="$AUDIT_VENV/bin/python"
    else
        echo "[ERROR] No actual runtime Python is available; set MATRIX_SONIC_PYTHON or bootstrap .venv-audit" >&2
        exit 1
    fi
    if [[ "$RUNTIME_PYTHON" == "$AUDIT_VENV/bin/python" ]]; then
        local digest_marker="$AUDIT_VENV/.matrix-lock-requirements.sha256"
        local actual_digest=""
        if [[ -f "$digest_marker" ]]; then
            actual_digest="$(<"$digest_marker")"
        fi
        if [[ "$actual_digest" != "$EXPECTED_VENV_DIGEST" ]]; then
            echo "[ERROR] .venv-audit does not match the current runtime lock; rerun without --skip-python" >&2
            exit 1
        fi
    fi
}

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
        EXTERNAL_PIP_MARKER="$AUDIT_VENV/.matrix-external-pip"
        VENV_DIGEST_MARKER="$AUDIT_VENV/.matrix-lock-requirements.sha256"
        if ! command -v "$BOOTSTRAP_PYTHON" >/dev/null; then
            echo "[ERROR] Bootstrap interpreter is unavailable: $BOOTSTRAP_PYTHON" >&2
            exit 1
        fi
        bootstrap_python_version="$($BOOTSTRAP_PYTHON -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
        if [[ "$bootstrap_python_version" != "$LOCKED_PYTHON_VERSION" ]]; then
            echo "[ERROR] Bootstrap Python does not match runtime lock: expected=$LOCKED_PYTHON_VERSION actual=$bootstrap_python_version" >&2
            exit 1
        fi
        if [[ -x "$AUDIT_VENV/bin/python" ]]; then
            actual_venv_digest=""
            if [[ -f "$VENV_DIGEST_MARKER" ]]; then
                actual_venv_digest="$(<"$VENV_DIGEST_MARKER")"
            fi
            if [[ "$actual_venv_digest" != "$EXPECTED_VENV_DIGEST" ]]; then
                echo "[INFO] Recreating .venv-audit: runtime lock or requirements changed"
                rm -rf "$AUDIT_VENV"
            fi
        fi
        if [[ -x "$AUDIT_VENV/bin/python" ]]; then
            actual_python="$($AUDIT_VENV/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
            if [[ "$actual_python" != "$LOCKED_PYTHON_VERSION" ]]; then
                echo "[INFO] Recreating .venv-audit: Python $actual_python -> $LOCKED_PYTHON_VERSION"
                rm -rf "$AUDIT_VENV"
            fi
        fi
        if [[ -x "$AUDIT_VENV/bin/python" ]]; then
            if ! /usr/bin/python3 -I - "$AUDIT_VENV/pyvenv.cfg" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
if path.is_symlink() or not path.is_file():
    raise SystemExit(1)
values = []
for line in path.read_text(encoding="utf-8").splitlines():
    if "=" not in line:
        continue
    key, value = line.split("=", 1)
    if key.strip().lower() == "include-system-site-packages":
        values.append(value.strip().lower())
raise SystemExit(0 if values == ["false"] else 1)
PY
            then
                echo "[INFO] Recreating non-isolated .venv-audit"
                rm -rf "$AUDIT_VENV"
            fi
        fi
        if [[ -x "$AUDIT_VENV/bin/python" && -f "$EXTERNAL_PIP_MARKER" ]]; then
            mapfile -t external_pip_roots < "$EXTERNAL_PIP_MARKER"
            external_pip_root="${external_pip_roots[0]:-}"
            if [[ "${#external_pip_roots[@]}" != "1" \
                || "$external_pip_root" != "$AUDIT_VENV/.matrix-pip-runner" \
                || ! -f "$external_pip_root/pip/__init__.py" ]] \
                || ! PYTHONPATH="$($AUDIT_VENV/bin/python -c \
                    'import site; print(site.getsitepackages()[0])'):$external_pip_root" \
                    "$AUDIT_VENV/bin/python" -m pip --version >/dev/null 2>&1; then
                echo "[INFO] Recreating incomplete .venv-audit: external pip metadata is invalid"
                rm -rf "$AUDIT_VENV"
            fi
        fi
        if [[ -x "$AUDIT_VENV/bin/python" && ! -f "$EXTERNAL_PIP_MARKER" ]]; then
            echo "[INFO] Recreating incomplete .venv-audit: isolated pip runner is absent"
            rm -rf "$AUDIT_VENV"
        fi
        if [[ ! -x "$AUDIT_VENV/bin/python" ]]; then
            if "$BOOTSTRAP_PYTHON" -c 'import ensurepip' >/dev/null 2>&1; then
                "$BOOTSTRAP_PYTHON" -m venv "$AUDIT_VENV"
                audit_site_packages="$($AUDIT_VENV/bin/python -c \
                    'import site; print(site.getsitepackages()[0])')"
                external_pip_root="$AUDIT_VENV/.matrix-pip-runner"
                mkdir -p "$external_pip_root"
                if [[ ! -f "$audit_site_packages/pip/__init__.py" ]]; then
                    echo "[ERROR] ensurepip did not create a usable pip package" >&2
                    exit 1
                fi
                mv "$audit_site_packages/pip" "$external_pip_root/pip"
                # Remove every ensurepip seed from runtime site-packages. The
                # locked resolver will reinstall the exact setuptools wheel.
                find "$audit_site_packages" -mindepth 1 -maxdepth 1 \
                    -exec rm -rf -- {} +
            else
                if ! "$BOOTSTRAP_PYTHON" -m pip --version >/dev/null 2>&1; then
                    echo "[ERROR] Python has neither ensurepip nor a system pip fallback" >&2
                    echo "[ERROR] Install python3-venv or make pip available to $BOOTSTRAP_PYTHON" >&2
                    exit 1
                fi
                echo "[WARN] ensurepip is unavailable; using system pip as an isolated installer"
                "$BOOTSTRAP_PYTHON" -m venv --without-pip "$AUDIT_VENV"
                host_pip_package="$($BOOTSTRAP_PYTHON -c \
                    'import pathlib,pip; print(pathlib.Path(pip.__file__).resolve().parent)')"
                external_pip_root="$AUDIT_VENV/.matrix-pip-runner"
                mkdir -p "$external_pip_root"
                ln -s "$host_pip_package" "$external_pip_root/pip"
            fi
            temporary_external_pip_marker="${EXTERNAL_PIP_MARKER}.tmp.$$"
            printf '%s\n' "$external_pip_root" > "$temporary_external_pip_marker"
            mv "$temporary_external_pip_marker" "$EXTERNAL_PIP_MARKER"
        fi
        external_pip_root="$(<"$EXTERNAL_PIP_MARKER")"
        audit_site_packages="$($AUDIT_VENV/bin/python -c \
            'import site; print(site.getsitepackages()[0])')"
        PIP_COMMAND=(
            /usr/bin/env PYTHONDONTWRITEBYTECODE=1
            PYTHONPATH="$audit_site_packages:$external_pip_root"
            "$AUDIT_VENV/bin/python" -m pip install
            --target "$audit_site_packages" --upgrade --ignore-installed
            --no-compile
        )
        WHEELHOUSE="$RUNTIME_ROOT/python-wheelhouse"
        PIP_ARGS=(--no-index --only-binary=:all: --find-links "$WHEELHOUSE")
        if [[ -d "$WHEELHOUSE" && -f "$WHEELHOUSE/SHA256SUMS" ]]; then
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
            PYTHONDONTWRITEBYTECODE=1 \
                python3 - "$SCRIPT_DIR" "$WHEELHOUSE" "$expected_manifest" <<'PY'
import sys
from pathlib import Path

sys.path.insert(0, sys.argv[1])
import verify_matrix_sonic_runtime as verifier

checks = verifier.verify_wheelhouse(Path(sys.argv[2]), sys.argv[3])
for name, ok, detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")
if not all(ok for _, ok, _ in checks):
    raise SystemExit("locked offline wheelhouse verification failed")
PY
            echo "[INFO] Installing Python dependencies from locked offline wheelhouse"
        else
            echo "[ERROR] Locked offline Python wheelhouse is absent: $WHEELHOUSE" >&2
            exit 1
        fi
        "${PIP_COMMAND[@]}" \
            "${PIP_ARGS[@]}" \
            -r "$REQUIREMENTS_FILE"
        PYTHONDONTWRITEBYTECODE=1 "$AUDIT_VENV/bin/python" - \
            "$REQUIREMENTS_FILE" <<'PY'
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
        PYTHONDONTWRITEBYTECODE=1 \
            PYTHONPATH="$audit_site_packages:$external_pip_root" \
            "$AUDIT_VENV/bin/python" -m pip check
        temporary_digest_marker="${VENV_DIGEST_MARKER}.tmp.$$"
        printf '%s\n' "$EXPECTED_VENV_DIGEST" > "$temporary_digest_marker"
        mv "$temporary_digest_marker" "$VENV_DIGEST_MARKER"
    fi

    select_runtime_python

    if [[ -n "$RELEASE_CACHE" ]]; then
        PYTHONDONTWRITEBYTECODE=1 \
            python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
            --lock "$LOCK_FILE" \
            --runtime-root "$RUNTIME_ROOT" \
            --matrix-root "$PROJECT_ROOT" \
            --sonic-root "$MATRIX_SONIC_ROOT" \
            --python "$RUNTIME_PYTHON" \
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
map_packages = [
    item
    for item in release["packages"]
    if item["name"] not in {"assets", "base", "shared"}
]

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
                "name": item["name"],
                "file": item["file"],
                "required": False,
                "size": item["size"],
                "sha256": item["sha256"],
            }
            for item in map_packages
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
        LOCKED_MATRIX_MAPS="$(python3 - "$LOCK_FILE" <<'PY'
import json
import sys

release = json.load(open(sys.argv[1], encoding="utf-8"))["matrix_release"]
maps = [
    item["name"]
    for item in release["packages"]
    if item["name"] not in {"assets", "base", "shared"}
]
print(" ".join(maps))
PY
        )"
        INSTALL_ENV=(MATRIX_MAPS="$LOCKED_MATRIX_MAPS" MATRIX_ASSUME_YES=1)
        if [[ -n "$RELEASE_CACHE" ]]; then
            INSTALL_ENV+=(MATRIX_OFFLINE=1)
        fi
        /usr/bin/env "${INSTALL_ENV[@]}" \
            bash "$PROJECT_ROOT/scripts/release_manager/install_chunks.sh" 0.1.2
    fi

    deploy="$MATRIX_SONIC_ROOT/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
    [[ -f "$deploy" ]] && chmod +x "$deploy"

    if [[ "$MATRIX_ROS_PREFIX" == "$RUNTIME_ROOT/ros2-humble-prefix" ]]; then
        rmw_dir="$RUNTIME_ROOT/ros2-humble-prefix/lib"
        ue_rmw="$PROJECT_ROOT/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/librmw_fastrtps_cpp.so"
        mkdir -p "$rmw_dir"
        if [[ -f "$ue_rmw" ]]; then
            ln -sfn "$ue_rmw" "$rmw_dir/librmw_fastrtps_cpp.so"
        fi
    fi
fi

if [[ -z "${RUNTIME_PYTHON:-}" ]]; then
    select_runtime_python
fi

VERIFY_ARGS=(
    --lock "$LOCK_FILE"
    --runtime-root "$RUNTIME_ROOT"
    --matrix-root "$PROJECT_ROOT"
    --sonic-root "$MATRIX_SONIC_ROOT"
    --profile "$PROFILE"
    --python "$RUNTIME_PYTHON"
    --json-output "$PROJECT_ROOT/outputs/runtime-verification-$PROFILE.json"
)
if [[ -n "$RELEASE_CACHE" ]]; then
    VERIFY_ARGS+=(--release-cache "$RELEASE_CACHE")
fi
if [[ "$SKIP_ASSETS" == "1" ]]; then
    VERIFY_ARGS+=(--skip-installed-assets)
fi

PYTHONDONTWRITEBYTECODE=1 \
    python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" "${VERIFY_ARGS[@]}"
if [[ "$WRITE_LOCAL_ENV" == "1" ]]; then
    mkdir -p "$PROJECT_ROOT/.matrix"
    python3 "$SCRIPT_DIR/update_matrix_local_env.py" \
        "$PROJECT_ROOT/.matrix/local.env" MATRIX_RUNTIME_ROOT "$RUNTIME_ROOT"
    echo "[INFO] Updated ignored host override: $PROJECT_ROOT/.matrix/local.env"
fi
echo "[PASS] Matrix SONIC bootstrap complete: profile=$PROFILE runtime=$RUNTIME_ROOT"
