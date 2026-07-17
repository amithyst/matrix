#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOCK_FILE="$PROJECT_ROOT/config/runtime/matrix-sonic.lock.json"

SONIC_ROOT=""
INFERENCE_ROOT=""
VISUAL_ROOT=""
ROS_PREFIX=""
NATIVE_DEPS=""
WHEELHOUSE=""
RUNTIME_PYTHON=""
OUTPUT=""

usage() {
    cat <<'EOF'
Usage: bash scripts/package_matrix_sonic_artifacts.sh [options]

Required:
  --sonic-root PATH       Original GR00T-WholeBodyControl checkout at the pinned commit
  --inference-root PATH   Root containing TensorRT/ and onnxruntime/
  --visual-root PATH      Canonical G1 visual URDF and meshes
  --native-deps PATH      Isolated native dependency root
  --wheelhouse PATH       Offline Python wheelhouse with SHA256SUMS
  --python PATH           Actual CPython interpreter for the locked runtime
  --output PATH           New matrix-sonic-native-v2 artifact directory

Optional:
  --ros-prefix PATH       Isolated ROS2 ament prefix
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --sonic-root) SONIC_ROOT="$2"; shift 2 ;;
        --inference-root) INFERENCE_ROOT="$2"; shift 2 ;;
        --visual-root) VISUAL_ROOT="$2"; shift 2 ;;
        --ros-prefix) ROS_PREFIX="$2"; shift 2 ;;
        --native-deps) NATIVE_DEPS="$2"; shift 2 ;;
        --wheelhouse) WHEELHOUSE="$2"; shift 2 ;;
        --python) RUNTIME_PYTHON="$2"; shift 2 ;;
        --output) OUTPUT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "[ERROR] Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

for value_name in SONIC_ROOT INFERENCE_ROOT VISUAL_ROOT NATIVE_DEPS WHEELHOUSE RUNTIME_PYTHON OUTPUT; do
    if [[ -z "${!value_name}" ]]; then
        echo "[ERROR] --${value_name,,} is required" >&2
        exit 2
    fi
done
for directory in "$SONIC_ROOT" "$INFERENCE_ROOT" "$VISUAL_ROOT" "$NATIVE_DEPS" "$WHEELHOUSE"; do
    if [[ ! -d "$directory" ]]; then
        echo "[ERROR] Source directory is missing: $directory" >&2
        exit 1
    fi
done
for directory in "$ROS_PREFIX"; do
    if [[ -n "$directory" && ! -d "$directory" ]]; then
        echo "[ERROR] Optional source directory is missing: $directory" >&2
        exit 1
    fi
done
if [[ -e "$OUTPUT" ]]; then
    echo "[ERROR] Output already exists; refusing to replace it: $OUTPUT" >&2
    exit 1
fi
for required_command in cp git mkdir mv python3 realpath rm rsync tar; do
    command -v "$required_command" >/dev/null || {
        echo "[ERROR] Required command is unavailable: $required_command" >&2
        exit 1
    }
done
if ! command -v "$RUNTIME_PYTHON" >/dev/null \
    && [[ ! -x "$RUNTIME_PYTHON" ]]; then
    echo "[ERROR] Locked runtime Python is unavailable: $RUNTIME_PYTHON" >&2
    exit 1
fi

for value_name in SONIC_ROOT INFERENCE_ROOT VISUAL_ROOT WHEELHOUSE; do
    printf -v "$value_name" '%s' "$(realpath "${!value_name}")"
done
if [[ -n "$ROS_PREFIX" ]]; then
    ROS_PREFIX="$(realpath "$ROS_PREFIX")"
fi
NATIVE_DEPS="$(realpath "$NATIVE_DEPS")"
OUTPUT="$(realpath -m "$OUTPUT")"
STAGING="${OUTPUT}.tmp.$$"

path_is_within() {
    local candidate="$1"
    local root="$2"
    [[ "$candidate" == "$root" || "$candidate" == "$root/"* ]]
}

INPUT_ROOTS=("$SONIC_ROOT" "$INFERENCE_ROOT" "$VISUAL_ROOT" "$WHEELHOUSE")
if [[ -n "$ROS_PREFIX" ]]; then
    INPUT_ROOTS+=("$ROS_PREFIX")
fi
INPUT_ROOTS+=("$NATIVE_DEPS")
for input_root in "${INPUT_ROOTS[@]}"; do
    if path_is_within "$OUTPUT" "$input_root" \
        || path_is_within "$STAGING" "$input_root"; then
        echo "[ERROR] Output/staging path must not be inside an input root: $input_root" >&2
        exit 1
    fi
done

python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
    --lock "$LOCK_FILE" \
    --schema-only
echo "[WARN] Creating an acceptance-candidate bundle; support-tree release attestation is incomplete."

EXPECTED_SONIC_COMMIT="$(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1], encoding="utf-8"))["source_revisions"]["gr00t_whole_body_control"]["commit"])
PY
)"
mapfile -t CRITICAL_SOURCE_PATHS < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
lock = json.load(open(sys.argv[1], encoding="utf-8"))
for path in lock["source_revisions"]["gr00t_whole_body_control"]["critical_source_paths"]:
    print(path)
PY
)
mapfile -t SONIC_RUNTIME_FILES < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
lock = json.load(open(sys.argv[1], encoding="utf-8"))
for entry in lock["runtime_files"]:
    if entry["root"] == "sonic":
        print(entry["path"])
PY
)
mapfile -t SONIC_RUNTIME_TREES < <(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
lock = json.load(open(sys.argv[1], encoding="utf-8"))
for entry in lock["runtime_trees"]:
    if entry["root"] == "sonic":
        print(entry["path"])
PY
)

SONIC_TOPLEVEL="$(git -C "$SONIC_ROOT" rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$SONIC_TOPLEVEL" ]] \
    || [[ "$(realpath -m "$SONIC_TOPLEVEL")" != "$SONIC_ROOT" ]]; then
    echo "[ERROR] --sonic-root must be the root of an original SONIC Git checkout: $SONIC_ROOT" >&2
    exit 1
fi
ACTUAL_SONIC_COMMIT="$(git -C "$SONIC_ROOT" rev-parse HEAD)"
if [[ "$ACTUAL_SONIC_COMMIT" != "$EXPECTED_SONIC_COMMIT" ]]; then
    echo "[ERROR] SONIC HEAD does not match runtime lock" >&2
    echo "[ERROR] expected=$EXPECTED_SONIC_COMMIT actual=$ACTUAL_SONIC_COMMIT" >&2
    exit 1
fi
dirty_sources="$(git -C "$SONIC_ROOT" status --porcelain=v1 \
    --untracked-files=all -- "${CRITICAL_SOURCE_PATHS[@]}")"
if [[ -n "$dirty_sources" ]]; then
    echo "[ERROR] Critical SONIC source paths are dirty:" >&2
    printf '%s\n' "$dirty_sources" >&2
    exit 1
fi
for relative in "${CRITICAL_SOURCE_PATHS[@]}"; do
    if ! git -C "$SONIC_ROOT" cat-file -e "$EXPECTED_SONIC_COMMIT:$relative"; then
        echo "[ERROR] Locked SONIC commit is missing critical source: $relative" >&2
        exit 1
    fi
done

cleanup() {
    rm -rf "$STAGING"
}
trap cleanup EXIT
mkdir -p "$STAGING/GR00T-WholeBodyControl"
GEAR_TARGET="$STAGING/GR00T-WholeBodyControl"
# Copy the exact committed original SONIC source, excluding all unrelated dirty
# worktree files, then overlay only files and trees attested by the runtime lock.
git -C "$SONIC_ROOT" archive "$EXPECTED_SONIC_COMMIT" | tar -x -C "$GEAR_TARGET"
printf '%s\n' "$EXPECTED_SONIC_COMMIT" > "$GEAR_TARGET/SONIC_COMMIT"
for relative in "${SONIC_RUNTIME_FILES[@]}"; do
    source_path="$SONIC_ROOT/$relative"
    target_path="$GEAR_TARGET/$relative"
    if [[ ! -f "$source_path" ]]; then
        echo "[ERROR] Locked SONIC runtime file is missing: $source_path" >&2
        exit 1
    fi
    mkdir -p "$(dirname "$target_path")"
    cp -aL "$source_path" "$target_path"
done
for relative in "${SONIC_RUNTIME_TREES[@]}"; do
    source_path="$SONIC_ROOT/$relative"
    target_path="$GEAR_TARGET/$relative"
    if [[ ! -d "$source_path" ]]; then
        echo "[ERROR] Locked SONIC runtime tree is missing: $source_path" >&2
        exit 1
    fi
    mkdir -p "$target_path"
    rsync -aL --delete "$source_path/" "$target_path/"
done

rsync -aL "$INFERENCE_ROOT/" "$STAGING/inference/"
rsync -aL "$VISUAL_ROOT/" "$STAGING/g1-visual/"
if [[ -n "$ROS_PREFIX" ]]; then
    rsync -aL "$ROS_PREFIX/" "$STAGING/ros2-humble-prefix/"
fi
rsync -aL "$NATIVE_DEPS/" "$STAGING/matrix-native-deps/"
if [[ ! -f "$WHEELHOUSE/SHA256SUMS" ]]; then
    echo "[ERROR] Wheelhouse SHA256SUMS is missing: $WHEELHOUSE" >&2
    exit 1
fi
rsync -aL "$WHEELHOUSE/" "$STAGING/python-wheelhouse/"

python3 - "$STAGING" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
pointer_prefix = b"version https://git-lfs.github.com/spec/v1"
pointers = []
for path in root.rglob("*"):
    if path.is_symlink() or not path.is_file():
        continue
    try:
        with path.open("rb") as stream:
            prefix = stream.read(len(pointer_prefix))
    except OSError as exc:
        raise SystemExit(f"cannot inspect staged artifact {path}: {exc}") from exc
    if prefix == pointer_prefix:
        pointers.append(path.relative_to(root).as_posix())
if pointers:
    raise SystemExit("unresolved Git LFS pointers: " + ", ".join(sorted(pointers)))
print("[PASS] Staged artifact contains no unresolved Git LFS pointers")
PY

python3 "$SCRIPT_DIR/verify_matrix_sonic_runtime.py" \
    --lock "$LOCK_FILE" \
    --runtime-root "$STAGING" \
    --sonic-root "$GEAR_TARGET" \
    --matrix-root "$PROJECT_ROOT" \
    --python "$RUNTIME_PYTHON" \
    --skip-dynamic \
    --skip-installed-assets

python3 - "$LOCK_FILE" "$STAGING" <<'PY'
import datetime as dt
import hashlib
import json
import pathlib
import sys

lock_path = pathlib.Path(sys.argv[1])
root = pathlib.Path(sys.argv[2])
lock_bytes = lock_path.read_bytes()
lock = json.loads(lock_bytes)
payload = {
    "schema_version": lock["schema_version"],
    "runtime_id": lock["runtime_id"],
    "release_ready": False,
    "release_blockers": [
        "visual/inference/ROS/native support trees require complete lock attestation",
        "clean-SONIC packager and Git LFS smoke must pass on the release host",
    ],
    "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
    "runtime_lock_sha256": hashlib.sha256(lock_bytes).hexdigest(),
    "source_revisions": lock["source_revisions"],
    "python": {
        key: lock["python"][key]
        for key in ("version", "soabi", "machine", "requirements_sha256")
    },
}
(root / "bundle.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
PY

mkdir -p "$(dirname "$OUTPUT")"
mv "$STAGING" "$OUTPUT"
trap - EXIT
echo "[PASS] Runtime artifact bundle created: $OUTPUT"
