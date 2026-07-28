from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO_ROOT / "scripts"
HELPER_PATH = SCRIPT_DIR / "matrix_bfm_isaac_checkout_snapshot.py"
NAMESPACE_PATH = SCRIPT_DIR / "run_matrix_bfm_isaac_renderer_namespace.sh"
SPEC = importlib.util.spec_from_file_location(
    "matrix_bfm_isaac_checkout_snapshot", HELPER_PATH
)
assert SPEC is not None and SPEC.loader is not None
SNAPSHOT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = SNAPSHOT
SPEC.loader.exec_module(SNAPSHOT)


TRACKED_PATHS = (
    "config/config.json",
    "src/robot_mc/run_mc.sh",
    "src/robot_mujoco/simulate/config.yaml",
)


class MatrixCheckoutSnapshotTest(unittest.TestCase):
    def make_sources(self, root: Path) -> dict[str, tuple[bytes, int]]:
        sources = {
            TRACKED_PATHS[0]: (b'{"robot":"original"}\n\x00', 0o640),
            TRACKED_PATHS[1]: (b"#!/bin/sh\noriginal mc\n", 0o751),
            TRACKED_PATHS[2]: (b'robot: "xgb"\n', 0o604),
        }
        for relative, (data, mode) in sources.items():
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            path.chmod(mode)
        return sources

    def assert_sources(
        self,
        root: Path,
        expected: dict[str, tuple[bytes, int]],
    ) -> None:
        for relative, (data, mode) in expected.items():
            with self.subTest(path=relative):
                path = root / relative
                self.assertFalse(path.is_symlink())
                self.assertEqual(path.read_bytes(), data)
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), mode)

    def test_restore_is_byte_and_mode_exact_and_removes_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            root.mkdir()
            expected = self.make_sources(root)
            snapshot = root / "run" / "snapshot"
            snapshot.parent.mkdir()

            SNAPSHOT.capture(root, snapshot, TRACKED_PATHS)
            for index, relative in enumerate(TRACKED_PATHS):
                path = root / relative
                path.write_bytes(f"mutated-{index}\n".encode())
                path.chmod(0o600 + index)

            SNAPSHOT.restore(root, snapshot, remove_snapshot=True)

            self.assert_sources(root, expected)
            self.assertFalse(snapshot.exists())

    def test_corrupt_snapshot_fails_before_changing_any_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            root.mkdir()
            self.make_sources(root)
            snapshot = root / "run" / "snapshot"
            snapshot.parent.mkdir()
            SNAPSHOT.capture(root, snapshot, TRACKED_PATHS)
            for relative in TRACKED_PATHS:
                (root / relative).write_bytes(b"live mutation\n")
            (snapshot / "0001.bin").write_bytes(b"corrupt\n")

            with self.assertRaisesRegex(
                SNAPSHOT.SnapshotError, "integrity verification"
            ):
                SNAPSHOT.restore(root, snapshot)

            for relative in TRACKED_PATHS:
                self.assertEqual((root / relative).read_bytes(), b"live mutation\n")
            self.assertTrue(snapshot.is_dir())

    def test_capture_rejects_symlink_and_unsafe_source_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            root.mkdir()
            expected = self.make_sources(root)
            real = root / TRACKED_PATHS[0]
            real.unlink()
            target = root / "outside.json"
            target.write_bytes(expected[TRACKED_PATHS[0]][0])
            real.symlink_to(target)

            with self.assertRaisesRegex(SNAPSHOT.SnapshotError, "symlink"):
                SNAPSHOT.capture(
                    root,
                    root / "symlink-snapshot",
                    TRACKED_PATHS,
                )
            with self.assertRaisesRegex(SNAPSHOT.SnapshotError, "unsafe"):
                SNAPSHOT.capture(
                    root,
                    root / "unsafe-snapshot",
                    ("../outside",),
                )
            self.assertFalse((root / "symlink-snapshot").exists())
            self.assertFalse((root / "unsafe-snapshot").exists())

    def test_capture_rejects_a_missing_source_without_leaving_a_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "matrix"
            root.mkdir()
            self.make_sources(root)
            (root / TRACKED_PATHS[2]).unlink()
            snapshot = root / "missing-snapshot"

            with self.assertRaisesRegex(SNAPSHOT.SnapshotError, "unavailable"):
                SNAPSHOT.capture(root, snapshot, TRACKED_PATHS)

            self.assertFalse(snapshot.exists())


class RendererNamespaceSnapshotIntegrationTest(unittest.TestCase):
    def write(self, path: Path, text: str, *, executable: bool = False) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        if executable:
            path.chmod(0o755)

    def make_project(
        self, root: Path
    ) -> tuple[Path, dict[str, tuple[bytes, int]], dict[str, str]]:
        project = root / "matrix"
        scripts = project / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(NAMESPACE_PATH, scripts / NAMESPACE_PATH.name)
        shutil.copy2(HELPER_PATH, scripts / HELPER_PATH.name)

        expected = {
            TRACKED_PATHS[0]: (b'{"robot":"clean"}\n', 0o640),
            TRACKED_PATHS[1]: (b"#!/bin/sh\nclean mc\n", 0o751),
            TRACKED_PATHS[2]: (b'robot: "xgb"\n', 0o604),
        }
        for relative, (data, mode) in expected.items():
            path = project / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            path.chmod(mode)

        self.write(
            scripts / "run_sim.sh",
            """#!/usr/bin/env bash
set -euo pipefail
printf 'mutated config\\n' > config/config.json
chmod 600 config/config.json
printf '#!/bin/sh\\nmutated mc\\n' > src/robot_mc/run_mc.sh
chmod 700 src/robot_mc/run_mc.sh
printf 'robot: \"custom\"\\n' > src/robot_mujoco/simulate/config.yaml
chmod 666 src/robot_mujoco/simulate/config.yaml
printf 'started\\n' > "${FAKE_RUN_SIM_STARTED:?}"
case "${FAKE_RUN_SIM_MODE:?}" in
    normal) sleep 0.25; exit 0 ;;
    failure) sleep 0.25; exit 23 ;;
    startup_failure) sleep 0.05; exit 23 ;;
    corrupt_snapshot)
        printf 'corrupt\\n' > "${MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR:?}/0000.bin"
        sleep 0.25
        exit 0
        ;;
    signal)
        trap 'if grep -q "mutated config" config/config.json; then printf "stopped\\n" > "${FAKE_RUN_SIM_STOPPED:?}"; fi; exit 143' TERM
        while :; do sleep 0.05; done
        ;;
    *) exit 91 ;;
esac
""",
            executable=True,
        )
        self.write(
            scripts / "matrix_external_state_relay.py",
            """#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import socket
import sys
import time

socket_path = Path(sys.argv[sys.argv.index("--unix-socket") + 1])
socket_path.unlink(missing_ok=True)
receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
receiver.bind(str(socket_path))
socket_path.chmod(0o600)
running = True
def stop(_signum, _frame):
    global running
    running = False
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
while running:
    time.sleep(0.02)
receiver.close()
socket_path.unlink(missing_ok=True)
""",
        )
        fake_bin = root / "bin"
        self.write(
            fake_bin / "ss",
            """#!/usr/bin/env bash
if [[ "${FAKE_SS_READY:-1}" == "1" ]]; then
    printf 'UNCONN 0 0 0.0.0.0:9999 0.0.0.0:*\\n'
fi
""",
            executable=True,
        )

        (project / ".gitignore").write_text("/run/\n", encoding="utf-8")
        subprocess.run(("git", "init", "-q", os.fspath(project)), check=True)
        subprocess.run(
            ("git", "-C", os.fspath(project), "config", "user.email", "test@example.com"),
            check=True,
        )
        subprocess.run(
            ("git", "-C", os.fspath(project), "config", "user.name", "Test"),
            check=True,
        )
        subprocess.run(("git", "-C", os.fspath(project), "add", "."), check=True)
        subprocess.run(
            ("git", "-C", os.fspath(project), "commit", "-qm", "fixture"),
            check=True,
        )

        run_dir = project / "run"
        run_dir.mkdir()
        bootstrap = run_dir / "bootstrap.json"
        bootstrap.write_text("{}\n", encoding="utf-8")
        environment = {
            **os.environ,
            "FAKE_RUN_SIM_MODE": "normal",
            "FAKE_RUN_SIM_STARTED": os.fspath(run_dir / "run-sim.started"),
            "FAKE_RUN_SIM_STOPPED": os.fspath(run_dir / "run-sim.stopped"),
            "FAKE_SS_READY": "1",
            "MATRIX_BFM_ISAAC_STATE_SOCKET": os.fspath(run_dir / "state.sock"),
            "MATRIX_BFM_ISAAC_RELAY_STATUS": os.fspath(run_dir / "relay.json"),
            "MATRIX_BFM_ISAAC_RELAY_LOG": os.fspath(run_dir / "relay.log"),
            "MATRIX_BFM_ISAAC_BOOTSTRAP_STATE": os.fspath(bootstrap),
            "MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE": os.fspath(
                run_dir / "namespace.pid"
            ),
            "MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR": os.fspath(
                run_dir / "checkout-snapshot"
            ),
            "PATH": os.fspath(fake_bin)
            + os.pathsep
            + os.environ.get("PATH", "/usr/bin:/bin"),
        }
        return project, expected, environment

    def assert_restored(
        self,
        project: Path,
        expected: dict[str, tuple[bytes, int]],
        environment: dict[str, str],
    ) -> None:
        for relative, (data, mode) in expected.items():
            with self.subTest(path=relative):
                path = project / relative
                self.assertEqual(path.read_bytes(), data)
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), mode)
        self.assertFalse(
            Path(environment["MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR"]).exists()
        )
        status = subprocess.run(
            (
                "git",
                "-C",
                os.fspath(project),
                "status",
                "--porcelain=v1",
                "--untracked-files=all",
            ),
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        self.assertEqual(status, "", status)

    def run_namespace(
        self,
        project: Path,
        environment: dict[str, str],
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ("bash", os.fspath(project / "scripts" / NAMESPACE_PATH.name)),
            cwd=project,
            env=environment,
            text=True,
            capture_output=True,
            timeout=15,
            check=False,
        )

    def test_normal_and_failure_exit_restore_exact_sources(self) -> None:
        for mode, expected_code in (("normal", 0), ("failure", 23)):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as temporary:
                project, expected, environment = self.make_project(Path(temporary))
                environment["FAKE_RUN_SIM_MODE"] = mode

                result = self.run_namespace(project, environment)

                self.assertEqual(result.returncode, expected_code, result.stderr)
                self.assertIn("Restored Matrix source files", result.stdout)
                self.assert_restored(project, expected, environment)

    def test_startup_failure_restores_exact_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project, expected, environment = self.make_project(Path(temporary))
            environment["FAKE_RUN_SIM_MODE"] = "startup_failure"
            environment["FAKE_SS_READY"] = "0"

            result = self.run_namespace(project, environment)

            self.assertEqual(result.returncode, 4, result.stderr)
            self.assertIn("renderer exited before namespace UDP", result.stderr)
            self.assertIn("Restored Matrix source files", result.stdout)
            self.assert_restored(project, expected, environment)

    def test_restore_integrity_failure_is_fail_closed_and_preserves_snapshot(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project, _expected, environment = self.make_project(Path(temporary))
            environment["FAKE_RUN_SIM_MODE"] = "corrupt_snapshot"

            result = self.run_namespace(project, environment)

            self.assertEqual(result.returncode, 70, result.stderr)
            self.assertIn("restoration failed", result.stderr)
            self.assertTrue(
                Path(environment["MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR"]).is_dir()
            )
            self.assertEqual(
                (project / TRACKED_PATHS[0]).read_bytes(), b"mutated config\n"
            )

    def test_int_term_and_hup_restore_after_stopping_run_sim(self) -> None:
        signal_cases = (
            (signal.SIGINT, 130),
            (signal.SIGTERM, 143),
            (signal.SIGHUP, 129),
        )
        for sent_signal, expected_code in signal_cases:
            with self.subTest(
                signal=sent_signal
            ), tempfile.TemporaryDirectory() as temporary:
                project, expected, environment = self.make_project(Path(temporary))
                environment["FAKE_RUN_SIM_MODE"] = "signal"
                process = subprocess.Popen(
                    ("bash", os.fspath(project / "scripts" / NAMESPACE_PATH.name)),
                    cwd=project,
                    env=environment,
                    text=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                started = Path(environment["FAKE_RUN_SIM_STARTED"])
                deadline = time.monotonic() + 5.0
                while time.monotonic() < deadline and not started.is_file():
                    if process.poll() is not None:
                        break
                    time.sleep(0.02)
                self.assertTrue(started.is_file(), "fake run_sim did not start")

                process.send_signal(sent_signal)
                stdout, stderr = process.communicate(timeout=15)

                self.assertEqual(process.returncode, expected_code, stderr)
                self.assertIn("Restored Matrix source files", stdout)
                self.assertTrue(
                    Path(environment["FAKE_RUN_SIM_STOPPED"]).is_file(),
                    "run_sim must observe TERM while source files are still mutated",
                )
                self.assert_restored(project, expected, environment)


class OuterLauncherSnapshotGateIntegrationTest(unittest.TestCase):
    def write(self, path: Path, text: str, *, executable: bool = False) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
        if executable:
            path.chmod(0o755)

    def make_project(
        self, root: Path, *, residual_kind: str
    ) -> tuple[Path, dict[str, str], Path]:
        project = root / "matrix"
        scripts = project / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(SCRIPT_DIR / "run_matrix_bfm_isaac.sh", scripts)
        shutil.copy2(SCRIPT_DIR / "matrix_bfm_isaac_path_guard.py", scripts)
        shutil.copy2(SCRIPT_DIR / "matrix_bfm_isaac_video_settings.py", scripts)
        self.write(
            scripts / "build_matrix_ue_material_fix.sh",
            """#!/usr/bin/env bash
set -euo pipefail
[[ "$*" == *"--expected-sha256"* ]]
[[ "$*" == *"--expected-ue-build-id"* ]]
[[ "$*" == *"--verify-only"* ]]
""",
            executable=True,
        )
        video_settings = (
            project / "config/runtime/matrix-bfm-isaac-video-settings.json"
        )
        video_settings.parent.mkdir(parents=True)
        shutil.copy2(
            REPO_ROOT / "config/runtime/matrix-bfm-isaac-video-settings.json",
            video_settings,
        )
        self.write(
            scripts / "matrix_local_env.sh",
            """#!/usr/bin/env bash
load_matrix_local_env() { return 0; }
""",
        )
        self.write(
            scripts / "matrix_bfm_isaac_instance_ledger.py",
            """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import signal
import sys

args = sys.argv[1:]
path = Path(args[args.index("--path") + 1])
actions = ("init", "add", "signal", "signal-pid", "verify-empty")
action = next(item for item in actions if item in args)
if action == "init":
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({"fixture": True}), encoding="utf-8")
elif action == "signal-pid":
    pid = int(args[args.index("--pid") + 1])
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
print(json.dumps({"ok": True, "action": action}))
""",
        )
        self.write(
            scripts / "verify_matrix_bfm_isaac_runtime.py",
            """#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

args = sys.argv[1:]
if "--output" in args:
    output = Path(args[args.index("--output") + 1])
    output.write_text(json.dumps({"overall_ok": True}) + "\\n", encoding="utf-8")
    Path(os.environ["FAKE_ACCEPTANCE_MARKER"]).write_text("called\\n", encoding="utf-8")
print(json.dumps({"overall_ok": True}))
""",
        )
        self.write(
            scripts / "run_matrix_bfm_isaac_renderer_isolated.sh",
            """#!/usr/bin/env bash
set -euo pipefail
snapshot="${MATRIX_BFM_ISAAC_CHECKOUT_SNAPSHOT_DIR:?}"
case "${FAKE_RESIDUAL_KIND:?}" in
    corrupt-directory)
        mkdir "$snapshot"
        printf 'corrupt\\n' > "$snapshot/0000.bin"
        ;;
    dangling-symlink)
        ln -s "$snapshot.missing" "$snapshot"
        ;;
    clean-exit70|clean-term143)
        ;;
    *) exit 92 ;;
esac
printf '%s\\n' "$$" > "${MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE:?}"
printf '{}\\n' > "${MATRIX_BFM_ISAAC_RELAY_STATUS:?}"
python3 - "${MATRIX_BFM_ISAAC_STATE_SOCKET:?}" <<'PY' &
from pathlib import Path
import signal
import socket
import sys
import time
path = Path(sys.argv[1])
path.unlink(missing_ok=True)
receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
receiver.bind(str(path))
running = True
def stop(_signum, _frame):
    global running
    running = False
signal.signal(signal.SIGTERM, stop)
while running:
    time.sleep(0.02)
receiver.close()
path.unlink(missing_ok=True)
PY
socket_pid=$!
cleanup() {
    trap - TERM INT HUP
    if [[ -f "${FAKE_PHYSICS_DONE:?}" ]]; then
        printf 'physics-first\\n' > "${FAKE_RENDERER_STOP_AFTER_PHYSICS:?}"
    fi
    kill -TERM "$socket_pid" 2>/dev/null || true
    wait "$socket_pid" 2>/dev/null || true
    rm -f -- "${MATRIX_BFM_ISAAC_RENDERER_NAMESPACE_PID_FILE:?}"
    exit "${FAKE_RENDERER_EXIT_CODE:?}"
}
trap cleanup TERM INT HUP
wait "$socket_pid"
""",
            executable=True,
        )

        runtime = root / "runtime"
        runtime_config = runtime / "configs/alienware/moon-matrix.toml"
        runtime_runner = runtime / "scripts/run_g1_teacher_closed_loop.py"
        physics = root / "physics"
        collision = root / "collision"
        source = root / "source"
        visual = root / "visual"
        visual_venv = root / "visual-venv"
        teacher = root / "teacher-profile.toml"
        for directory in (physics, collision, source, visual, visual_venv):
            directory.mkdir(parents=True)
        (physics / "main.usd").write_text("usd\n", encoding="utf-8")
        (collision / "collision.usda").write_text("usd\n", encoding="utf-8")
        (visual / "g1_29dof.urdf").write_text("<robot/>\n", encoding="utf-8")
        teacher.write_text("profile = true\n", encoding="utf-8")
        self.write(runtime_runner, "# fixture\n")
        self.write(
            runtime_config,
            "\n".join(
                (
                    "[paths]",
                    f'bfm_sonic_repo = "{source}"',
                    f'g1_usd = "{physics / "main.usd"}"',
                    f'scene_root = "{collision}"',
                    f'collision_usd = "{collision / "collision.usda"}"',
                    "",
                )
            ),
        )
        subprocess.run(("git", "init", "-q", os.fspath(runtime)), check=True)
        subprocess.run(
            ("git", "-C", os.fspath(runtime), "config", "user.email", "test@example.com"),
            check=True,
        )
        subprocess.run(
            ("git", "-C", os.fspath(runtime), "config", "user.name", "Test"),
            check=True,
        )
        subprocess.run(("git", "-C", os.fspath(runtime), "add", "."), check=True)
        subprocess.run(
            ("git", "-C", os.fspath(runtime), "commit", "-qm", "fixture"),
            check=True,
        )

        runtime_python = root / "runtime-python"
        self.write(
            runtime_python,
            """#!/usr/bin/env python3
import os
from pathlib import Path
import sys

args = sys.argv[1:]
if args[:2] == ["-I", "-"]:
    os.execv(sys.executable, [sys.executable, *args])
if args and args[0] == "-m":
    report = Path(args[args.index("--report") + 1])
    report.write_text("{}\\n", encoding="utf-8")
    raise SystemExit(0)
report = Path(args[args.index("--report") + 1])
trajectory = Path(args[args.index("--trajectory") + 1])
report.write_text("{}\\n", encoding="utf-8")
trajectory.write_bytes(b"trajectory")
Path(os.environ["FAKE_PHYSICS_DONE"]).write_text("done\\n", encoding="utf-8")
""",
            executable=True,
        )

        lock = {
            "ue_material_bridge": {
                "relative_path": (
                    "outputs/runtime/matrix-ue-material-fix/"
                    "libmatrix_ue_material_fix.so"
                ),
                "sha256": "9f64dd949bd44be61a11dcbbe3e5a49f6ef6f6f318c4771a24385e9781840b96",
                "ue_binary_relative_path": (
                    "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/"
                    "zsibot_mujoco_ue"
                ),
                "ue_binary_build_id": "056e17b8675b1006",
            },
            "scene_collision_contract": {
                "scene_id": 15,
                "runtime_config_suffix": "configs/alienware/moon-matrix.toml",
                "x_min_m": -97.0,
                "x_max_m": 143.0,
                "y_min_m": -107.0,
                "y_max_m": 133.0,
                "warning_margin_m": 20.0,
                "stop_margin_m": 10.0,
            },
            "physics_assets": {"main_usd": "main.usd"},
            "scene_assets": {"collision_usd": "collision.usda"},
        }
        lock_path = project / "config/runtime/matrix-bfm-isaac.lock.json"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        lock_path.write_text(json.dumps(lock), encoding="utf-8")

        evidence = root / "evidence"
        state = root / "state"
        acceptance_marker = root / "acceptance.called"
        environment = {
            **os.environ,
            "FAKE_ACCEPTANCE_MARKER": os.fspath(acceptance_marker),
            "FAKE_PHYSICS_DONE": os.fspath(root / "physics.done"),
            "FAKE_RENDERER_STOP_AFTER_PHYSICS": os.fspath(
                root / "renderer-stopped-after-physics"
            ),
            "FAKE_RENDERER_EXIT_CODE": (
                "143" if residual_kind == "clean-term143" else "70"
            ),
            "FAKE_RESIDUAL_KIND": residual_kind,
            "MATRIX_BFM_CLEAN_RUN_NONCE": "a" * 32,
            "MATRIX_BFM_ISAAC_CACHE_ROOT": os.fspath(root / "cache"),
            "MATRIX_BFM_ISAAC_COLLISION_ROOT": os.fspath(collision),
            "MATRIX_BFM_ISAAC_CONFIG": os.fspath(runtime_config),
            "MATRIX_BFM_ISAAC_CONFIG_ROOT": os.fspath(root / "config-home"),
            "MATRIX_BFM_ISAAC_GUARDED": "1",
            "MATRIX_BFM_ISAAC_IPC_ROOT": os.fspath(state / "ipc"),
            "MATRIX_BFM_ISAAC_PHYSICS_ASSET_ROOT": os.fspath(physics),
            "MATRIX_BFM_ISAAC_PYTHON": os.fspath(runtime_python),
            "MATRIX_BFM_ISAAC_RUN_ROOT": os.fspath(root / "runs"),
            "MATRIX_BFM_ISAAC_RUNTIME_ROOT": os.fspath(runtime),
            "MATRIX_BFM_ISAAC_SOURCE_ROOT": os.fspath(source),
            "MATRIX_BFM_ISAAC_STATE_ROOT": os.fspath(state),
            "MATRIX_BFM_ISAAC_TEACHER_PROFILE": os.fspath(teacher),
            "MATRIX_BFM_ISAAC_VISUAL_ROOT": os.fspath(visual),
            "MATRIX_BFM_ISAAC_VISUAL_URDF": os.fspath(visual / "g1_29dof.urdf"),
            "MATRIX_BFM_ISAAC_VISUAL_VENV": os.fspath(visual_venv),
            "MATRIX_INSTANCE_ID": f"outer-{residual_kind}",
        }
        environment.pop("MATRIX_UE_EXTRA_EXEC_CMDS", None)
        environment.pop("MATRIX_UE_MATERIAL_FIX_PRELOAD", None)
        return project, environment, evidence

    def test_python310_paths_fallback_rejects_decoy_table(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project, environment, evidence = self.make_project(
                root, residual_kind="clean-term143"
            )
            runtime_config = Path(environment["MATRIX_BFM_ISAAC_CONFIG"])
            expected = {
                "bfm_sonic_repo": Path(environment["MATRIX_BFM_ISAAC_SOURCE_ROOT"]),
                "g1_usd": (
                    Path(environment["MATRIX_BFM_ISAAC_PHYSICS_ASSET_ROOT"])
                    / "main.usd"
                ),
                "scene_root": Path(environment["MATRIX_BFM_ISAAC_COLLISION_ROOT"]),
                "collision_usd": (
                    Path(environment["MATRIX_BFM_ISAAC_COLLISION_ROOT"])
                    / "collision.usda"
                ),
            }
            lines = ["[paths]"]
            for key in expected:
                lines.append(f'"{key}" = "/unverified/{key}"')
            # This is valid TOML.  A fallback parser must leave [paths] at the
            # commented table header instead of accepting decoy values below.
            lines.extend(("", '[decoy] # = "still a table header"'))
            for key, value in expected.items():
                lines.append(f"{key} = {json.dumps(os.fspath(value))}")
            runtime_config.write_text("\n".join((*lines, "")), encoding="utf-8")

            result = subprocess.run(
                (
                    "bash",
                    os.fspath(project / "scripts/run_matrix_bfm_isaac.sh"),
                    "smoke",
                    "--run-dir",
                    os.fspath(evidence),
                ),
                cwd=project,
                env=environment,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )

            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertFalse(Path(environment["FAKE_ACCEPTANCE_MARKER"]).exists())

    def test_qualified_launcher_rejects_generic_video_and_material_overrides(
        self,
    ) -> None:
        for variable, value, expected_error in (
            (
                "MATRIX_UE_EXTRA_EXEC_CMDS",
                "t.MaxFPS 1,r.ScreenPercentage 1",
                "rejects MATRIX_UE_EXTRA_EXEC_CMDS",
            ),
            (
                "MATRIX_UE_MATERIAL_FIX_PRELOAD",
                "off",
                "rejects material bridge overrides",
            ),
            (
                "MATRIX_UE_MATERIAL_FIX_PRELOAD",
                "/tmp/unreviewed.so",
                "rejects material bridge overrides",
            ),
        ):
            with self.subTest(variable=variable, value=value), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                project, environment, evidence = self.make_project(
                    root, residual_kind="clean-exit70"
                )
                environment[variable] = value

                result = subprocess.run(
                    (
                        "bash",
                        os.fspath(project / "scripts/run_matrix_bfm_isaac.sh"),
                        "smoke",
                        "--run-dir",
                        os.fspath(evidence),
                    ),
                    cwd=project,
                    env=environment,
                    text=True,
                    capture_output=True,
                    timeout=10,
                    check=False,
                )

                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn(expected_error, result.stderr)
                self.assertFalse(evidence.exists())
                self.assertFalse(Path(environment["FAKE_PHYSICS_DONE"]).exists())

    def test_physics_first_residual_or_symlink_snapshot_forces_outer_exit70(
        self,
    ) -> None:
        for residual_kind in (
            "corrupt-directory",
            "dangling-symlink",
            "clean-exit70",
        ):
            with self.subTest(
                residual_kind=residual_kind
            ), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                project, environment, evidence = self.make_project(
                    root, residual_kind=residual_kind
                )

                result = subprocess.run(
                    (
                        "bash",
                        os.fspath(project / "scripts/run_matrix_bfm_isaac.sh"),
                        "smoke",
                        "--run-dir",
                        os.fspath(evidence),
                    ),
                    cwd=project,
                    env=environment,
                    text=True,
                    capture_output=True,
                    timeout=30,
                    check=False,
                )

                self.assertEqual(result.returncode, 70, result.stderr)
                if residual_kind == "clean-exit70":
                    self.assertIn(
                        "namespace reported Matrix source restoration failure",
                        result.stderr,
                    )
                else:
                    self.assertIn("unretired Matrix source snapshot", result.stderr)
                self.assertTrue(Path(environment["FAKE_ACCEPTANCE_MARKER"]).is_file())
                self.assertTrue(Path(environment["FAKE_PHYSICS_DONE"]).is_file())
                self.assertTrue(
                    Path(
                        environment["FAKE_RENDERER_STOP_AFTER_PHYSICS"]
                    ).is_file()
                )
                snapshot = evidence / "checkout-source-snapshot"
                if residual_kind == "clean-exit70":
                    self.assertFalse(snapshot.exists() or snapshot.is_symlink())
                else:
                    self.assertTrue(snapshot.exists() or snapshot.is_symlink())

    def test_physics_first_clean_renderer_term143_remains_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project, environment, evidence = self.make_project(
                root, residual_kind="clean-term143"
            )

            result = subprocess.run(
                (
                    "bash",
                    os.fspath(project / "scripts/run_matrix_bfm_isaac.sh"),
                    "smoke",
                    "--run-dir",
                    os.fspath(evidence),
                ),
                cwd=project,
                env=environment,
                text=True,
                capture_output=True,
                timeout=30,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(Path(environment["FAKE_ACCEPTANCE_MARKER"]).is_file())
            self.assertTrue(
                Path(environment["FAKE_RENDERER_STOP_AFTER_PHYSICS"]).is_file()
            )
            snapshot = evidence / "checkout-source-snapshot"
            self.assertFalse(snapshot.exists() or snapshot.is_symlink())

if __name__ == "__main__":
    unittest.main()
