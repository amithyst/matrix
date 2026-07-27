from __future__ import annotations

import importlib.util
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


if __name__ == "__main__":
    unittest.main()
