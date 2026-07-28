from __future__ import annotations

import os
from pathlib import Path
import fcntl
import shutil
import stat
import subprocess
import tempfile
import time
import unittest
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts" / "launch_matrix_bfm_isaac_desktop.sh"
INSTALLER = (
    REPO_ROOT / "scripts" / "install_matrix_bfm_isaac_desktop_launcher.sh"
)
TEMPLATE = REPO_ROOT / "packaging" / "matrix-bfm-isaac-mainline.desktop.in"


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def parse_call_log(path: Path) -> list[list[str]]:
    if not path.exists():
        return []
    return [line.split("\t") for line in path.read_text().splitlines()]


class MatrixBfmIsaacDesktopLauncherTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.project = self.root / "qualified matrix"
        scripts = self.project / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(LAUNCHER, scripts / LAUNCHER.name)
        self.launcher = scripts / LAUNCHER.name
        self.run_log = self.root / "run.log"
        write_executable(
            scripts / "run_matrix_bfm_isaac_guarded.sh",
            """#!/usr/bin/env bash
set -euo pipefail
run_dir=""
arguments=("$@")
for ((i=0; i<${#arguments[@]}; i++)); do
    if [[ "${arguments[$i]}" == "--run-dir" ]]; then
        run_dir="${arguments[$((i + 1))]}"
    fi
done
[[ -n "$run_dir" ]]
mkdir -p -- "$run_dir"
if [[ "${FAKE_FINALIZER_INVALID:-0}" == "1" ]]; then
    printf '{"complete":false}\n' > "$run_dir/finalizer-status.json"
else
    printf '%s\n' '{"complete":true,"trigger":"signal_term","physics_exit_code":0,"stack_failure_code":143,"report_present":true,"trajectory_present":true,"relay_status_present":true}' > "$run_dir/finalizer-status.json"
fi
sleep "${FAKE_RUN_DELAY:-0}"
{
    printf 'ARGV'
    for value in "$@"; do printf '\t%s' "$value"; done
    printf '\nENV\t%s\t%s\t%s\n' \
        "${MATRIX_INSTANCE_ID:-}" \
        "${MATRIX_UE_EXTRA_EXEC_CMDS-unset}" \
        "${MATRIX_BFM_ISAAC_VIDEO_QUALITY-unset}"
} >> "${FAKE_RUN_LOG:?}"
""",
        )
        self.fake_bin = self.root / "fake-bin"
        self.fake_bin.mkdir()
        self.tmux_state = self.root / "tmux.state"
        self.tmux_log = self.root / "tmux.log"
        write_executable(
            self.fake_bin / "tmux",
            """#!/usr/bin/env bash
set -euo pipefail
{
    printf '%s' "${1:-}"
    for value in "${@:2}"; do printf '\t%s' "$value"; done
    printf '\n'
} >> "${FAKE_TMUX_LOG:?}"
command_name="${1:-}"
shift || true
case "$command_name" in
    has-session) [[ -f "${FAKE_TMUX_STATE:?}" ]] ;;
    list-panes)
        if [[ "$*" == *pane_dead_status* ]]; then
            printf '%s\n' "${FAKE_TMUX_DEAD_STATUS:-0}"
        else
            cat "${FAKE_TMUX_STATE:?}"
        fi
        ;;
    new-session)
        while (($#)); do
            if [[ "$1" == "--" ]]; then shift; break; fi
            shift
        done
        "$@"
        printf '0\n' > "$FAKE_TMUX_STATE"
        ;;
    set-window-option) : ;;
    set-option)
        option="${@: -2:1}"
        value="${@: -1}"
        case "$option" in
            @matrix_run_dir) printf '%s\n' "$value" > "$FAKE_TMUX_STATE.run_dir" ;;
            @matrix_console_log) printf '%s\n' "$value" > "$FAKE_TMUX_STATE.console" ;;
        esac
        ;;
    show-options)
        option="${@: -1}"
        case "$option" in
            @matrix_run_dir) cat "$FAKE_TMUX_STATE.run_dir" ;;
            @matrix_console_log) cat "$FAKE_TMUX_STATE.console" ;;
            *) exit 1 ;;
        esac
        ;;
    send-keys) printf '1\n' > "$FAKE_TMUX_STATE" ;;
    kill-session)
        rm -f -- "$FAKE_TMUX_STATE" "$FAKE_TMUX_STATE.run_dir" \
            "$FAKE_TMUX_STATE.console"
        ;;
    attach-session) printf 'attached\n' ;;
    *) exit 64 ;;
esac
""",
        )
        self.environment = os.environ.copy()
        self.environment.update(
            {
                "FAKE_RUN_LOG": os.fspath(self.run_log),
                "FAKE_TMUX_LOG": os.fspath(self.tmux_log),
                "FAKE_TMUX_STATE": os.fspath(self.tmux_state),
                "MATRIX_SONIC_HOST_LOCK": os.fspath(self.root / "host.lock"),
                "MATRIX_BFM_ISAAC_DESKTOP_SESSION_NAME": "test-mainline",
                "MATRIX_BFM_ISAAC_DESKTOP_INSTANCE_ID": "test-mainline",
                "MATRIX_BFM_ISAAC_DESKTOP_STATE_DIR": os.fspath(
                    self.root / "launcher-state"
                ),
                "MATRIX_BFM_ISAAC_DESKTOP_LOG_DIR": os.fspath(
                    self.root / "launcher-log"
                ),
                "MATRIX_UE_EXTRA_EXEC_CMDS": "unsafe",
                "MATRIX_BFM_ISAAC_VIDEO_QUALITY": "epic",
                "PATH": os.fspath(self.fake_bin)
                + os.pathsep
                + self.environment.get("PATH", "/usr/bin:/bin"),
            }
        )
        for variable in ("DISPLAY", "WAYLAND_DISPLAY", "TMUX"):
            self.environment.pop(variable, None)

    def run_launcher(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["/bin/bash", os.fspath(self.launcher), *args],
            env=self.environment,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )

    def test_start_uses_qualified_guarded_interactive_contract(self) -> None:
        first = self.run_launcher("start", "--profile", "trna")
        second = self.run_launcher("start")

        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(second.returncode, 0, second.stderr)
        calls = parse_call_log(self.run_log)
        self.assertEqual(calls[0][:8], [
            "ARGV",
            "interactive",
            "--profile",
            "trna",
            "--onscreen",
            "--duration",
            "7200",
            "--correctness-only",
        ])
        self.assertEqual(calls[0][8], "--run-dir")
        self.assertIn("desktop_", calls[0][9])
        self.assertEqual(calls[1], ["ENV", "test-mainline", "unset", "unset"])
        tmux_calls = parse_call_log(self.tmux_log)
        self.assertEqual(
            sum(call[0] == "new-session" for call in tmux_calls), 1
        )
        self.assertIn("qualified Matrix BFM/Isaac", first.stdout)
        self.assertIn("already running", second.stdout)
        log = self.root / "launcher-log" / "mainline-desktop-launcher.log"
        self.assertIn("START ok", log.read_text(encoding="utf-8"))

    def test_status_and_graceful_stop_remove_session(self) -> None:
        started = self.run_launcher("start")
        status = self.run_launcher("status")
        stopped = self.run_launcher("stop")
        status_after = self.run_launcher("status")

        self.assertEqual(started.returncode, 0, started.stderr)
        self.assertEqual(status.returncode, 0, status.stderr)
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertEqual(status_after.returncode, 1)
        self.assertFalse(self.tmux_state.exists())
        self.assertIn("finalizer verified", stopped.stdout)

    def test_failed_finalizer_is_reported_and_session_is_retained(self) -> None:
        self.environment["FAKE_FINALIZER_INVALID"] = "1"
        started = self.run_launcher("start")
        stopped = self.run_launcher("stop")

        self.assertEqual(started.returncode, 0, started.stderr)
        self.assertEqual(stopped.returncode, 1)
        self.assertIn("finalizer did not pass", stopped.stderr)
        self.assertTrue(self.tmux_state.exists())
        retry = self.run_launcher("start")
        self.assertEqual(retry.returncode, 1)
        self.assertIn("retained", retry.stderr)
        self.assertTrue(self.tmux_state.exists())
        dismissed = self.run_launcher("dismiss")
        self.assertEqual(dismissed.returncode, 0, dismissed.stderr)
        self.assertFalse(self.tmux_state.exists())

    def test_host_lock_blocks_start_before_tmux_creation(self) -> None:
        lock = Path(self.environment["MATRIX_SONIC_HOST_LOCK"])
        with lock.open("w", encoding="utf-8") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_launcher("start")

        self.assertEqual(result.returncode, 1)
        self.assertIn("Another Matrix launcher owns this host", result.stderr)
        self.assertFalse(
            any(call[0] == "new-session" for call in parse_call_log(self.tmux_log))
        )

    def test_session_lock_serializes_concurrent_start_and_stop(self) -> None:
        environment = self.environment.copy()
        environment["FAKE_RUN_DELAY"] = "0.4"
        start = subprocess.Popen(
            ["/bin/bash", os.fspath(self.launcher), "start"],
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(0.05)
        stop = subprocess.Popen(
            ["/bin/bash", os.fspath(self.launcher), "stop"],
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        start_stdout, start_stderr = start.communicate(timeout=10.0)
        stop_stdout, stop_stderr = stop.communicate(timeout=10.0)

        self.assertEqual(start.returncode, 0, start_stderr)
        self.assertEqual(stop.returncode, 0, stop_stderr)
        self.assertIn("Started qualified", start_stdout)
        self.assertIn("finalizer verified", stop_stdout)
        self.assertFalse(self.tmux_state.exists())

    def test_rejects_argument_injection_before_tmux(self) -> None:
        profile = self.run_launcher("start", "--profile", "trna;touch")
        duration = self.run_launcher("start", "--duration", "20;touch")
        unknown = self.run_launcher("start", "--unknown")

        self.assertEqual(profile.returncode, 2)
        self.assertEqual(duration.returncode, 2)
        self.assertEqual(unknown.returncode, 2)
        self.assertFalse(self.tmux_log.exists())


class MatrixBfmIsaacDesktopInstallerTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.project = self.root / "installer repo"
        (self.project / "scripts").mkdir(parents=True)
        (self.project / "packaging").mkdir()
        shutil.copy2(INSTALLER, self.project / "scripts" / INSTALLER.name)
        shutil.copy2(TEMPLATE, self.project / "packaging" / TEMPLATE.name)
        self.active_release = self.root / "release 7926c11"
        (self.active_release / "scripts").mkdir(parents=True)
        (self.active_release / "demo_gif").mkdir()
        shutil.copy2(
            LAUNCHER,
            self.active_release / "scripts" / LAUNCHER.name,
        )
        (self.active_release / "demo_gif" / "Launcher.png").write_bytes(
            b"icon\n"
        )
        self.active = self.root / "matrix-mainline"
        self.active.symlink_to(self.active_release, target_is_directory=True)
        self.desktop = self.root / "Desktop"
        self.desktop.mkdir(mode=0o700)

    def run_installer(self, *args: str) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["HOME"] = os.fspath(self.root / "home")
        return subprocess.run(
            [
                "/bin/bash",
                os.fspath(self.project / "scripts" / INSTALLER.name),
                *args,
            ],
            env=environment,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )

    def test_installs_stable_active_release_entry_atomically(self) -> None:
        target = self.desktop / "matrix-sonic.desktop"
        target.write_text("old launcher\n", encoding="utf-8")
        result = self.run_installer(
            "--active-root",
            os.fspath(self.active),
            "--desktop-dir",
            os.fspath(self.desktop),
            "--profile",
            "trna",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        content = target.read_text(encoding="utf-8")
        launcher = self.active / "scripts" / LAUNCHER.name
        self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o755)
        self.assertIn(f'Exec=/usr/bin/bash "{launcher}" start --profile trna', content)
        self.assertIn(f'Exec=/usr/bin/bash "{launcher}" status --profile trna', content)
        self.assertIn(f'Exec=/usr/bin/bash "{launcher}" stop --profile trna', content)
        self.assertIn(f"X-Matrix-Active-Root={self.active}", content)
        self.assertIn("X-Matrix-Release-Channel=qualified-mainline", content)
        self.assertNotIn("@MATRIX_", content)
        self.assertNotIn("old launcher", content)
        self.assertEqual(list(self.desktop.glob(".*.tmp.*")), [])

    def test_stable_active_symlink_can_be_retargeted_without_reinstall(self) -> None:
        release_two = self.root / "release next"
        (release_two / "scripts").mkdir(parents=True)
        (release_two / "demo_gif").mkdir()
        shutil.copy2(LAUNCHER, release_two / "scripts" / LAUNCHER.name)
        (release_two / "demo_gif" / "Launcher.png").write_bytes(b"next\n")
        installed = self.run_installer(
            "--active-root",
            os.fspath(self.active),
            "--desktop-dir",
            os.fspath(self.desktop),
        )
        self.assertEqual(installed.returncode, 0, installed.stderr)

        self.active.unlink()
        self.active.symlink_to(release_two, target_is_directory=True)

        launcher = self.active / "scripts" / LAUNCHER.name
        self.assertEqual(launcher.resolve(), (release_two / "scripts" / LAUNCHER.name).resolve())
        content = (self.desktop / "matrix-sonic.desktop").read_text(
            encoding="utf-8"
        )
        self.assertIn(os.fspath(launcher), content)

    def test_rejects_missing_active_root_and_symlink_target(self) -> None:
        missing = self.run_installer(
            "--active-root",
            os.fspath(self.root / "missing"),
            "--desktop-dir",
            os.fspath(self.desktop),
        )
        victim = self.root / "victim"
        victim.write_text("keep\n", encoding="utf-8")
        target = self.desktop / "matrix-sonic.desktop"
        target.symlink_to(victim)
        linked = self.run_installer(
            "--active-root",
            os.fspath(self.active),
            "--desktop-dir",
            os.fspath(self.desktop),
        )

        self.assertEqual(missing.returncode, 2)
        self.assertEqual(linked.returncode, 2)
        self.assertEqual(victim.read_text(encoding="utf-8"), "keep\n")

    def test_sources_do_not_embed_machine_paths(self) -> None:
        for path in (LAUNCHER, INSTALLER, TEMPLATE):
            with self.subTest(path=path.name):
                self.assertNotIn("/home/", path.read_text(encoding="utf-8"))


@unittest.skipUnless(shutil.which("tmux"), "tmux is required for signal regression")
class MatrixBfmIsaacDesktopSignalTest(unittest.TestCase):
    def test_ignore_interrupts_tee_preserves_detached_child_finalizer(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        finalizer = root / "finalizer.txt"
        console = root / "console.log"
        child = root / "child.py"
        guard = root / "guard.py"
        child.write_text(
            """import signal
import sys
import time
from pathlib import Path

finalizer = Path(sys.argv[1])

def stop(_signum, _frame):
    print("CHILD_TERM", flush=True)
    finalizer.write_text("finalized\\n", encoding="utf-8")
    raise SystemExit(0)

signal.signal(signal.SIGTERM, stop)
print("CHILD_READY", flush=True)
while True:
    time.sleep(0.05)
""",
            encoding="utf-8",
        )
        guard.write_text(
            """import os
import signal
import subprocess
import sys
import time

child = subprocess.Popen(
    [sys.executable, sys.argv[2], sys.argv[1]],
    start_new_session=True,
)

def stop(_signum, _frame):
    os.killpg(child.pid, signal.SIGTERM)
    child.wait(timeout=5.0)
    print("GUARD_FINALIZED", flush=True)
    raise SystemExit(130)

signal.signal(signal.SIGINT, stop)
print("GUARD_READY", flush=True)
while child.poll() is None:
    time.sleep(0.05)
""",
            encoding="utf-8",
        )
        socket_name = f"matrix-desktop-{os.getpid()}-{time.time_ns()}"

        def tmux(*arguments: str) -> subprocess.CompletedProcess[str]:
            return subprocess.run(
                ["tmux", "-L", socket_name, *arguments],
                text=True,
                capture_output=True,
                timeout=10.0,
                check=False,
            )

        self.addCleanup(lambda: tmux("kill-server"))
        pipeline = (
            'log="$1"; shift; "$@" 2>&1 | '
            '/usr/bin/tee --ignore-interrupts -a -- "$log"'
        )
        started = tmux(
            "new-session",
            "-d",
            "-s",
            "signal-test",
            "/usr/bin/bash",
            "-o",
            "pipefail",
            "-c",
            pipeline,
            "matrix-mainline",
            os.fspath(console),
            sys.executable,
            os.fspath(guard),
            os.fspath(finalizer),
            os.fspath(child),
        )
        self.assertEqual(started.returncode, 0, started.stderr)
        retained = tmux(
            "set-window-option",
            "-t",
            "=signal-test:",
            "remain-on-exit",
            "on",
        )
        self.assertEqual(retained.returncode, 0, retained.stderr)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            content = console.read_text(encoding="utf-8") if console.exists() else ""
            if "GUARD_READY" in content and "CHILD_READY" in content:
                break
            time.sleep(0.05)
        else:
            self.fail("tmux pipeline did not become ready")

        interrupted = tmux("send-keys", "-t", "=signal-test:0.0", "C-c")
        self.assertEqual(interrupted.returncode, 0, interrupted.stderr)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not finalizer.exists():
            time.sleep(0.05)

        self.assertEqual(finalizer.read_text(encoding="utf-8"), "finalized\n")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            content = console.read_text(encoding="utf-8")
            if "GUARD_FINALIZED" in content:
                break
            time.sleep(0.05)
        self.assertIn("CHILD_TERM", content)
        self.assertIn("GUARD_FINALIZED", content)


if __name__ == "__main__":
    unittest.main()
