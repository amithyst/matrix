from __future__ import annotations

import fcntl
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts" / "launch_matrix_sonic_desktop.sh"
INSTALLER = REPO_ROOT / "scripts" / "install_matrix_desktop_launcher.sh"
TEMPLATE = REPO_ROOT / "packaging" / "matrix-sonic.desktop.in"


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def parse_call_log(path: Path) -> list[list[str]]:
    if not path.exists():
        return []
    calls: list[list[str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split("\t")
        if not fields or fields[0] != "CALL":
            raise AssertionError(f"invalid fake command log line: {line!r}")
        calls.append(fields[1:])
    return calls


class MatrixDesktopLauncherTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.project = self.root / "matrix fixture"
        scripts = self.project / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(LAUNCHER, scripts / LAUNCHER.name)

        self.run_log = self.root / "run.log"
        write_executable(
            scripts / "run_matrix_sonic_moon_v1.sh",
            """#!/usr/bin/env bash
set -euo pipefail
{
    printf 'CALL'
    for argument in "$@"; do
        printf '\\t%s' "$argument"
    done
    printf '\\n'
} >> "${FAKE_RUN_LOG:?}"
""",
        )
        write_executable(
            scripts / "run_matrix_sonic.sh",
            """#!/usr/bin/env bash
set -euo pipefail
{
    printf 'CALL'
    for argument in "$@"; do
        printf '\\t%s' "$argument"
    done
    printf '\\n'
} >> "${FAKE_RUN_LOG:?}"
""",
        )

        self.fake_bin = self.root / "fake-bin"
        self.fake_bin.mkdir()
        self.tmux_log = self.root / "tmux.log"
        self.tmux_state = self.root / "tmux.state"
        self.runtime_dir = self.root / "runtime"
        self.runtime_dir.mkdir(mode=0o700)
        self.host_lock = self.root / "matrix-sonic-host.lock"
        write_executable(
            self.fake_bin / "tmux",
            """#!/usr/bin/env bash
set -euo pipefail
{
    printf 'CALL'
    for argument in "$@"; do
        printf '\\t%s' "$argument"
    done
    printf '\\n'
} >> "${FAKE_TMUX_LOG:?}"

command_name="${1:-}"
if (($# > 0)); then
    shift
fi
case "$command_name" in
    has-session)
        [[ -f "${FAKE_TMUX_STATE:?}" ]]
        ;;
    list-panes)
        cat "${FAKE_TMUX_STATE:?}"
        ;;
    new-session)
        while (($# > 0)); do
            if [[ "$1" == "--" ]]; then
                shift
                break
            fi
            shift
        done
        (($# > 0)) || exit 64
        "$@"
        printf '%s\n' "${FAKE_TMUX_NEW_PANE_DEAD:-0}" > "$FAKE_TMUX_STATE"
        ;;
    kill-session)
        rm -f -- "$FAKE_TMUX_STATE"
        ;;
    send-keys)
        rm -f -- "$FAKE_TMUX_STATE"
        ;;
    attach-session)
        printf 'fake attach succeeded\\n'
        ;;
    *)
        printf 'unexpected fake tmux command: %s\\n' "$command_name" >&2
        exit 64
        ;;
esac
""",
        )

        self.launcher = scripts / LAUNCHER.name
        self.environment = os.environ.copy()
        self.environment.update(
            {
                "FAKE_RUN_LOG": os.fspath(self.run_log),
                "FAKE_TMUX_LOG": os.fspath(self.tmux_log),
                "FAKE_TMUX_STATE": os.fspath(self.tmux_state),
                "HOME": os.fspath(self.root / "home"),
                "MATRIX_DESKTOP_HOST_LOCK_PATH": os.fspath(self.host_lock),
                "XDG_RUNTIME_DIR": os.fspath(self.runtime_dir),
                "PATH": os.fspath(self.fake_bin)
                + os.pathsep
                + self.environment.get("PATH", "/usr/bin:/bin"),
            }
        )
        for variable in ("DISPLAY", "WAYLAND_DISPLAY", "TMUX"):
            self.environment.pop(variable, None)

    def run_launcher(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["/bin/bash", os.fspath(self.launcher), *arguments],
            env=self.environment,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )

    def test_start_passes_exact_runtime_argv_and_is_idempotent(self) -> None:
        first = self.run_launcher("start", "--profile", "trna")
        second = self.run_launcher("start", "--profile", "trna")

        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(second.returncode, 0, second.stderr)
        session_name = f"matrix-sonic-desktop-{os.getuid()}"
        calls = parse_call_log(self.tmux_log)
        new_sessions = [call for call in calls if call[0] == "new-session"]
        self.assertEqual(
            new_sessions,
            [
                [
                    "new-session",
                    "-d",
                    "-s",
                    session_name,
                    "-c",
                    os.fspath(self.project),
                    "--",
                    "/usr/bin/env",
                    "-u",
                    "LD_LIBRARY_PATH",
                    "-u",
                    "PYTHONPATH",
                    "/usr/bin/bash",
                    os.fspath(
                        self.project / "scripts/run_matrix_sonic_moon_v1.sh"
                    ),
                    "--profile",
                    "trna",
                    "--control-source",
                    "game",
                    "--game-fall-recovery",
                    "auto",
                ]
            ],
        )
        self.assertEqual(
            parse_call_log(self.run_log),
            [
                [
                    "--profile",
                    "trna",
                    "--control-source",
                    "game",
                    "--game-fall-recovery",
                    "auto",
                ]
            ],
        )
        self.assertIn(f"tmux attach-session -t ={session_name}", first.stdout)
        self.assertIn("already running", second.stdout)

    def test_concurrent_start_is_serialized_to_one_runtime(self) -> None:
        processes = [
            subprocess.Popen(
                ["/bin/bash", os.fspath(self.launcher), "start"],
                env=self.environment,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            for _ in range(2)
        ]
        results = [process.communicate(timeout=10.0) for process in processes]

        for process, (stdout, stderr) in zip(processes, results, strict=True):
            self.assertEqual(process.returncode, 0, stderr)
            self.assertIn("Matrix SONIC", stdout)
        self.assertEqual(len(parse_call_log(self.run_log)), 1)
        self.assertEqual(
            sum(
                call[0] == "new-session"
                for call in parse_call_log(self.tmux_log)
            ),
            1,
        )

    def test_moon_scene_does_not_inject_trna_policy_into_heyuan(self) -> None:
        result = self.run_launcher(
            "start", "--profile", "heyuan", "--scene", "15"
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            parse_call_log(self.run_log),
            [
                [
                    "--profile",
                    "heyuan",
                    "--control-source",
                    "game",
                    "--game-fall-recovery",
                    "auto",
                ]
            ],
        )
        self.assertIn("scene 15", result.stdout)

    def test_explicit_sonic_policy_overrides_the_moon_default(self) -> None:
        result = self.run_launcher(
            "start",
            "--profile",
            "trna",
            "--scene",
            "15",
            "--initial-locomotion-policy",
            "sonic",
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            parse_call_log(self.run_log),
            [
                [
                    "--profile",
                    "trna",
                    "--control-source",
                    "game",
                    "--game-fall-recovery",
                    "auto",
                    "--initial-locomotion-policy",
                    "sonic",
                ]
            ],
        )

    def test_inherited_sonic_policy_overrides_the_moon_default(self) -> None:
        self.environment["MATRIX_INITIAL_LOCOMOTION_POLICY"] = "sonic"

        result = self.run_launcher("start", "--profile", "trna", "--scene", "15")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            parse_call_log(self.run_log),
            [
                [
                    "--profile",
                    "trna",
                    "--control-source",
                    "game",
                    "--game-fall-recovery",
                    "auto",
                    "--initial-locomotion-policy",
                    "sonic",
                ]
            ],
        )

    def test_invalid_initial_policy_is_rejected_before_tmux(self) -> None:
        result = self.run_launcher(
            "start", "--initial-locomotion-policy", "bfm;touch"
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid initial locomotion policy", result.stderr)
        self.assertEqual(parse_call_log(self.tmux_log), [])

    def test_non_moon_scene_is_forwarded_to_generic_runtime(self) -> None:
        result = self.run_launcher(
            "start", "--profile", "heyuan", "--scene", "2"
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            parse_call_log(self.run_log),
            [
                [
                    "--profile",
                    "heyuan",
                    "--scene",
                    "2",
                    "--control-source",
                    "game",
                ]
            ],
        )
        self.assertIn("scene 2", result.stdout)

    def test_default_profile_status_attach_and_stop(self) -> None:
        started = self.run_launcher()
        status_result = self.run_launcher("status")
        attach_result = self.run_launcher("attach")
        stopped = self.run_launcher("stop")
        stopped_status = self.run_launcher("status")
        stopped_again = self.run_launcher("stop")

        self.assertEqual(started.returncode, 0, started.stderr)
        self.assertEqual(
            parse_call_log(self.run_log)[0],
            [
                "--profile",
                "heyuan",
                "--control-source",
                "game",
                "--game-fall-recovery",
                "auto",
            ],
        )
        self.assertEqual(status_result.returncode, 0, status_result.stderr)
        self.assertIn("is running", status_result.stdout)
        self.assertEqual(attach_result.returncode, 0, attach_result.stderr)
        self.assertIn("fake attach succeeded", attach_result.stdout)
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertFalse(self.tmux_state.exists())
        self.assertEqual(stopped_status.returncode, 1, stopped_status.stderr)
        self.assertIn("is stopped", stopped_status.stdout)
        self.assertEqual(stopped_again.returncode, 0, stopped_again.stderr)
        self.assertIn("already stopped", stopped_again.stdout)

    def test_rejects_profile_and_argument_injection_before_tmux(self) -> None:
        marker = self.root / "profile-injection-ran"
        result = self.run_launcher(
            "start",
            "--profile",
            f"heyuan;touch {marker}",
        )
        unknown = self.run_launcher("start", "--unknown")
        bad_scene = self.run_launcher("start", "--scene", "15;touch")

        self.assertEqual(result.returncode, 2)
        self.assertIn("unsupported profile", result.stderr)
        self.assertEqual(unknown.returncode, 2)
        self.assertIn("unsupported argument", unknown.stderr)
        self.assertEqual(bad_scene.returncode, 2)
        self.assertIn("invalid scene id", bad_scene.stderr)
        self.assertFalse(marker.exists())
        self.assertEqual(parse_call_log(self.tmux_log), [])

    def test_stale_session_is_reported_replaced_and_directly_cleaned(self) -> None:
        self.tmux_state.write_text("1\n", encoding="utf-8")

        stale_status = self.run_launcher("status")
        restarted = self.run_launcher("start")

        self.assertEqual(stale_status.returncode, 1, stale_status.stderr)
        self.assertIn("is stale", stale_status.stdout)
        self.assertEqual(restarted.returncode, 0, restarted.stderr)
        self.assertIn("Removed stale", restarted.stdout)
        self.assertEqual(self.tmux_state.read_text(encoding="utf-8"), "0\n")
        calls = parse_call_log(self.tmux_log)
        self.assertLess(
            next(i for i, call in enumerate(calls) if call[0] == "kill-session"),
            next(i for i, call in enumerate(calls) if call[0] == "new-session"),
        )

        self.tmux_state.write_text("1\n", encoding="utf-8")
        self.tmux_log.unlink()
        stopped = self.run_launcher("stop")

        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertIn("Removed stopped", stopped.stdout)
        self.assertFalse(self.tmux_state.exists())
        self.assertEqual(
            [call[0] for call in parse_call_log(self.tmux_log)],
            [
                "has-session",
                "has-session",
                "list-panes",
                "has-session",
                "has-session",
                "list-panes",
                "kill-session",
                "has-session",
            ],
        )

    def test_startup_failure_does_not_leave_a_dead_tmux_session(self) -> None:
        self.environment["FAKE_TMUX_NEW_PANE_DEAD"] = "1"

        result = self.run_launcher("start")

        self.assertEqual(result.returncode, 1)
        self.assertIn("exited during startup", result.stderr)
        self.assertFalse(self.tmux_state.exists())

    def test_external_host_runtime_is_reported_before_tmux_start(self) -> None:
        with self.host_lock.open("w", encoding="utf-8") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_launcher("start", "--profile", "trna")

        self.assertEqual(result.returncode, 1)
        self.assertIn(
            "Another Matrix SONIC instance owns this host", result.stderr
        )
        self.assertRegex(result.stderr, r"(?:source|lock)=")
        self.assertFalse(self.run_log.exists())
        self.assertFalse(
            any(
                call[0] == "new-session"
                for call in parse_call_log(self.tmux_log)
            )
        )


class MatrixDesktopInstallerTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.project = self.root / "Matrix Repo With Spaces"
        (self.project / "scripts").mkdir(parents=True)
        (self.project / "packaging").mkdir()
        (self.project / "demo_gif").mkdir()
        shutil.copy2(INSTALLER, self.project / "scripts" / INSTALLER.name)
        shutil.copy2(LAUNCHER, self.project / "scripts" / LAUNCHER.name)
        shutil.copy2(TEMPLATE, self.project / "packaging" / TEMPLATE.name)
        (self.project / "demo_gif" / "Launcher.png").write_bytes(b"test icon\n")
        self.installer = self.project / "scripts" / INSTALLER.name

    def run_installer(
        self, *arguments: str
    ) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["HOME"] = os.fspath(self.root / "home")
        return subprocess.run(
            ["/bin/bash", os.fspath(self.installer), *arguments],
            env=environment,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )

    def make_desktop_dir(self, name: str = "Desktop Target") -> Path:
        desktop = self.root / name
        desktop.mkdir(mode=0o700)
        return desktop

    def test_installs_all_profiles_with_template_replacement_and_atomic_mode(self) -> None:
        desktop = self.make_desktop_dir()
        launcher_path = self.project / "scripts" / LAUNCHER.name
        icon_path = self.project / "demo_gif" / "Launcher.png"

        for profile in ("heyuan", "trna", "zza"):
            with self.subTest(profile=profile):
                result = self.run_installer(
                    "--desktop-dir",
                    os.fspath(desktop),
                    "--profile",
                    profile,
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                target = desktop / "matrix-sonic.desktop"
                content = target.read_text(encoding="utf-8")
                self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o755)
                self.assertIn("Terminal=false\n", content)
                self.assertIn(
                    f"Name=Matrix MoonWorld SONIC ({profile})\n",
                    content,
                )
                self.assertIn(
                    "Comment=Launch MoonWorld with BFM SONIC Teacher50k and fall recovery\n",
                    content,
                )
                self.assertIn(
                    f'Exec=/usr/bin/bash "{launcher_path}" start --profile {profile} --scene 15\n',
                    content,
                )
                self.assertIn(
                    f'Exec=/usr/bin/bash "{launcher_path}" status --profile {profile} --scene 15\n',
                    content,
                )
                self.assertIn(
                    f'Exec=/usr/bin/bash "{launcher_path}" stop --profile {profile} --scene 15\n',
                    content,
                )
                self.assertIn(f"Icon={icon_path}\n", content)
                self.assertIn(f"X-Matrix-Repository={self.project}\n", content)
                self.assertIn(f"X-Matrix-Profile={profile}\n", content)
                self.assertIn("X-Matrix-Scene=15\n", content)
                self.assertNotIn("@MATRIX_", content)
                self.assertEqual(
                    list(desktop.glob(".matrix-sonic.desktop.tmp.*")),
                    [],
                )

    def test_city_shortcut_coexists_with_the_default_moon_shortcut(self) -> None:
        desktop = self.make_desktop_dir()
        default_result = self.run_installer(
            "--desktop-dir", os.fspath(desktop), "--profile", "heyuan"
        )
        city_result = self.run_installer(
            "--desktop-dir",
            os.fspath(desktop),
            "--profile",
            "heyuan",
            "--scene",
            "2",
        )

        self.assertEqual(default_result.returncode, 0, default_result.stderr)
        self.assertEqual(city_result.returncode, 0, city_result.stderr)
        self.assertTrue((desktop / "matrix-sonic.desktop").is_file())
        city = desktop / "matrix-sonic-scene-2.desktop"
        self.assertEqual(stat.S_IMODE(city.stat().st_mode), 0o755)
        content = city.read_text(encoding="utf-8")
        self.assertIn("Name=Matrix SONIC scene 2 (heyuan)\n", content)
        self.assertIn("Launch Matrix SONIC scene 2", content)
        self.assertIn("start --profile heyuan --scene 2\n", content)
        self.assertIn("X-Matrix-Scene=2\n", content)
        self.assertNotIn("@MATRIX_", content)

    def test_custom_icon_is_written_as_an_absolute_path(self) -> None:
        desktop = self.make_desktop_dir()
        icon = self.root / "icons" / "custom icon.png"
        icon.parent.mkdir()
        icon.write_bytes(b"custom\n")
        result = self.run_installer(
            "--desktop-dir",
            os.fspath(desktop),
            "--icon",
            os.fspath(icon),
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        content = (desktop / "matrix-sonic.desktop").read_text(encoding="utf-8")
        self.assertIn(f"Icon={icon.resolve()}\n", content)

    def test_rejects_profile_and_path_injection(self) -> None:
        desktop = self.make_desktop_dir()
        marker = self.root / "installer-injection-ran"
        bad_profile = self.run_installer(
            "--desktop-dir",
            os.fspath(desktop),
            "--profile",
            f"heyuan;touch {marker}",
        )
        reserved_icon = self.root / "icon$injection.png"
        reserved_icon.write_bytes(b"icon\n")
        bad_icon = self.run_installer(
            "--desktop-dir",
            os.fspath(desktop),
            "--icon",
            os.fspath(reserved_icon),
        )
        unknown = self.run_installer(
            "--desktop-dir",
            os.fspath(desktop),
            "--unknown",
            "value",
        )

        self.assertEqual(bad_profile.returncode, 2)
        self.assertIn("unsupported profile", bad_profile.stderr)
        self.assertEqual(bad_icon.returncode, 2)
        self.assertIn("reserved by Desktop Entry Exec", bad_icon.stderr)
        self.assertEqual(unknown.returncode, 2)
        self.assertIn("unsupported argument", unknown.stderr)
        self.assertFalse(marker.exists())
        self.assertFalse((desktop / "matrix-sonic.desktop").exists())

    def test_rejects_symlink_and_dangerous_desktop_directories(self) -> None:
        real_desktop = self.make_desktop_dir("Real Desktop")
        linked_desktop = self.root / "Linked Desktop"
        linked_desktop.symlink_to(real_desktop, target_is_directory=True)
        linked = self.run_installer(
            "--desktop-dir",
            os.fspath(linked_desktop),
        )
        dangerous = self.run_installer("--desktop-dir", "/tmp")

        shared_desktop = self.make_desktop_dir("Shared Desktop")
        shared_desktop.chmod(0o777)
        shared = self.run_installer(
            "--desktop-dir",
            os.fspath(shared_desktop),
        )

        self.assertEqual(linked.returncode, 2)
        self.assertIn("symlink components", linked.stderr)
        self.assertEqual(dangerous.returncode, 2)
        self.assertIn("dangerous desktop directory", dangerous.stderr)
        self.assertEqual(shared.returncode, 2)
        self.assertIn("world-writable", shared.stderr)

    def test_accepts_owner_controlled_group_writable_desktop(self) -> None:
        desktop = self.make_desktop_dir("GNOME Desktop")
        desktop.chmod(0o775)

        result = self.run_installer("--desktop-dir", os.fspath(desktop))

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue((desktop / "matrix-sonic.desktop").is_file())

    def test_rejects_symlink_target_without_touching_victim(self) -> None:
        desktop = self.make_desktop_dir()
        victim = self.root / "victim.desktop"
        victim.write_text("keep me\n", encoding="utf-8")
        target = desktop / "matrix-sonic.desktop"
        target.symlink_to(victim)

        result = self.run_installer("--desktop-dir", os.fspath(desktop))

        self.assertEqual(result.returncode, 2)
        self.assertIn("symlink desktop target", result.stderr)
        self.assertEqual(victim.read_text(encoding="utf-8"), "keep me\n")
        self.assertTrue(target.is_symlink())

    def test_sources_do_not_embed_machine_paths(self) -> None:
        for path in (LAUNCHER, INSTALLER, TEMPLATE):
            with self.subTest(path=path.name):
                content = path.read_text(encoding="utf-8")
                self.assertNotIn("/home/", content)


if __name__ == "__main__":
    unittest.main()
