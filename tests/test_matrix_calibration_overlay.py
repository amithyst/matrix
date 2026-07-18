from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import tempfile
import time
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_calibration_overlay.py"
SPEC = importlib.util.spec_from_file_location("matrix_calibration_overlay", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class OverlayLayoutTest(unittest.TestCase):
    def test_crosshair_intersects_exact_client_centre(self) -> None:
        geometry = MODULE.WindowGeometry(
            window=41,
            x=100,
            y=80,
            width=801,
            height=601,
        )
        layout = MODULE.overlay_layout(geometry)
        centre_x, centre_y = geometry.centre

        horizontal = layout["horizontal"]
        vertical = layout["vertical"]
        self.assertLessEqual(horizontal[0], centre_x)
        self.assertGreater(horizontal[0] + horizontal[2], centre_x)
        self.assertLessEqual(horizontal[1], centre_y)
        self.assertGreater(horizontal[1] + horizontal[3], centre_y)
        self.assertLessEqual(vertical[0], centre_x)
        self.assertGreater(vertical[0] + vertical[2], centre_x)
        self.assertLessEqual(vertical[1], centre_y)
        self.assertGreater(vertical[1] + vertical[3], centre_y)

    def test_hint_flips_above_centre_when_client_bottom_is_tight(self) -> None:
        geometry = MODULE.WindowGeometry(
            window=1,
            x=10,
            y=20,
            width=500,
            height=100,
        )
        hint = MODULE.overlay_layout(geometry)["hint"]
        self.assertLess(hint[1], geometry.centre[1])


class OverlayStateTest(unittest.TestCase):
    def test_state_is_visible_only_for_exact_versioned_true(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "state.json"
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text("{", encoding="utf-8")
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 2, "active": True}))
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 1, "active": 1}))
            self.assertFalse(MODULE.read_active_state(path))
            path.write_text(json.dumps({"version": 1, "active": True}))
            self.assertTrue(MODULE.read_active_state(path))


class TargetCacheTest(unittest.TestCase):
    def test_cached_live_pid_avoids_rewalking_the_x11_tree(self) -> None:
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay.expected_ue_pid = 1234
        overlay._target_window = 99
        overlay._window_pid = lambda window: 1234 if window == 99 else None
        expected = MODULE.WindowGeometry(99, 1, 2, 800, 600)
        overlay._geometry = lambda window: expected if window == 99 else None
        overlay._children = lambda _window: self.fail("cached target walked tree")

        self.assertEqual(overlay.find_target(), expected)

    def test_invalid_cached_window_is_replaced_by_largest_viewable_match(self) -> None:
        overlay = object.__new__(MODULE.X11CalibrationOverlay)
        overlay.expected_ue_pid = 1234
        overlay._root = 1
        overlay._target_window = 90
        tree = {1: [10, 20], 10: [], 20: []}
        overlay._children = lambda window: tree[window]
        overlay._window_pid = lambda window: {
            90: None,
            10: 1234,
            20: 1234,
        }.get(window)
        geometries = {
            10: MODULE.WindowGeometry(10, 0, 0, 320, 240),
            20: MODULE.WindowGeometry(20, 0, 0, 1280, 720),
        }
        overlay._geometry = geometries.get

        self.assertEqual(overlay.find_target(), geometries[20])
        self.assertEqual(overlay._target_window, 20)


@unittest.skipUnless(sys.platform.startswith("linux"), "Linux prctl is required")
class ParentDeathTest(unittest.TestCase):
    def test_overlay_child_dies_when_its_provider_is_sigkilled(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            ready = Path(temporary) / "ready"
            child_source = (
                "import os, pathlib, sys, time; "
                f"sys.path.insert(0, {os.fspath(REPO_ROOT / 'scripts')!r}); "
                "import matrix_calibration_overlay as overlay; "
                "overlay.arm_parent_death_signal(os.getppid()); "
                f"pathlib.Path({os.fspath(ready)!r}).write_text('ready'); "
                "time.sleep(30)"
            )
            helper_source = (
                "import pathlib, subprocess, sys, time; "
                f"child=subprocess.Popen([sys.executable, '-c', {child_source!r}]); "
                f"ready=pathlib.Path({os.fspath(ready)!r}); "
                "deadline=time.monotonic()+5; "
                "\nwhile not ready.exists() and time.monotonic()<deadline: time.sleep(.01)\n"
                "print(child.pid, flush=True); time.sleep(30)"
            )
            helper = subprocess.Popen(
                [sys.executable, "-c", helper_source],
                stdout=subprocess.PIPE,
                text=True,
            )
            assert helper.stdout is not None
            line = helper.stdout.readline().strip()
            self.assertTrue(line.isdigit(), msg=f"helper output: {line!r}")
            child_pid = int(line)
            try:
                os.kill(helper.pid, signal.SIGKILL)
                helper.wait(timeout=3.0)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    stat = Path(f"/proc/{child_pid}/stat")
                    if not stat.exists():
                        break
                    fields = stat.read_text(encoding="utf-8").split()
                    if len(fields) > 2 and fields[2] == "Z":
                        break
                    time.sleep(0.02)
                else:
                    self.fail("overlay child survived provider SIGKILL")
            finally:
                try:
                    os.kill(child_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                if helper.poll() is None:
                    helper.kill()
                    helper.wait(timeout=2.0)
                helper.stdout.close()


@unittest.skipUnless(
    all(shutil.which(command) for command in ("Xvfb", "xdotool", "xmessage", "xprop", "xwininfo")),
    "Xvfb and X11 smoke-test tools are required",
)
class X11IntegrationTest(unittest.TestCase):
    @staticmethod
    def run_x11(environment: dict[str, str], *command: str) -> str:
        return subprocess.run(
            command,
            env=environment,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=True,
        ).stdout

    @staticmethod
    def wait_until(predicate, *, timeout: float = 5.0):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                last = predicate()
                if last:
                    return last
            except (OSError, subprocess.SubprocessError, ValueError):
                pass
            time.sleep(0.03)
        raise AssertionError(f"X11 smoke condition timed out; last={last!r}")

    @classmethod
    def geometry(cls, environment: dict[str, str], window: str) -> dict[str, int]:
        output = cls.run_x11(environment, "xdotool", "getwindowgeometry", "--shell", window)
        values: dict[str, int] = {}
        for line in output.splitlines():
            key, separator, value = line.partition("=")
            if separator and key in {"X", "Y", "WIDTH", "HEIGHT"}:
                values[key] = int(value)
        if set(values) != {"X", "Y", "WIDTH", "HEIGHT"}:
            raise ValueError(f"incomplete geometry: {output!r}")
        return values

    @classmethod
    def client_geometry(
        cls, environment: dict[str, str], window: str
    ) -> dict[str, int]:
        output = cls.run_x11(environment, "xwininfo", "-id", window)
        labels = {
            "Absolute upper-left X": "X",
            "Absolute upper-left Y": "Y",
            "Width": "WIDTH",
            "Height": "HEIGHT",
        }
        values: dict[str, int] = {}
        for line in output.splitlines():
            label, separator, value = line.strip().partition(":")
            if separator and label in labels:
                values[labels[label]] = int(value.strip())
        if set(values) != {"X", "Y", "WIDTH", "HEIGHT"}:
            raise ValueError(f"incomplete client geometry: {output!r}")
        return values

    def test_click_through_overlay_follows_client_and_preserves_focus(self) -> None:
        xvfb = subprocess.Popen(
            ["Xvfb", "-displayfd", "1", "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        assert xvfb.stdout is not None
        display_number = xvfb.stdout.readline().strip()
        self.assertTrue(display_number.isdigit(), msg=f"Xvfb display={display_number!r}")
        environment = {**os.environ, "DISPLAY": f":{display_number}"}
        target: subprocess.Popen[bytes] | None = None
        overlay: subprocess.Popen[bytes] | None = None
        try:
            target = subprocess.Popen(
                [
                    "xmessage",
                    "-title",
                    "MatrixSmoke",
                    "-borderwidth",
                    "0",
                    "-geometry",
                    "800x600+100+80",
                    "target",
                ],
                env=environment,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            target_window = self.wait_until(
                lambda: self.run_x11(
                    environment, "xdotool", "search", "--name", "^MatrixSmoke$"
                ).splitlines()[0]
            )
            self.run_x11(
                environment,
                "xprop",
                "-id",
                target_window,
                "-f",
                "_NET_WM_PID",
                "32c",
                "-set",
                "_NET_WM_PID",
                str(target.pid),
            )
            self.run_x11(environment, "xdotool", "windowfocus", target_window)
            focus_before = self.run_x11(environment, "xdotool", "getwindowfocus").strip()

            with tempfile.TemporaryDirectory() as temporary:
                state = Path(temporary) / "state.json"
                status = Path(temporary) / "status.json"
                MODULE.atomic_json(state, {"version": 1, "active": True})
                overlay = subprocess.Popen(
                    [
                        sys.executable,
                        os.fspath(SCRIPT),
                        "--state-file",
                        os.fspath(state),
                        "--status-file",
                        os.fspath(status),
                        "--expected-ue-pid",
                        str(target.pid),
                        "--expected-parent-pid",
                        str(os.getpid()),
                        "--display",
                        environment["DISPLAY"],
                        "--poll-hz",
                        "60",
                    ],
                    env=environment,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                )
                self.wait_until(
                    lambda: status.is_file()
                    and json.loads(status.read_text(encoding="utf-8")).get("ready") is True
                )
                horizontal = self.wait_until(
                    lambda: self.run_x11(
                        environment,
                        "xdotool",
                        "search",
                        "--name",
                        "^Matrix Calibration horizontal$",
                    ).splitlines()[0]
                )
                overlay_windows = {"horizontal": int(horizontal)}
                for role in (
                    "horizontal-shadow",
                    "vertical-shadow",
                    "vertical",
                    "hint",
                ):
                    window = self.wait_until(
                        lambda role=role: self.run_x11(
                            environment,
                            "xdotool",
                            "search",
                            "--name",
                            f"^Matrix Calibration {role}$",
                        ).splitlines()[0]
                    )
                    overlay_windows[role] = int(window)
                # xdotool reports the outer origin for bordered test windows;
                # the overlay intentionally follows X11 client coordinates.
                target_geometry = self.client_geometry(environment, target_window)
                observed_centres: list[dict[str, int]] = []

                def centred_geometry():
                    value = self.geometry(environment, horizontal)
                    observed_centres.append(value)
                    expected_x = target_geometry["X"] + target_geometry["WIDTH"] // 2 - 32
                    expected_y = target_geometry["Y"] + target_geometry["HEIGHT"] // 2 - 1
                    return value if (value["X"], value["Y"]) == (expected_x, expected_y) else None

                try:
                    self.wait_until(centred_geometry)
                except AssertionError as exc:
                    stderr = ""
                    if overlay.poll() is not None and overlay.stderr is not None:
                        stderr = overlay.stderr.read().decode("utf-8", errors="replace")
                    self.fail(
                        f"{exc}; target={target_geometry}; "
                        f"last_cross={observed_centres[-1] if observed_centres else None}; "
                        f"overlay_code={overlay.poll()}; stderr={stderr!r}"
                    )
                focus_after = self.run_x11(environment, "xdotool", "getwindowfocus").strip()
                self.assertEqual(focus_after, focus_before)

                centre_x = target_geometry["X"] + target_geometry["WIDTH"] // 2
                centre_y = target_geometry["Y"] + target_geometry["HEIGHT"] // 2
                self.run_x11(
                    environment, "xdotool", "mousemove", str(centre_x), str(centre_y)
                )
                pointer = self.run_x11(
                    environment, "xdotool", "getmouselocation", "--shell"
                )
                pointer_window = next(
                    int(line.partition("=")[2])
                    for line in pointer.splitlines()
                    if line.startswith("WINDOW=")
                )
                self.assertNotIn(pointer_window, set(overlay_windows.values()))

                self.run_x11(environment, "xdotool", "windowmove", target_window, "20", "30")
                self.run_x11(environment, "xdotool", "windowsize", target_window, "700", "500")

                def moved_and_followed():
                    moved_target = self.client_geometry(environment, target_window)
                    moved_cross = self.geometry(environment, horizontal)
                    return (
                        moved_cross
                        if (
                            moved_cross["X"]
                            == moved_target["X"] + moved_target["WIDTH"] // 2 - 32
                            and moved_cross["Y"]
                            == moved_target["Y"] + moved_target["HEIGHT"] // 2 - 1
                        )
                        else None
                    )

                self.wait_until(moved_and_followed)
                MODULE.atomic_json(state, {"version": 1, "active": False})

                def is_unmapped():
                    info = self.run_x11(environment, "xwininfo", "-id", horizontal)
                    return "Map State: IsUnMapped" in info

                self.wait_until(is_unmapped)
        finally:
            for process in (overlay, target):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
            if overlay is not None and overlay.stderr is not None:
                overlay.stderr.close()
            if xvfb.poll() is None:
                xvfb.terminate()
                try:
                    xvfb.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    xvfb.kill()
                    xvfb.wait(timeout=2.0)
            xvfb.stdout.close()
            assert xvfb.stderr is not None
            xvfb.stderr.close()


if __name__ == "__main__":
    unittest.main()
