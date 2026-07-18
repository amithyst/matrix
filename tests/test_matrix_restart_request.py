from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts/matrix_restart_request.py"
PROVIDER = REPO_ROOT / "scripts/matrix_game_control_input.py"
SPEC = importlib.util.spec_from_file_location("matrix_restart_request_tested", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class RestartRequestTest(unittest.TestCase):
    def private_root(self, temporary: str) -> Path:
        root = Path(temporary) / "private"
        root.mkdir(mode=0o700)
        return root

    def test_capability_and_request_are_private_atomic_files(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self.private_root(temporary)
            capability = root / "capability"
            request_path = root / "request.json"
            MODULE.atomic_write_capability(capability)
            nonce = MODULE.read_capability(capability)
            MODULE.atomic_write_request(
                request_path,
                MODULE.RestartRequest(
                    launcher_pid=os.getpid(),
                    provider_pid=os.getpid(),
                    nonce=nonce,
                ),
            )
            self.assertEqual(stat.S_IMODE(capability.stat().st_mode), 0o600)
            self.assertEqual(stat.S_IMODE(request_path.stat().st_mode), 0o600)

    def test_validation_binds_live_descendant_and_pinned_script(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self.private_root(temporary)
            capability = root / "capability"
            request_path = root / "request.json"
            MODULE.atomic_write_capability(capability)
            provider = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "import time; time.sleep(30)",
                    os.fspath(PROVIDER),
                ]
            )
            try:
                MODULE.atomic_write_request(
                    request_path,
                    MODULE.RestartRequest(
                        launcher_pid=os.getpid(),
                        provider_pid=provider.pid,
                        nonce=MODULE.read_capability(capability),
                    ),
                )
                validated = MODULE.validate_request(
                    request_path,
                    expected_launcher_pid=os.getpid(),
                    expected_run_sim_pid=os.getpid(),
                    expected_provider_script=PROVIDER,
                    expected_nonce=MODULE.read_capability(capability),
                    consume=True,
                )
                self.assertEqual(validated.provider_pid, provider.pid)
                self.assertFalse(request_path.exists())
            finally:
                provider.terminate()
                provider.wait(timeout=5)

    @unittest.skipUnless(hasattr(os, "mkfifo"), "FIFO requires POSIX")
    def test_fifo_request_is_rejected_without_blocking(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self.private_root(temporary)
            request_path = root / "request.json"
            os.mkfifo(request_path, mode=0o600)
            started = time.monotonic()
            with self.assertRaises(PermissionError):
                MODULE._read_private_request(request_path)
            self.assertLess(time.monotonic() - started, 1.0)

    def test_long_lived_watcher_returns_only_after_valid_request(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = self.private_root(temporary)
            capability = root / "capability"
            request_path = root / "request.json"
            MODULE.atomic_write_capability(capability)
            provider = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "import time; time.sleep(30)",
                    os.fspath(PROVIDER),
                ]
            )
            watcher = subprocess.Popen(
                [
                    sys.executable,
                    "-B",
                    "-I",
                    os.fspath(SCRIPT),
                    "watch",
                    "--file",
                    os.fspath(request_path),
                    "--launcher-pid",
                    str(os.getpid()),
                    "--run-sim-pid",
                    str(os.getpid()),
                    "--provider-script",
                    os.fspath(PROVIDER),
                    "--capability-file",
                    os.fspath(capability),
                    "--poll-seconds",
                    "0.05",
                ]
            )
            try:
                MODULE.atomic_write_request(
                    request_path,
                    MODULE.RestartRequest(
                        launcher_pid=os.getpid(),
                        provider_pid=provider.pid,
                        nonce=MODULE.read_capability(capability),
                    ),
                )
                self.assertEqual(
                    watcher.wait(timeout=5), MODULE.WATCH_REQUEST_VALID
                )
                self.assertFalse(request_path.exists())
            finally:
                for process in (watcher, provider):
                    if process.poll() is None:
                        process.send_signal(signal.SIGTERM)
                        process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
