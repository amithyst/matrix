from __future__ import annotations

from dataclasses import asdict
import importlib.util
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))


def load_script(name: str):
    path = SCRIPT_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


LEDGER = load_script("matrix_bfm_isaac_instance_ledger")
RESOURCE = load_script("matrix_bfm_isaac_resource_guard")


def write_fake_process(
    proc_root: Path,
    pid: int,
    *,
    ppid: int = 1,
    pgid: int | None = None,
    sid: int | None = None,
    starttime: int = 100,
    uid: int | None = None,
    nonce: str | None = None,
    executable: str = "/usr/bin/python3",
    cwd: str = "/tmp",
    tokens: tuple[str, ...] = ("python3",),
    comm: str = "worker",
) -> Path:
    process_dir = proc_root / str(pid)
    process_dir.mkdir(parents=True)
    actual_pgid = pid if pgid is None else pgid
    actual_sid = actual_pgid if sid is None else sid
    fields = ["S", str(ppid), str(actual_pgid), str(actual_sid)]
    fields.extend("0" for _ in range(15))
    fields.append(str(starttime))
    (process_dir / "stat").write_text(
        f"{pid} ({comm}) {' '.join(fields)}\n", encoding="utf-8"
    )
    actual_uid = os.getuid() if uid is None else uid
    uid_fields = "\t".join(str(actual_uid) for _ in range(4))
    (process_dir / "status").write_text(
        f"Name:\t{comm}\nUid:\t{uid_fields}\n",
        encoding="utf-8",
    )
    environment = b"PATH=/usr/bin\0"
    if nonce is not None:
        environment += f"{LEDGER.NONCE_ENV}={nonce}".encode() + b"\0"
    (process_dir / "environ").write_bytes(environment)
    (process_dir / "cmdline").write_bytes(
        b"\0".join(token.encode() for token in tokens) + b"\0"
    )
    (process_dir / "exe").symlink_to(executable)
    (process_dir / "cwd").symlink_to(cwd)
    return process_dir


def write_ledger(path: Path, nonce: str, group: object) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "xvi.mbsc.instance_ledger",
                "version": 1,
                "nonce": nonce,
                "launcher": {},
                "groups": [group],
            }
        ),
        encoding="utf-8",
    )


class MatrixBfmIsaacInstanceLedgerTest(unittest.TestCase):
    def test_process_identity_uses_pid_group_session_starttime_and_uid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc_root = Path(temporary) / "proc"
            proc_root.mkdir()
            write_fake_process(
                proc_root,
                321,
                pgid=444,
                sid=555,
                starttime=777,
                nonce="run-a",
                comm="worker ) with spaces",
            )

            self.assertEqual(
                LEDGER.process_identity(321, proc_root),
                LEDGER.ProcessIdentity(
                    pid=321,
                    pgid=444,
                    sid=555,
                    starttime=777,
                    uid=os.getuid(),
                ),
            )
            self.assertEqual(LEDGER.process_nonce(321, proc_root), "run-a")
            self.assertIsNone(LEDGER.process_identity(999, proc_root))
            self.assertIsNone(LEDGER.process_nonce(999, proc_root))

    def test_reused_pid_requires_matching_nonce_before_group_is_owned(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            proc_root = root / "proc"
            proc_root.mkdir()
            ledger_path = root / "owner-ledger.json"
            recorded = LEDGER.ProcessIdentity(
                pid=123,
                pgid=123,
                sid=123,
                starttime=100,
                uid=os.getuid(),
            )
            write_ledger(ledger_path, "run-a", asdict(recorded))
            process_dir = write_fake_process(
                proc_root,
                123,
                pgid=123,
                sid=123,
                starttime=101,
                nonce="foreign-run",
            )

            self.assertEqual(
                LEDGER.safe_group_targets(ledger_path, proc_root=proc_root), ()
            )
            self.assertEqual(
                LEDGER.owned_processes(ledger_path, proc_root=proc_root), ()
            )

            (process_dir / "environ").write_bytes(
                f"{LEDGER.NONCE_ENV}=run-a\0".encode()
            )
            self.assertEqual(
                LEDGER.safe_group_targets(ledger_path, proc_root=proc_root), (123,)
            )
            self.assertEqual(
                tuple(
                    item.pid
                    for item in LEDGER.owned_processes(
                        ledger_path, proc_root=proc_root
                    )
                ),
                (123,),
            )

    def test_expected_nonce_mismatch_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            proc_root = root / "proc"
            proc_root.mkdir()
            ledger_path = root / "owner-ledger.json"
            write_ledger(
                ledger_path,
                "run-a",
                asdict(LEDGER.ProcessIdentity(123, 123, 123, 100, os.getuid())),
            )

            for operation in (LEDGER.safe_group_targets, LEDGER.owned_processes):
                with self.subTest(operation=operation.__name__), self.assertRaisesRegex(
                    ValueError, "nonce mismatch"
                ):
                    operation(
                        ledger_path,
                        expected_nonce="run-b",
                        proc_root=proc_root,
                    )

    def test_owned_process_by_pid_selects_only_nonce_bound_member(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            proc_root = root / "proc"
            proc_root.mkdir()
            ledger_path = root / "owner-ledger.json"
            leader = LEDGER.ProcessIdentity(
                123, 123, 123, 100, os.getuid()
            )
            write_ledger(ledger_path, "run-a", asdict(leader))
            write_fake_process(
                proc_root,
                123,
                pgid=123,
                sid=123,
                starttime=100,
                nonce="run-a",
            )
            write_fake_process(
                proc_root,
                124,
                ppid=123,
                pgid=123,
                sid=123,
                starttime=101,
                nonce="run-a",
            )
            write_fake_process(
                proc_root,
                125,
                ppid=123,
                pgid=123,
                sid=123,
                starttime=102,
                nonce="foreign",
            )

            selected = LEDGER.owned_process_by_pid(
                ledger_path,
                124,
                expected_nonce="run-a",
                proc_root=proc_root,
            )
            self.assertIsNotNone(selected)
            self.assertEqual(selected.pid, 124)
            self.assertIsNone(
                LEDGER.owned_process_by_pid(
                    ledger_path,
                    125,
                    expected_nonce="run-a",
                    proc_root=proc_root,
                )
            )


class MatrixBfmIsaacResourceGuardTest(unittest.TestCase):
    def test_guarded_launcher_shares_the_native_matrix_host_lock(self) -> None:
        launcher = SCRIPT_DIR / "run_matrix_bfm_isaac_guarded.sh"
        launcher_text = launcher.read_text(encoding="utf-8")
        self.assertIn(
            'MATRIX_SONIC_HOST_LOCK="${MATRIX_SONIC_HOST_LOCK:-/tmp/matrix-sonic-${UID}.lock}"',
            launcher_text,
        )
        self.assertIn('exec 8>"$MATRIX_SONIC_HOST_LOCK"', launcher_text)
        self.assertIn("flock -n 8", launcher_text)

        with tempfile.TemporaryDirectory() as temporary:
            lock_path = Path(temporary) / "matrix-sonic-host.lock"
            with lock_path.open("w", encoding="utf-8") as lock_handle:
                fcntl.flock(lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
                result = subprocess.run(
                    ("bash", str(launcher), "smoke"),
                    env={
                        **os.environ,
                        "MATRIX_SONIC_HOST_LOCK": os.fspath(lock_path),
                    },
                    text=True,
                    capture_output=True,
                    check=False,
                    timeout=10,
                )

        self.assertEqual(result.returncode, 75, result.stderr)
        self.assertIn("Another Matrix launcher owns this host", result.stderr)

    def test_launcher_checks_latched_shutdown_before_isaac_start(self) -> None:
        launcher = (SCRIPT_DIR / "run_matrix_bfm_isaac.sh").read_text(
            encoding="utf-8"
        )

        self.assertIn("abort_startup_if_shutdown_requested()", launcher)
        self.assertGreaterEqual(
            launcher.count("abort_startup_if_shutdown_requested"),
            3,
        )
        self.assertIn("Isaac was not started", launcher)
        self.assertIn(
            'if [[ -n "$SHUTDOWN_SIGNAL" ]]; then\n'
            "    # A signal can be latched after the last readiness check",
            launcher,
        )
        self.assertLess(
            launcher.index('if [[ -n "$SHUTDOWN_SIGNAL" ]]; then\n'
                           "    # A signal can be latched"),
            launcher.index('wait -n -p COMPLETED_PID "${WAIT_PIDS[@]}"'),
        )

    def test_nonce_scan_requires_both_nonce_and_uid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc_root = Path(temporary) / "proc"
            proc_root.mkdir()
            write_fake_process(proc_root, 10, nonce="run-a")
            write_fake_process(proc_root, 11, nonce="run-b")
            write_fake_process(proc_root, 12, nonce="run-a", uid=os.getuid() + 1)

            self.assertEqual(
                tuple(
                    item.pid
                    for item in RESOURCE.nonce_processes(
                        "run-a", proc_root=proc_root
                    )
                ),
                (10,),
            )

    def test_foreign_scan_ignores_rg_arbitrary_args_own_root_and_prefix_sibling(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc_root = Path(temporary) / "proc"
            proc_root.mkdir()
            foreign_root = "/srv/colleague/matrix"
            own_root = "/srv/ours/matrix"
            foreign_script = f"{foreign_root}/scripts/run_sim.sh"
            own_script = f"{own_root}/scripts/run_sim.sh"

            write_fake_process(
                proc_root,
                100,
                executable="/usr/bin/bash",
                tokens=("bash", foreign_script),
            )
            write_fake_process(
                proc_root,
                101,
                executable="/usr/bin/rg",
                tokens=("rg", foreign_script),
            )
            write_fake_process(
                proc_root,
                102,
                executable="/usr/bin/python3",
                tokens=("python3", "/tmp/audit.py", foreign_script),
            )
            write_fake_process(
                proc_root,
                103,
                executable="/usr/bin/bash",
                tokens=("bash", own_script),
            )
            write_fake_process(
                proc_root,
                104,
                executable="/usr/bin/bash",
                tokens=("bash", "/srv/colleague/matrix-copy/scripts/run_sim.sh"),
            )
            write_fake_process(
                proc_root,
                105,
                executable=(
                    f"{foreign_root}/src/UeSim/Linux/zsibot_mujoco_ue"
                ),
                tokens=(f"{foreign_root}/src/UeSim/Linux/zsibot_mujoco_ue",),
            )
            write_fake_process(
                proc_root,
                106,
                executable="/usr/bin/bash",
                tokens=("bash", "-c", foreign_script),
            )

            foreign = RESOURCE.scan_foreign_processes(
                own_root=own_root,
                proc_root=proc_root,
                foreign_roots=(foreign_root,),
            )

            self.assertEqual(tuple(item.pid for item in foreign), (100, 105))
            self.assertNotIn("rg", " ".join(item.command for item in foreign))


if __name__ == "__main__":
    unittest.main()
