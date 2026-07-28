from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


TESTS_DIR = Path(__file__).resolve().parent
if os.fspath(TESTS_DIR) not in sys.path:
    sys.path.insert(0, os.fspath(TESTS_DIR))

import test_matrix_game_control_integration as INTEGRATION  # noqa: E402


class RunSimExternalStateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture_builder = INTEGRATION.LauncherArgumentChainIntegrationTest()

    def _make_project(self, root: Path) -> tuple[Path, dict[str, Path]]:
        project = root / "matrix"
        fixture = self.fixture_builder.make_project(project)
        # Exercise the real JSON update.  The shared integration fixture's jq
        # shim intentionally preserves its input and cannot prove the external
        # renderer receives mujoco_running=true.
        (fixture["fake_bin"] / "jq").unlink()

        invocation_log = project / "runtime-python-invocations.jsonl"
        runtime_python = fixture["fake_bin"] / "external-runtime-python"
        self.fixture_builder.write(
            runtime_python,
            r'''#!/usr/bin/python3
import json
import os
from pathlib import Path
import sys

script = Path(sys.argv[1]).name
args = sys.argv[2:]
with Path(os.environ["RUNTIME_INVOCATION_LOG"]).open("a", encoding="utf-8") as stream:
    stream.write(json.dumps({"script": script, "args": args}) + "\n")

if script != "supervise_matrix_ue.py":
    Path(os.environ["UNEXPECTED_RUNTIME_MARKER"]).write_text(
        script, encoding="utf-8"
    )
    raise SystemExit(91)

pid_file = Path(args[args.index("--pid-file") + 1])
pid_file.write_text(str(os.getpid()), encoding="ascii")
project_root = Path(os.environ["MATRIX_PROJECT_ROOT"])
capture = {
    "config": json.loads(
        (project_root / "config/config.json").read_text(encoding="utf-8")
    ),
    "command": args[args.index("--") + 1 :],
}
Path(os.environ["UE_CAPTURE_PATH"]).write_text(
    json.dumps(capture), encoding="utf-8"
)
''',
            executable=True,
        )

        pkill_log = project / "pkill.log"
        self.fixture_builder.write(
            fixture["fake_bin"] / "pkill",
            """#!/usr/bin/env bash
set -euo pipefail
printf '%s\\n' "$*" >> "${PKILL_LOG:?}"
""",
            executable=True,
        )
        local_mujoco_marker = project / "local-mujoco.started"
        self.fixture_builder.write(
            project / "src/robot_mujoco/simulate/build/robot_mujoco",
            """#!/usr/bin/env bash
set -euo pipefail
printf 'started\\n' > "${LOCAL_MUJOCO_MARKER:?}"
""",
            executable=True,
        )
        local_mc_marker = project / "local-mc.started"
        self.fixture_builder.write(
            project / "src/robot_mc/run_mc.sh",
            """#!/usr/bin/env bash
set -euo pipefail
export ROBOT_TYPE=XG
printf 'started\\n' > "${LOCAL_MC_MARKER:?}"
""",
            executable=True,
        )
        config = project / "config/config.json"
        config.write_text(
            json.dumps(
                {
                    "robot": {
                        "position": {"x": 0, "y": 0, "z": 0},
                        "mujoco_running": False,
                    }
                }
            ),
            encoding="utf-8",
        )
        fixture.update(
            {
                "invocation_log": invocation_log,
                "runtime_python": runtime_python,
                "pkill_log": pkill_log,
                "local_mujoco_marker": local_mujoco_marker,
                "local_mc_marker": local_mc_marker,
                "unexpected_runtime_marker": (
                    project / "unexpected-runtime.started"
                ),
            }
        )
        return project, fixture

    @staticmethod
    def _environment(project: Path, fixture: dict[str, Path]) -> dict[str, str]:
        runtime_dir = project / "runtime"
        runtime_dir.mkdir()
        return {
            "HOME": os.fspath(project / "home"),
            "LANG": "C.UTF-8",
            "LOCAL_MC_MARKER": os.fspath(fixture["local_mc_marker"]),
            "LOCAL_MUJOCO_MARKER": os.fspath(fixture["local_mujoco_marker"]),
            "MATRIX_DISABLE_MC": "0",
            "MATRIX_EXTERNAL_STATE": "1",
            "MATRIX_GAME_CENTERED_CAMERA": "off",
            "MATRIX_PROJECT_ROOT": os.fspath(project),
            "MATRIX_SKIP_ENV_CHECK": "1",
            "MATRIX_SONIC": "0",
            "MATRIX_SONIC_PYTHON": os.fspath(fixture["runtime_python"]),
            "MATRIX_UE_STARTUP_SECONDS": "0",
            "PATH": os.fspath(fixture["fake_bin"])
            + os.pathsep
            + os.environ.get("PATH", "/usr/bin:/bin"),
            "PKILL_LOG": os.fspath(fixture["pkill_log"]),
            "RUNTIME_INVOCATION_LOG": os.fspath(fixture["invocation_log"]),
            "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
            "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
            "UNEXPECTED_RUNTIME_MARKER": os.fspath(
                fixture["unexpected_runtime_marker"]
            ),
            "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
        }

    @staticmethod
    def _run(
        project: Path,
        environment: dict[str, str],
        *,
        mujoco_running: str = "0",
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                "/bin/bash",
                os.fspath(project / "scripts/run_sim.sh"),
                "xgb",
                "21",
                "0",
                "0",
                mujoco_running,
            ],
            env=environment,
            text=True,
            capture_output=True,
            timeout=20.0,
            check=False,
        )

    def test_external_state_starts_only_ue_and_advertises_state_consumer(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project, fixture = self._make_project(Path(temporary))
            environment = self._environment(project, fixture)

            result = self._run(project, environment)

            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertIn(
                "External-state mode: name-wide process cleanup is disabled",
                result.stdout,
            )
            self.assertIn(
                "External-state mode: local MuJoCo and MC are disabled",
                result.stdout,
            )
            self.assertIn(
                "UE external-state consumer enabled without local MuJoCo",
                result.stdout,
            )
            self.assertFalse(fixture["pkill_log"].exists())
            self.assertFalse(fixture["local_mujoco_marker"].exists())
            self.assertFalse(fixture["local_mc_marker"].exists())
            self.assertFalse(fixture["unexpected_runtime_marker"].exists())

            invocations = [
                json.loads(line)
                for line in fixture["invocation_log"]
                .read_text(encoding="utf-8")
                .splitlines()
            ]
            self.assertEqual(
                [invocation["script"] for invocation in invocations],
                ["supervise_matrix_ue.py"],
            )
            capture = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )
            self.assertIs(capture["config"]["robot"]["mujoco_running"], True)
            self.assertTrue(
                any(
                    argument.endswith("zsibot_mujoco_ue.sh")
                    for argument in capture["command"]
                )
            )

    def test_external_state_rejects_non_literal_switch_values(self) -> None:
        for invalid in ("true", "2", " 1"):
            with self.subTest(
                value=invalid
            ), tempfile.TemporaryDirectory() as temporary:
                project, fixture = self._make_project(Path(temporary))
                environment = self._environment(project, fixture)
                environment["MATRIX_EXTERNAL_STATE"] = invalid

                result = self._run(project, environment)

                self.assertEqual(result.returncode, 2)
                self.assertIn(
                    "MATRIX_EXTERNAL_STATE must be the literal 0 or 1",
                    result.stderr,
                )
                self.assertFalse(fixture["pkill_log"].exists())
                self.assertFalse(fixture["ue_capture"].exists())
                self.assertFalse(fixture["local_mujoco_marker"].exists())
                self.assertFalse(fixture["local_mc_marker"].exists())

    def test_external_state_rejects_local_physics_or_sonic_requests(self) -> None:
        cases = (("1", "0", "MUJOCORUNNING=0"), ("0", "1", "MATRIX_SONIC"))
        for mujoco_running, sonic, error_fragment in cases:
            with self.subTest(
                mujoco_running=mujoco_running, sonic=sonic
            ), tempfile.TemporaryDirectory() as temporary:
                project, fixture = self._make_project(Path(temporary))
                environment = self._environment(project, fixture)
                environment["MATRIX_SONIC"] = sonic

                result = self._run(
                    project,
                    environment,
                    mujoco_running=mujoco_running,
                )

                self.assertEqual(result.returncode, 2)
                self.assertIn(error_fragment, result.stderr)
                self.assertFalse(fixture["pkill_log"].exists())
                self.assertFalse(fixture["ue_capture"].exists())
                self.assertFalse(fixture["local_mujoco_marker"].exists())
                self.assertFalse(fixture["local_mc_marker"].exists())
                self.assertFalse(fixture["unexpected_runtime_marker"].exists())


if __name__ == "__main__":
    unittest.main()
