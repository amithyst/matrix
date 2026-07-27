from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts" / "run_matrix_sonic_moon_v1.sh"


class MatrixSonicMoonWrapperTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.scripts = self.root / "scripts"
        self.scripts.mkdir()
        shutil.copy2(LAUNCHER, self.scripts / LAUNCHER.name)
        self.capture = self.root / "capture.json"
        generic = self.scripts / "run_matrix_sonic.sh"
        generic.write_text(
            """#!/usr/bin/env bash
exec python3 - "$@" <<'PY'
import json
import os
from pathlib import Path
import sys
Path(os.environ["CAPTURE"]).write_text(json.dumps({
    "argv": sys.argv[1:],
    "policy_present": "MATRIX_INITIAL_LOCOMOTION_POLICY" in os.environ,
    "policy": os.environ.get("MATRIX_INITIAL_LOCOMOTION_POLICY"),
}))
PY
""",
            encoding="utf-8",
        )
        generic.chmod(0o755)
        self.environment = {
            **os.environ,
            "CAPTURE": os.fspath(self.capture),
        }
        self.environment.pop("MATRIX_INITIAL_LOCOMOTION_POLICY", None)

    def run_launcher(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", os.fspath(self.scripts / LAUNCHER.name), *arguments],
            env=self.environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def captured(self) -> dict[str, object]:
        return json.loads(self.capture.read_text(encoding="utf-8"))

    def test_unset_policy_is_left_for_the_selected_host_profile(self) -> None:
        result = self.run_launcher("--profile", "heyuan")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            self.captured(),
            {
                "argv": ["--scene", "15", "--profile", "heyuan"],
                "policy_present": False,
                "policy": None,
            },
        )

    def test_explicit_environment_and_cli_sonic_overrides_are_preserved(self) -> None:
        self.environment["MATRIX_INITIAL_LOCOMOTION_POLICY"] = "sonic"
        inherited = self.run_launcher("--profile", "trna")
        self.assertEqual(inherited.returncode, 0, inherited.stderr)
        self.assertEqual(self.captured()["policy"], "sonic")

        self.environment.pop("MATRIX_INITIAL_LOCOMOTION_POLICY")
        explicit = self.run_launcher(
            "--profile", "trna", "--initial-locomotion-policy", "sonic"
        )
        self.assertEqual(explicit.returncode, 0, explicit.stderr)
        self.assertEqual(
            self.captured()["argv"],
            [
                "--scene",
                "15",
                "--profile",
                "trna",
                "--initial-locomotion-policy",
                "sonic",
            ],
        )

    def test_scene_override_is_rejected_before_the_generic_launcher(self) -> None:
        result = self.run_launcher("--scene", "2")

        self.assertEqual(result.returncode, 2)
        self.assertIn("fixes the native scene", result.stderr)
        self.assertFalse(self.capture.exists())


if __name__ == "__main__":
    unittest.main()
