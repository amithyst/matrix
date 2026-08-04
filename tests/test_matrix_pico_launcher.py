from __future__ import annotations

from pathlib import Path
import subprocess
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = REPO_ROOT / "scripts/run_matrix_pico.sh"


class MatrixPicoLauncherTest(unittest.TestCase):
    def run_launcher(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["bash", str(LAUNCHER), *arguments],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

    def test_dry_run_defaults_to_town10_and_locked_game_control(self) -> None:
        completed = self.run_launcher("--dry-run", "--profile", "trna")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("MATRIX_DEV_LAUNCH scene=2 control_source=game", completed.stdout)
        self.assertIn("run_matrix_sonic.sh", completed.stdout)
        self.assertIn("--scene 2", completed.stdout)
        self.assertIn("--control-source game", completed.stdout)
        self.assertIn("--profile trna", completed.stdout)

    def test_explicit_scene_is_canonicalized(self) -> None:
        completed = self.run_launcher("--scene=002", "--dry-run")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("scene=2", completed.stdout)
        self.assertIn("--scene 2", completed.stdout)

    def test_control_source_override_is_rejected(self) -> None:
        for arguments in (
            ("--control-source", "planner", "--dry-run"),
            ("--control-source=game", "--dry-run"),
        ):
            with self.subTest(arguments=arguments):
                completed = self.run_launcher(*arguments)
                self.assertEqual(completed.returncode, 2)
                self.assertIn("fixes --control-source game", completed.stderr)

    def test_invalid_scene_is_rejected(self) -> None:
        for value in ("-1", "100", "town10", ""):
            with self.subTest(value=value):
                completed = self.run_launcher("--scene", value, "--dry-run")
                self.assertEqual(completed.returncode, 2)
                self.assertIn("integer in [0, 99]", completed.stderr)

    def test_realscan_dry_run_remains_fail_closed_until_install(self) -> None:
        completed = self.run_launcher("--scene", "18", "--dry-run")
        self.assertEqual(completed.returncode, 2)
        self.assertIn("installed physics asset", completed.stderr)

    def test_game_and_pico_paths_keep_build_provenance(self) -> None:
        text = (REPO_ROOT / "scripts/run_matrix_sonic.sh").read_text()
        self.assertIn(
            'if [[ "$CONTROL_SOURCE" == "game" || "$CONTROL_SOURCE" == "pico" ]]',
            text,
        )


if __name__ == "__main__":
    unittest.main()
