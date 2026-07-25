from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "matrix_build_info.py"
SPEC = importlib.util.spec_from_file_location("matrix_build_info", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixBuildInfoTest(unittest.TestCase):
    @staticmethod
    def git(repository: Path, *arguments: str) -> str:
        return subprocess.run(
            ["git", "-C", os.fspath(repository), *arguments],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        ).stdout

    def repository(self, root: Path) -> Path:
        repository = root / "matrix"
        repository.mkdir()
        self.git(repository, "init", "-q")
        self.git(repository, "config", "user.name", "Matrix Test")
        self.git(repository, "config", "user.email", "matrix@example.invalid")
        (repository / "README.md").write_text("first\n", encoding="utf-8")
        self.git(repository, "add", "README.md")
        self.git(repository, "commit", "-q", "-m", "initial snapshot")
        self.git(repository, "switch", "-q", "-c", "feature/esc-build-info")
        (repository / "README.md").write_text("first\nsecond\n", encoding="utf-8")
        (repository / "runtime.txt").write_text("runtime\n", encoding="utf-8")
        self.git(repository, "add", "README.md", "runtime.txt")
        self.git(
            repository,
            "commit",
            "-q",
            "-m",
            "show launch provenance",
            "-m",
            "Expose the branch and changed files in ESC.",
        )
        return repository

    def test_collects_branch_commit_diff_and_dirty_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            repository = self.repository(Path(temporary))
            clean = MODULE.collect_build_info(
                repository,
                profile="heyuan",
                scene_id=2,
                control_source="game",
            )
            self.assertTrue(clean["available"])
            self.assertEqual(clean["branch"], "feature/esc-build-info")
            self.assertEqual(clean["subject"], "show launch provenance")
            self.assertIn("Expose the branch", clean["body"])
            self.assertEqual(clean["scene_name"], "Town10World")
            self.assertEqual(clean["changed_files"], 2)
            self.assertGreaterEqual(clean["additions"], 2)
            self.assertFalse(clean["dirty"])
            self.assertEqual(
                {item["path"] for item in clean["files"]},
                {"README.md", "runtime.txt"},
            )

            (repository / "runtime.txt").write_text(
                "runtime\nchanged\n", encoding="utf-8"
            )
            (repository / "untracked.txt").write_text("new\n", encoding="utf-8")
            dirty = MODULE.collect_build_info(
                repository,
                profile="trna",
                scene_id=15,
                control_source="game",
            )
            self.assertTrue(dirty["dirty"])
            self.assertEqual(dirty["dirty_files"], 2)
            self.assertEqual(dirty["scene_name"], "MoonWorld")
            self.assertIn("runtime.txt", dirty["dirty_paths"])
            self.assertIn("untracked.txt", dirty["dirty_paths"])

    def test_non_repository_is_renderable_and_does_not_block_launch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            value = MODULE.collect_build_info(
                Path(temporary),
                profile="zza",
                scene_id=6,
                control_source="game",
            )
        self.assertFalse(value["available"])
        self.assertEqual(value["profile"], "zza")
        self.assertEqual(value["scene_name"], "HouseWorld")
        self.assertIsNotNone(value["error"])
        MODULE.validate_build_info(value)

    def test_json_parser_rejects_duplicate_and_extra_fields(self) -> None:
        value = MODULE.unavailable_build_info(
            profile="local",
            scene_id=0,
            control_source="game",
            error="not launched from Git",
        )
        encoded = json.dumps(value, allow_nan=False)
        self.assertEqual(MODULE.parse_build_info_json(encoded), value)
        duplicate = encoded[:-1] + ',"schema":"matrix-build-info/v1"}'
        with self.assertRaisesRegex(MODULE.BuildInfoError, "duplicate"):
            MODULE.parse_build_info_json(duplicate)
        value["unexpected"] = True
        with self.assertRaisesRegex(MODULE.BuildInfoError, "keys are invalid"):
            MODULE.validate_build_info(value)

    def test_invalid_launch_choices_are_rejected_before_git(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertRaisesRegex(MODULE.BuildInfoError, "profile"):
                MODULE.collect_build_info(
                    root,
                    profile="heyuan;touch /tmp/no",
                    scene_id=2,
                    control_source="game",
                )
            with self.assertRaisesRegex(MODULE.BuildInfoError, "scene_id"):
                MODULE.collect_build_info(
                    root,
                    profile="heyuan",
                    scene_id=200,
                    control_source="game",
                )


if __name__ == "__main__":
    unittest.main()
