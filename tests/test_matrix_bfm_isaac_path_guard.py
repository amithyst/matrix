from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "matrix_bfm_isaac_path_guard.py"
SPEC = importlib.util.spec_from_file_location(
    "matrix_bfm_isaac_path_guard", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class MatrixBfmIsaacPathGuardTest(unittest.TestCase):
    def test_subtree_rejects_exact_descendant_dotdot_and_nonexistent_child(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protected = root / "matrix"
            protected.mkdir()
            cases = {
                "exact": protected,
                "descendant": protected / "outputs" / "run",
                "dotdot": root / "scratch" / ".." / "matrix" / "runtime",
                "nonexistent_child": protected / "not-created" / "child",
            }

            for label, candidate in cases.items():
                with self.subTest(label=label), self.assertRaisesRegex(
                    ValueError, "overlaps protected colleague tree"
                ):
                    MODULE.validate_path(candidate, protected, mode="subtree")

    def test_subtree_resolves_symlink_before_checking_nonexistent_tail(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protected = root / "matrix"
            protected.mkdir()
            alias = root / "matrix-link"
            alias.symlink_to(protected, target_is_directory=True)

            for candidate in (alias, alias / "not-created" / "child"):
                with self.subTest(candidate=candidate), self.assertRaisesRegex(
                    ValueError, "overlaps protected colleague tree"
                ):
                    MODULE.validate_path(candidate, protected, mode="subtree")

    def test_overlap_rejects_ancestor_but_subtree_mode_allows_it(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protected = root / "colleague" / "matrix"
            protected.mkdir(parents=True)
            ancestor = protected.parent

            with self.assertRaisesRegex(
                ValueError, "overlaps protected colleague tree"
            ):
                MODULE.validate_path(ancestor, protected, mode="overlap")
            self.assertEqual(
                MODULE.validate_path(ancestor, protected, mode="subtree"),
                ancestor.resolve(),
            )

    def test_prefix_sibling_is_not_mistaken_for_protected_tree(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            protected = root / "matrix"
            protected.mkdir()
            sibling = root / "matrix-clean-port" / "runtime"

            self.assertEqual(
                MODULE.validate_path(sibling, protected, mode="subtree"),
                sibling.resolve(),
            )
            self.assertEqual(
                MODULE.validate_path(sibling, protected, mode="overlap"),
                sibling.resolve(),
            )

    def test_unix_socket_path_enforces_linux_byte_limit(self) -> None:
        accepted = Path("/" + "a" * (MODULE.UNIX_SOCKET_PATH_MAX_BYTES - 1))
        rejected = Path("/" + "a" * MODULE.UNIX_SOCKET_PATH_MAX_BYTES)

        self.assertEqual(
            len(os.fsencode(accepted)), MODULE.UNIX_SOCKET_PATH_MAX_BYTES
        )
        self.assertEqual(MODULE.validate_unix_socket_path(accepted), accepted)
        with self.assertRaisesRegex(ValueError, "AF_UNIX path exceeds"):
            MODULE.validate_unix_socket_path(rejected)

    def test_unix_socket_path_counts_encoded_not_character_length(self) -> None:
        multibyte = Path("/" + "月" * 36)

        self.assertLess(len(os.fspath(multibyte)), MODULE.UNIX_SOCKET_PATH_MAX_BYTES)
        self.assertGreater(
            len(os.fsencode(multibyte)), MODULE.UNIX_SOCKET_PATH_MAX_BYTES
        )
        with self.assertRaisesRegex(ValueError, "AF_UNIX path exceeds"):
            MODULE.validate_unix_socket_path(multibyte)


if __name__ == "__main__":
    unittest.main()
