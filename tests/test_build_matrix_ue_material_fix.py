from __future__ import annotations

import os
from pathlib import Path
import hashlib
import shutil
import subprocess
import tempfile
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build_matrix_ue_material_fix.sh"
BOOTSTRAP_SCRIPT = REPO_ROOT / "scripts" / "bootstrap_matrix_bfm_isaac.sh"


def write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


class MatrixUeMaterialFixBuildTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        scripts = self.root / "scripts"
        scripts.mkdir(parents=True)
        shutil.copy2(BUILD_SCRIPT, scripts / BUILD_SCRIPT.name)
        source = self.root / "src/ue_shims/matrix_ue_material_fix.c"
        source.parent.mkdir(parents=True)
        source.write_text("/* fixture */\n", encoding="utf-8")
        ue_binary = (
            self.root
            / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux/"
            "zsibot_mujoco_ue"
        )
        ue_binary.parent.mkdir(parents=True)
        ue_binary.write_bytes(b"fixture UE")

        self.fake_bin = self.root / "fake-bin"
        self.fake_bin.mkdir()
        self.compiler_log = self.root / "compiler.log"
        write_executable(
            self.fake_bin / "readelf",
            """#!/usr/bin/env bash
set -euo pipefail
case "$1" in
    -n) printf '    Build ID: 056e17b8675b1006\n' ;;
    -h) cat <<'EOF'
ELF Header:
  Class:                             ELF64
  Data:                              2's complement, little endian
  Type:                              DYN (Shared object file)
  Machine:                           Advanced Micro Devices X86-64
EOF
        ;;
    *) exit 64 ;;
esac
""",
        )
        write_executable(
            self.fake_bin / "strings",
            """#!/usr/bin/env bash
set -euo pipefail
if [[ "${MISSING_MARKER:-0}" != "1" ]]; then
    printf '%s\n' 'matrix-ue-material-fix: installed audited Matrix 0.1.2 material bridge'
fi
""",
        )
        write_executable(
            self.fake_bin / "fixture-cc",
            """#!/usr/bin/env bash
set -euo pipefail
printf 'called\n' >> "${COMPILER_LOG:?}"
output=""
while (($#)); do
    if [[ "$1" == "-o" ]]; then
        output="$2"
        break
    fi
    shift
done
[[ -n "$output" ]]
printf 'fixture shared object\n' > "$output"
""",
        )
        self.output = self.root / "runtime/libmatrix_ue_material_fix.so"
        self.environment = {
            **os.environ,
            "CC": "fixture-cc",
            "COMPILER_LOG": os.fspath(self.compiler_log),
            "PATH": os.fspath(self.fake_bin)
            + os.pathsep
            + os.environ.get("PATH", "/usr/bin:/bin"),
        }

    def run_build(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                "bash",
                os.fspath(self.root / "scripts" / BUILD_SCRIPT.name),
                "--output",
                os.fspath(self.output),
                *arguments,
            ],
            env=self.environment,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_build_is_atomic_and_verify_only_does_not_invoke_compiler(self) -> None:
        built = self.run_build()
        self.assertEqual(built.returncode, 0, built.stderr)
        self.assertTrue(self.output.is_file())
        self.assertTrue(os.access(self.output, os.X_OK))
        self.assertEqual(self.compiler_log.read_text(encoding="utf-8"), "called\n")

        original = self.output.read_bytes()
        expected_sha256 = hashlib.sha256(original).hexdigest()
        self.environment["CC"] = "compiler-must-not-be-used"
        verified = self.run_build(
            "--expected-sha256", expected_sha256, "--verify-only"
        )
        self.assertEqual(verified.returncode, 0, verified.stderr)
        self.assertEqual(self.output.read_bytes(), original)
        self.assertEqual(self.compiler_log.read_text(encoding="utf-8"), "called\n")

    def test_build_and_verify_only_reject_hash_mismatch_without_replacing_output(
        self,
    ) -> None:
        built = self.run_build()
        self.assertEqual(built.returncode, 0, built.stderr)
        original = self.output.read_bytes()

        rejected_verify = self.run_build(
            "--expected-sha256", "0" * 64, "--verify-only"
        )
        self.assertNotEqual(rejected_verify.returncode, 0)
        self.assertIn("SHA256 mismatch", rejected_verify.stderr)
        self.assertEqual(self.output.read_bytes(), original)

        rejected_build = self.run_build("--expected-sha256", "0" * 64)
        self.assertNotEqual(rejected_build.returncode, 0)
        self.assertIn("SHA256 mismatch", rejected_build.stderr)
        self.assertEqual(self.output.read_bytes(), original)

        invalid = self.run_build("--expected-sha256", "ABC")
        self.assertEqual(invalid.returncode, 2)
        self.assertIn("lowercase SHA256", invalid.stderr)

        wrong_build_id = self.run_build(
            "--expected-ue-build-id", "0" * 16, "--verify-only"
        )
        self.assertNotEqual(wrong_build_id.returncode, 0)
        self.assertIn("unsupported Matrix UE Build ID", wrong_build_id.stderr)

    def test_verify_only_rejects_missing_symlink_or_unmarked_output(self) -> None:
        missing = self.run_build("--verify-only")
        self.assertNotEqual(missing.returncode, 0)
        self.assertIn("regular executable file", missing.stderr)

        target = self.root / "target.so"
        target.write_bytes(b"fixture")
        target.chmod(0o755)
        self.output.parent.mkdir(parents=True)
        self.output.symlink_to(target)
        symlink = self.run_build("--verify-only")
        self.assertNotEqual(symlink.returncode, 0)
        self.assertIn("regular executable file", symlink.stderr)

        self.output.unlink()
        self.output.write_bytes(b"fixture")
        self.output.chmod(0o755)
        self.environment["MISSING_MARKER"] = "1"
        unmarked = self.run_build("--verify-only")
        self.assertNotEqual(unmarked.returncode, 0)
        self.assertIn("audited install marker", unmarked.stderr)

    def test_bfm_bootstrap_builds_or_verifies_the_bridge_by_mode(self) -> None:
        bootstrap = BOOTSTRAP_SCRIPT.read_text(encoding="utf-8")
        build_call = 'bash "$SCRIPT_DIR/build_matrix_ue_material_fix.sh"'
        self.assertGreaterEqual(bootstrap.count(build_call), 3)
        self.assertIn('--expected-sha256 "$MATERIAL_BRIDGE_SHA256"', bootstrap)
        self.assertIn('--expected-ue-build-id "$MATERIAL_UE_BUILD_ID"', bootstrap)
        self.assertIn('"${MATERIAL_BRIDGE_ARGS[@]}" --verify-only', bootstrap)
        self.assertLess(
            bootstrap.index('if [[ "$VERIFY_ONLY" == "1" ]]'),
            bootstrap.index('"${MATERIAL_BRIDGE_ARGS[@]}" --verify-only'),
        )


if __name__ == "__main__":
    unittest.main()
