from __future__ import annotations

from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

TESTS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TESTS_DIR.parent
if os.fspath(TESTS_DIR) not in sys.path:
    sys.path.insert(0, os.fspath(TESTS_DIR))

import test_matrix_game_control_integration as INTEGRATION  # noqa: E402
from test_matrix_spawn_clearance import (  # noqa: E402
    Contact,
    FakeData,
    FakeModel,
    FakeMujoco,
    HORIZONTAL,
    MODULE as SPAWN_CLEARANCE,
    VERTICAL,
)


RUNTIME_SCRIPT = REPO_ROOT / "scripts/run_matrix_sonic.py"
WORLD_STATE = INTEGRATION.WORLD_STATE


class MatrixSonicShellRollbackTest(unittest.TestCase):
    def setUp(self) -> None:
        self.launcher_fixture = INTEGRATION.LauncherArgumentChainIntegrationTest()

    @staticmethod
    def _moon_no_ground_audits() -> tuple[dict[str, object], ...]:
        one_foot_model = FakeModel()
        one_foot_model.enable_moon_continuous_support()
        one_foot = SPAWN_CLEARANCE.audit_spawn_safety(
            FakeMujoco(
                default_support=False,
                ray_hits={(0.0, 3): (0.04, VERTICAL)},
            ),
            one_foot_model,
            FakeData(),
        )

        height_delta_model = FakeModel()
        height_delta_model.enable_moon_continuous_support()
        height_delta = SPAWN_CLEARANCE.audit_spawn_safety(
            FakeMujoco(
                default_support=False,
                ray_hits={
                    (0.0, 3): (0.02, VERTICAL),
                    (1.0, 3): (0.08, VERTICAL),
                },
            ),
            height_delta_model,
            FakeData(),
        )

        assert one_foot["support"]["required_hits"] == 2
        assert one_foot["support"]["rejection_reason"] == (
            "insufficient_distinct_foot_support"
        )
        assert height_delta["support"]["required_hits"] == 2
        assert height_delta["support"]["rejection_reason"] == (
            "foot_support_height_delta"
        )
        return one_foot, height_delta

    @staticmethod
    def _moon_contact_audits() -> tuple[dict[str, object], ...]:
        body_model = FakeModel()
        body_model.enable_moon_continuous_support()
        body_contact = SPAWN_CLEARANCE.audit_spawn_clearance(
            body_model,
            FakeData(Contact(2, 3, dist=0.0)),
        )

        foot_model = FakeModel()
        foot_model.nmocap = 1
        foot_model.body_mocapid = (-1, -1, -1, -1, -1, 0)
        foot_model._bodies[5] = "gb_0_0"
        foot_contact = SPAWN_CLEARANCE.audit_spawn_clearance(
            foot_model,
            FakeData(Contact(0, 4, dist=-0.004, frame=HORIZONTAL)),
        )

        assert body_contact["worst"]["classification"] == "unsafe_body_contact"
        assert foot_contact["worst"]["classification"] == (
            "unsafe_foot_terrain_edge"
        )
        return body_contact, foot_contact

    def _runtime_python_wrapper(self, project: Path, fake_python: Path) -> Path:
        wrapper = project / "fake-bin/runtime-python-with-validator"
        self.launcher_fixture.write(
            wrapper,
            f"""#!/usr/bin/python3
import os
import sys

if len(sys.argv) > 1 and sys.argv[1] == "-I":
    os.execv("/usr/bin/python3", ["/usr/bin/python3", *sys.argv[1:]])
os.execv({os.fspath(fake_python)!r}, [{os.fspath(fake_python)!r}, *sys.argv[1:]])
""",
            executable=True,
        )
        return wrapper

    def _install_runtime_validator_shim(self, project: Path) -> None:
        self.launcher_fixture.write(
            project / "scripts/run_matrix_sonic.py",
            f"""from runpy import run_path

_runtime = run_path({os.fspath(RUNTIME_SCRIPT)!r}, run_name="_matrix_runtime_validator")
_spawn_clearance_rollback_reason = _runtime["_spawn_clearance_rollback_reason"]
""",
        )

    def _proposal_status(
        self,
        *,
        audit: dict[str, object],
        state_file: Path,
        world_id: str,
        world_revision: str,
        checkpoint_id: str,
        generation: int,
        source: str,
        run_id: str,
    ) -> dict[str, object]:
        reason = audit["reason"]
        assert isinstance(reason, str)
        status = self.launcher_fixture.rollback_proposal_status(
            state_file=state_file,
            world_id=world_id,
            world_revision=world_revision,
            checkpoint_id=checkpoint_id,
            generation=generation,
            source=source,
            run_id=run_id,
            spawn_clearance_reason=reason,
            dynamic_resume_clearance=True,
        )
        status["spawn_clearance"] = audit
        probation = status["resume_probation"]
        assert isinstance(probation, dict)
        probation["failure_reason"] = reason
        probation["last_clearance_audit"] = audit
        return status

    def _run_launcher_case(
        self,
        audit: dict[str, object],
        *,
        expect_restart: bool,
        suffix: str,
        validator_available: bool = True,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.launcher_fixture.make_project(project)
            self._install_runtime_validator_shim(project)
            if not validator_available:
                self.launcher_fixture.write(
                    project / "scripts/run_matrix_sonic.py",
                    "raise RuntimeError('validator unavailable')\n",
                )
            runtime_python = self._runtime_python_wrapper(
                project,
                fixture["fake_python"],
            )
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            state_file = project / "state/world.json"
            world_id = "g1_29dof:scene_terrain_moon_dynamic"
            world_revision = f"moon-shell-rollback-{suffix}-v1"
            store = WORLD_STATE.WorldStateStore(
                state_file,
                world_id=world_id,
                world_revision=world_revision,
            )
            older_state = store.state.checkpoint(
                WORLD_STATE.WorldPose(1.0, 2.0, -0.13, 0.0),
                upright=True,
                now_unix_ns=1,
            )
            store.save(older_state)
            selected_state = older_state.checkpoint(
                WORLD_STATE.WorldPose(3.0, 4.0, -0.14, 0.2),
                upright=True,
                now_unix_ns=2,
            )
            store.save(selected_state)
            older = older_state.resolve_start()
            selected = selected_state.resolve_start()
            assert older.checkpoint_id is not None
            assert selected.checkpoint_id is not None

            status = self._proposal_status(
                audit=audit,
                state_file=state_file,
                world_id=world_id,
                world_revision=world_revision,
                checkpoint_id=selected.checkpoint_id,
                generation=selected.generation,
                source=selected.source,
                run_id=(suffix[0] * 32),
            )
            proposal_file = project / "proposal.json"
            proposal_file.write_text(
                json.dumps(status, allow_nan=False, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            generations = project / "generations.txt"
            self.launcher_fixture.write(
                project / "scripts/run_sim.sh",
                """#!/usr/bin/env bash
set -euo pipefail
generation=1
if [[ -f "${GENERATION_FILE:?}" ]]; then
    generation=$(( $(<"$GENERATION_FILE") + 1 ))
fi
printf '%s' "$generation" > "$GENERATION_FILE"
mkdir -p "$(dirname "${MATRIX_SONIC_STATUS_FILE:?}")"
if [[ "$generation" == "1" ]]; then
    cp "${ROLLBACK_PROPOSAL_FILE:?}" "$MATRIX_SONIC_STATUS_FILE"
    exit 76
fi
exit 0
""",
                executable=True,
            )
            primary_before = state_file.read_bytes()
            backup_before = store.backup_path.read_bytes()
            environment = {
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_GAME_WORLD_STATE_FILE": os.fspath(state_file),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(runtime_python),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_SONIC_STATUS_FILE": os.fspath(
                    project / "outputs/matrix-sonic-status.json"
                ),
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "ROLLBACK_PROPOSAL_FILE": os.fspath(proposal_file),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "15",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )

            if expect_restart:
                self.assertEqual(
                    result.returncode,
                    0,
                    msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                )
                self.assertEqual(generations.read_text(encoding="utf-8"), "2")
                committed = store.load()
                self.assertEqual(committed.generation, selected.generation + 1)
                self.assertEqual(
                    committed.resolve_start().checkpoint_id,
                    older.checkpoint_id,
                )
                self.assertEqual(
                    [item.checkpoint_id for item in committed.invalid_checkpoints],
                    [selected.checkpoint_id],
                )
                self.assertIn(
                    "Quarantined failed Matrix resume checkpoint",
                    result.stdout,
                )
            else:
                self.assertEqual(
                    result.returncode,
                    2,
                    msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                )
                self.assertEqual(generations.read_text(encoding="utf-8"), "1")
                self.assertEqual(state_file.read_bytes(), primary_before)
                self.assertEqual(store.backup_path.read_bytes(), backup_before)
                self.assertIn(
                    "Refusing unverified Matrix resume rollback proposal",
                    result.stderr,
                )

    def test_real_moon_audits_quarantine_then_restart(self) -> None:
        audits = (*self._moon_no_ground_audits(), *self._moon_contact_audits())
        expected = (
            ("no_ground_support", "insufficient_distinct_foot_support"),
            ("no_ground_support", "foot_support_height_delta"),
            ("scene_penetration", "unsafe_body_contact"),
            ("unsafe_foot_contact", "unsafe_foot_terrain_edge"),
        )
        for index, (audit, evidence) in enumerate(zip(audits, expected, strict=True)):
            reason, classification = evidence
            with self.subTest(reason=reason, classification=classification):
                self.assertEqual(audit["reason"], reason)
                if reason == "no_ground_support":
                    self.assertEqual(
                        audit["support"]["rejection_reason"],
                        classification,
                    )
                else:
                    self.assertEqual(
                        audit["worst"]["classification"],
                        classification,
                    )
                self._run_launcher_case(
                    audit,
                    expect_restart=True,
                    suffix=chr(ord("a") + index),
                )

    def test_unknown_real_audit_classification_fails_closed(self) -> None:
        body_audit, _foot_audit = self._moon_contact_audits()
        unknown = deepcopy(body_audit)
        unknown["worst"]["classification"] = "unknown_unsafe_body_contact"
        self._run_launcher_case(
            unknown,
            expect_restart=False,
            suffix="f",
        )

    def test_authoritative_validator_failure_fails_closed(self) -> None:
        body_audit, _foot_audit = self._moon_contact_audits()
        self._run_launcher_case(
            body_audit,
            expect_restart=False,
            suffix="e",
            validator_available=False,
        )


if __name__ == "__main__":
    unittest.main()
