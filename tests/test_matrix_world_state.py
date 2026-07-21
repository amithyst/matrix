import hashlib
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import matrix_world_state as MODULE  # noqa: E402
import prepare_sonic_physics_model as PHYSICS  # noqa: E402


class WorldPoseTest(unittest.TestCase):
    def test_pose_round_trip_normalizes_yaw(self) -> None:
        pose = MODULE.WorldPose(12, -4.5, 0.793, 3.0 * math.pi)

        self.assertAlmostEqual(pose.yaw_rad, math.pi)
        self.assertEqual(MODULE.WorldPose.from_mapping(pose.to_mapping()), pose)

    def test_pose_rejects_nonfinite_and_out_of_bounds_values(self) -> None:
        for values in (
            (math.nan, 0.0, 0.8, 0.0),
            (0.0, math.inf, 0.8, 0.0),
            (0.0, 0.0, 20_000.0, 0.0),
            (100_001.0, 0.0, 0.8, 0.0),
        ):
            with self.subTest(values=values), self.assertRaises(
                MODULE.WorldStateError
            ):
                MODULE.WorldPose(*values)


class MatrixWorldStateTest(unittest.TestCase):
    def setUp(self) -> None:
        self.default = MODULE.WorldPose(0.0, 0.0, 0.793, 0.0)
        self.state = MODULE.MatrixWorldState.empty(
            world_id="town10:scene_terrain_t10",
            world_revision="a" * 64,
        )

    def test_startup_fallback_is_last_exit_then_home_then_default(self) -> None:
        self.assertEqual(self.state.startup_pose(self.default), (self.default, "default"))

        home = MODULE.WorldPose(1.0, 2.0, 0.8, 0.25)
        with_home, _point = self.state.add_teleport_point(
            home,
            ("home",),
            entity_id="tp-" + "1" * 32,
            now_unix_ns=1,
        )
        self.assertEqual(with_home.startup_pose(self.default), (home, "home"))

        last_exit = MODULE.WorldPose(5.0, 6.0, 0.82, -0.5)
        resumed = with_home.set_resume_pose(last_exit, now_unix_ns=2)
        self.assertEqual(
            resumed.startup_pose(self.default), (last_exit, "last_exit")
        )

    def test_fallen_checkpoint_keeps_observed_xy_but_last_safe_upright_pose(self) -> None:
        safe = MODULE.WorldPose(10.0, 20.0, 0.81, 0.6)
        state = self.state.checkpoint(safe, upright=True, now_unix_ns=1)
        fallen = MODULE.WorldPose(10.8, 19.7, 0.18, -2.2)

        state = state.checkpoint(fallen, upright=False, now_unix_ns=2)

        self.assertEqual(state.last_observed, fallen)
        self.assertEqual(state.last_safe, safe)
        self.assertEqual(
            state.last_exit,
            MODULE.WorldPose(fallen.x, fallen.y, safe.z, safe.yaw_rad),
        )
        self.assertEqual(state.resume_source, "fallen_xy_last_safe_upright")

    def test_fallen_outlier_checkpoint_preserves_last_safe_resume_pose(self) -> None:
        safe = MODULE.WorldPose(10.0, 20.0, 0.81, 0.6)
        state = self.state.checkpoint(safe, upright=True, now_unix_ns=1)
        outlier = MODULE.WorldPose(4_000.0, -3_000.0, 200.0, -2.2)

        state = state.checkpoint(outlier, upright=False, now_unix_ns=2)

        self.assertEqual(state.last_observed, outlier)
        self.assertEqual(state.last_safe, safe)
        self.assertEqual(state.last_exit, safe)
        self.assertEqual(state.resume_source, "fallen_outlier_last_safe")
        self.assertEqual(state.startup_pose(self.default), (safe, "last_exit"))
        self.assertEqual(MODULE.MatrixWorldState.from_mapping(state.to_mapping()), state)

    def test_startup_rejects_legacy_exit_outlier_from_last_safe(self) -> None:
        safe = MODULE.WorldPose(10.0, 20.0, 0.81, 0.6)
        outlier = MODULE.WorldPose(4_000.0, -3_000.0, 0.81, 0.6)
        state = MODULE.MatrixWorldState(
            world_id="town10:scene_terrain_t10",
            world_revision="a" * 64,
            last_observed=outlier,
            last_safe=safe,
            last_exit=outlier,
            resume_source="fallen_xy_last_safe_upright",
            updated_at_unix_ns=2,
        )

        self.assertEqual(
            state.startup_pose(self.default),
            (safe, "last_safe_outlier_fallback"),
        )

    def test_fall_without_known_safe_pose_does_not_invent_resume_height(self) -> None:
        fallen = MODULE.WorldPose(3.0, 4.0, 0.15, 1.0)

        state = self.state.checkpoint(fallen, upright=False, now_unix_ns=1)

        self.assertEqual(state.last_observed, fallen)
        self.assertIsNone(state.last_exit)
        self.assertEqual(state.startup_pose(self.default), (self.default, "default"))

    def test_nearest_tag_selector_is_deterministic_and_limited(self) -> None:
        state, far = self.state.add_teleport_point(
            MODULE.WorldPose(20.0, 0.0, 0.8, 0.0),
            ("base",),
            entity_id="tp-" + "1" * 32,
            now_unix_ns=2,
        )
        state, near = state.add_teleport_point(
            MODULE.WorldPose(2.0, 0.0, 0.8, 0.0),
            ("base", "safe"),
            entity_id="tp-" + "2" * 32,
            now_unix_ns=3,
        )

        selected = state.select_teleport_points(
            tag="base", origin=self.default, sort="nearest", limit=1
        )

        self.assertEqual(selected, (near,))
        self.assertNotEqual(selected, (far,))

    def test_tags_are_unique_bounded_and_safe(self) -> None:
        state, point = self.state.add_teleport_point(
            self.default,
            ("Home", "Home", "checkpoint_1"),
            entity_id="tp-" + "3" * 32,
            now_unix_ns=1,
        )
        self.assertEqual(point.tags, ("Home", "checkpoint_1"))
        self.assertEqual(len(state.teleport_points), 1)
        for invalid in ("", "has space", "x" * 65, "中文"):
            with self.subTest(tag=invalid), self.assertRaises(
                MODULE.WorldStateError
            ):
                self.state.add_teleport_point(self.default, (invalid,))

    def test_mapping_parser_rejects_unknown_fields(self) -> None:
        mapping = self.state.to_mapping()
        mapping["unexpected"] = True
        with self.assertRaises(MODULE.WorldStateError):
            MODULE.MatrixWorldState.from_mapping(mapping)

    def test_json_decoder_rejects_duplicate_fields_and_nonfinite_constants(self) -> None:
        valid = json.dumps(self.state.to_mapping(), allow_nan=False)
        duplicate = valid[:-1] + ',"schema":"matrix-world-state/v1"}'
        nonfinite_mapping = self.state.to_mapping()
        nonfinite_mapping["updated_at_unix_ns"] = math.nan
        nonfinite = json.dumps(nonfinite_mapping)
        for payload in (duplicate.encode("utf-8"), nonfinite.encode("utf-8")):
            with self.subTest(payload=payload[:80]), self.assertRaises(
                MODULE.WorldStateError
            ):
                MODULE._decode_state_bytes(payload)


class WorldStateStoreTest(unittest.TestCase):
    def test_atomic_round_trip_permissions_and_backup_recovery(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "state" / "town10.json"
            store = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="b" * 64,
            )
            self.assertEqual(store.load(), store.state)
            self.assertEqual(store.load_status, "missing")
            first = store.state.checkpoint(
                MODULE.WorldPose(1.0, 2.0, 0.8, 0.1),
                upright=True,
                now_unix_ns=1,
            )
            store.save(first)
            second = first.checkpoint(
                MODULE.WorldPose(2.0, 3.0, 0.8, 0.2),
                upright=True,
                now_unix_ns=2,
            )
            store.save(second)

            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            self.assertEqual(
                stat.S_IMODE(path.parent.stat().st_mode) & 0o077,
                0,
            )
            path.write_text("{truncated", encoding="utf-8")
            recovered = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="b" * 64,
            )

            self.assertEqual(recovered.load(), first)
            self.assertEqual(recovered.load_status, "backup")
            self.assertIn("primary", recovered.load_error or "")

    def test_revision_mismatch_is_preserved_as_invalid_and_falls_back_empty(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "town10.json"
            old = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="old",
            )
            old.save()
            original_bytes = path.read_bytes()
            new = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="new",
            )

            loaded = new.load()

            self.assertEqual(new.load_status, "invalid")
            self.assertIsNone(loaded.last_exit)
            self.assertEqual(path.read_bytes(), original_bytes)

    def test_refuses_symlink_state_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target.json"
            target.write_text("{}", encoding="utf-8")
            link = root / "state.json"
            link.symlink_to(target)
            store = MODULE.WorldStateStore(
                link,
                world_id="town10",
                world_revision="revision",
            )
            store.load()
            self.assertEqual(store.load_status, "invalid")
            with self.assertRaises(MODULE.WorldStateError):
                store.save()

    def test_regular_file_open_does_not_rely_on_a_symlink_precheck(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target.json"
            target.write_bytes(b"guarded target")
            link = root / "state.json"
            link.symlink_to(target)

            with mock.patch.object(Path, "is_symlink", return_value=False):
                with self.assertRaises(MODULE.WorldStateError):
                    MODULE._read_regular_file(link)

            self.assertEqual(target.read_bytes(), b"guarded target")

    def test_parent_symlink_is_rejected_without_reading_or_writing_target(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / "target"
            target_path = target / "nested/state.json"
            direct = MODULE.WorldStateStore(
                target_path,
                world_id="town10",
                world_revision="revision",
            )
            protected = direct.state.checkpoint(
                MODULE.WorldPose(7.0, 8.0, 0.8, 0.4),
                upright=True,
                now_unix_ns=1,
            )
            direct.save(protected)
            protected_bytes = target_path.read_bytes()
            linked_parent = root / "linked-parent"
            linked_parent.symlink_to(target, target_is_directory=True)
            indirect = MODULE.WorldStateStore(
                linked_parent / "nested/state.json",
                world_id="town10",
                world_revision="revision",
            )

            loaded = indirect.load()

            self.assertEqual(indirect.load_status, "invalid")
            self.assertIsNone(loaded.last_exit)
            with self.assertRaises(MODULE.WorldStateError):
                indirect.save()
            self.assertEqual(target_path.read_bytes(), protected_bytes)

    def test_new_directories_and_final_rename_fsync_their_directories(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / "state-root"
            root.mkdir()
            path = root / "profile" / "world" / "state.json"
            store = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="revision",
            )
            real_fsync = os.fsync
            directory_fsyncs: list[tuple[int, int]] = []
            regular_fsyncs = 0

            def record_fsync(descriptor: int) -> None:
                nonlocal regular_fsyncs
                metadata = os.fstat(descriptor)
                if stat.S_ISDIR(metadata.st_mode):
                    directory_fsyncs.append((metadata.st_dev, metadata.st_ino))
                elif stat.S_ISREG(metadata.st_mode):
                    regular_fsyncs += 1
                real_fsync(descriptor)

            with mock.patch.object(
                MODULE.os, "fsync", side_effect=record_fsync
            ), mock.patch.object(
                MODULE.os,
                "chmod",
                side_effect=AssertionError("pathname chmod is unsafe"),
            ):
                store.save()

            expected_directories = [
                (item.stat().st_dev, item.stat().st_ino)
                for item in (root, root / "profile", root / "profile/world")
            ]
            self.assertEqual(directory_fsyncs, expected_directories)
            self.assertEqual(regular_fsyncs, 1)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)

    def test_parent_swap_after_open_cannot_redirect_atomic_write(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            stable_parent = root / "stable"
            stable_parent.mkdir()
            moved_parent = root / "opened-parent"
            attacker_target = root / "attacker-target"
            attacker_target.mkdir()
            path = stable_parent / "state.json"
            store = MODULE.WorldStateStore(
                path,
                world_id="town10",
                world_revision="revision",
            )
            real_open = os.open
            stable_open_count = 0

            def racing_open(
                item: str | os.PathLike[str],
                flags: int,
                mode: int = 0o777,
                *,
                dir_fd: int | None = None,
            ) -> int:
                nonlocal stable_open_count
                descriptor = real_open(item, flags, mode, dir_fd=dir_fd)
                if item == "stable" and flags & os.O_DIRECTORY:
                    stable_open_count += 1
                    if stable_open_count == 2:
                        stable_parent.rename(moved_parent)
                        stable_parent.symlink_to(
                            attacker_target,
                            target_is_directory=True,
                        )
                return descriptor

            with mock.patch.object(MODULE.os, "open", side_effect=racing_open):
                store.save()

            self.assertEqual(stable_open_count, 2)
            self.assertFalse((attacker_target / "state.json").exists())
            persisted = MODULE._decode_state_bytes(
                (moved_parent / "state.json").read_bytes()
            )
            self.assertEqual(persisted, store.state)

    def test_fifo_state_is_rejected_without_blocking(self) -> None:
        script = SCRIPTS / "matrix_world_state.py"
        with tempfile.TemporaryDirectory() as temporary:
            fifo = Path(temporary) / "state.json"
            os.mkfifo(fifo, 0o600)

            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "resolve-start",
                    "--file",
                    str(fifo),
                    "--world-id",
                    "town10",
                    "--world-revision",
                    "revision",
                ],
                text=True,
                capture_output=True,
                timeout=2.0,
                check=True,
            )

            self.assertEqual(completed.stdout.splitlines(), ["none", "invalid"])

    def test_save_does_not_replace_correct_backup_with_other_world_identity(self) -> None:
        for wrong_world_id, wrong_revision in (
            ("warehouse", "revision"),
            ("town10", "other-revision"),
        ):
            with self.subTest(
                world_id=wrong_world_id,
                world_revision=wrong_revision,
            ), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / "town10.json"
                store = MODULE.WorldStateStore(
                    path,
                    world_id="town10",
                    world_revision="revision",
                )
                first = store.state.checkpoint(
                    MODULE.WorldPose(1.0, 2.0, 0.8, 0.1),
                    upright=True,
                    now_unix_ns=1,
                )
                store.save(first)
                second = first.checkpoint(
                    MODULE.WorldPose(2.0, 3.0, 0.8, 0.2),
                    upright=True,
                    now_unix_ns=2,
                )
                store.save(second)
                backup = path.with_name(f"{path.name}.bak")
                correct_backup = backup.read_bytes()

                wrong = MODULE.MatrixWorldState.empty(
                    world_id=wrong_world_id,
                    world_revision=wrong_revision,
                )
                path.write_text(
                    json.dumps(wrong.to_mapping(), allow_nan=False),
                    encoding="utf-8",
                )
                third = second.checkpoint(
                    MODULE.WorldPose(3.0, 4.0, 0.8, 0.3),
                    upright=True,
                    now_unix_ns=3,
                )

                store.save(third)

                self.assertEqual(backup.read_bytes(), correct_backup)
                self.assertEqual(MODULE._decode_state_bytes(correct_backup), first)
                self.assertEqual(MODULE._decode_state_bytes(path.read_bytes()), third)

    def test_default_path_is_profile_and_world_scoped(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            old = os.environ.get("XDG_STATE_HOME")
            os.environ["XDG_STATE_HOME"] = temporary
            try:
                path = MODULE.default_world_state_path(
                    profile="heyuan", world_id="g1:scene_terrain_t10"
                )
            finally:
                if old is None:
                    os.environ.pop("XDG_STATE_HOME", None)
                else:
                    os.environ["XDG_STATE_HOME"] = old
            digest = hashlib.sha256(b"g1:scene_terrain_t10").hexdigest()[:32]
            self.assertEqual(
                path,
                Path(temporary)
                / "matrix"
                / "heyuan"
                / f"g1_scene_terrain_t10-{digest}.json",
            )

    def test_default_paths_do_not_collide_after_world_name_sanitizing(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.dict(os.environ, {"XDG_STATE_HOME": temporary}):
                colon = MODULE.default_world_state_path(
                    profile="heyuan", world_id="g1:town10"
                )
                slash = MODULE.default_world_state_path(
                    profile="heyuan", world_id="g1/town10"
                )

            self.assertNotEqual(colon, slash)
            self.assertEqual(colon.parent, slash.parent)
            self.assertTrue(colon.name.startswith("g1_town10-"))
            self.assertTrue(slash.name.startswith("g1_town10-"))

    def test_default_path_keeps_a_leaf_symlink_lexical_and_refuses_save(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with mock.patch.dict(os.environ, {"XDG_STATE_HOME": temporary}):
                slot = MODULE.default_world_state_path(
                    profile="heyuan", world_id="g1:town10"
                )
                slot.parent.mkdir(parents=True)
                target = root / "guarded-target.json"
                target.write_bytes(b"do not overwrite")
                slot.symlink_to(target)

                returned = MODULE.default_world_state_path(
                    profile="heyuan", world_id="g1:town10"
                )

            self.assertEqual(returned, slot)
            self.assertTrue(returned.is_symlink())
            store = MODULE.WorldStateStore(
                returned,
                world_id="g1:town10",
                world_revision="revision",
            )
            store.load()
            self.assertEqual(store.load_status, "invalid")
            with self.assertRaises(MODULE.WorldStateError):
                store.save()
            self.assertEqual(target.read_bytes(), b"do not overwrite")

    def test_revision_and_resolve_start_cli_are_stable_and_line_bounded(self) -> None:
        script = SCRIPTS / "matrix_world_state.py"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = root / "scene.xml"
            model = root / "robot.xml"
            meshes = root / "meshes"
            meshes.mkdir()
            (meshes / "body.stl").write_bytes(b"body")
            scene.write_text("<mujoco model='scene'/>", encoding="utf-8")
            model.write_text("<mujoco model='robot'/>", encoding="utf-8")
            revision = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "revision",
                    "--world-id",
                    "g1:town10",
                    "--native-scene",
                    str(scene),
                    "--canonical-model",
                    str(model),
                    "--canonical-meshes",
                    str(meshes),
                ],
                text=True,
                capture_output=True,
                check=True,
            ).stdout.strip()
            self.assertRegex(revision, r"^[0-9a-f]{64}$")
            path = root / "state.json"
            store = MODULE.WorldStateStore(
                path,
                world_id="g1:town10",
                world_revision=revision,
            )
            pose = MODULE.WorldPose(12.5, -3.0, 0.81, 0.25)
            store.save(store.state.set_resume_pose(pose, now_unix_ns=1))

            lines = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "resolve-start",
                    "--file",
                    str(path),
                    "--world-id",
                    "g1:town10",
                    "--world-revision",
                    revision,
                ],
                text=True,
                capture_output=True,
                check=True,
            ).stdout.splitlines()

            self.assertEqual(lines[0], "pose")
            self.assertEqual([float(value) for value in lines[1:5]], [12.5, -3.0, 0.81, 0.25])
            self.assertEqual(lines[5:], ["last_exit", "loaded"])

            safe = MODULE.WorldPose(10.0, 20.0, 0.81, 0.6)
            outlier = MODULE.WorldPose(4_000.0, -3_000.0, 0.81, 0.6)
            store.save(
                MODULE.MatrixWorldState(
                    world_id="g1:town10",
                    world_revision=revision,
                    last_observed=outlier,
                    last_safe=safe,
                    last_exit=outlier,
                    resume_source="fallen_xy_last_safe_upright",
                    updated_at_unix_ns=2,
                )
            )
            outlier_lines = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "resolve-start",
                    "--file",
                    str(path),
                    "--world-id",
                    "g1:town10",
                    "--world-revision",
                    revision,
                ],
                text=True,
                capture_output=True,
                check=True,
            ).stdout.splitlines()

            self.assertEqual(outlier_lines[0], "pose")
            self.assertEqual(
                [float(value) for value in outlier_lines[1:5]],
                [safe.x, safe.y, safe.z, safe.yaw_rad],
            )
            self.assertEqual(
                outlier_lines[5:],
                ["last_safe_outlier_fallback", "loaded"],
            )

    def test_revision_covers_location_independent_physics_source_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            def make_source(name: str) -> tuple[Path, Path, Path]:
                source = root / name
                meshes = source / "canonical-meshes"
                native_assets = source / "native/assets"
                meshes.mkdir(parents=True)
                native_assets.mkdir(parents=True)
                model = source / "canonical.xml"
                scene = source / "native/scene.xml"
                model.write_text("<mujoco model='robot'/>", encoding="utf-8")
                (meshes / "body.stl").write_bytes(b"canonical mesh")
                (native_assets / "terrain.bin").write_bytes(b"native asset")
                scene.write_text(
                    "<mujoco><asset><mesh name='terrain' "
                    "file='terrain.bin'/></asset></mujoco>",
                    encoding="utf-8",
                )
                return model, meshes, scene

            model_a, meshes_a, scene_a = make_source("host-a")
            model_b, meshes_b, scene_b = make_source("host-b")

            def revision(model: Path, meshes: Path, scene: Path) -> str:
                return MODULE.world_revision_for_files(
                    world_id="g1:town10",
                    native_scene=scene,
                    canonical_model=model,
                    canonical_meshes=meshes,
                )

            baseline = revision(model_a, meshes_a, scene_a)
            self.assertEqual(baseline, revision(model_b, meshes_b, scene_b))

            payload = PHYSICS.physics_revision_payload(
                model_a,
                meshes_a,
                scene_a,
            )
            serialized = json.dumps(payload, sort_keys=True)
            self.assertNotIn(str(root), serialized)
            self.assertNotIn("spawn_xyz", payload)
            self.assertNotIn("spawn_yaw_rad", payload)

            (meshes_b / "body.stl").write_bytes(b"changed canonical mesh")
            self.assertNotEqual(baseline, revision(model_b, meshes_b, scene_b))
            (meshes_b / "body.stl").write_bytes(b"canonical mesh")

            (scene_b.parent / "assets/terrain.bin").write_bytes(
                b"changed native asset"
            )
            self.assertNotEqual(baseline, revision(model_b, meshes_b, scene_b))
            (scene_b.parent / "assets/terrain.bin").write_bytes(b"native asset")

            with mock.patch.object(
                PHYSICS,
                "PIPELINE_VERSION",
                PHYSICS.PIPELINE_VERSION + 1,
            ):
                self.assertNotEqual(
                    baseline,
                    revision(model_b, meshes_b, scene_b),
                )

    def test_revision_records_scene_transform_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            meshes = root / "canonical-meshes"
            native = root / "native"
            meshes.mkdir()
            native.mkdir()
            model = root / "canonical.xml"
            scene = native / "scene_terrain_t10.xml"
            model.write_text("<mujoco model='robot'/>", encoding="utf-8")
            (meshes / "body.stl").write_bytes(b"canonical mesh")
            scene.write_text(
                """<mujoco><worldbody>
<geom name="floor" size="0 0 0.01" type="plane" />
<geom name="ps_Cube" type="box" size="125.0 0.05 1.5" pos="0.9 72.6 1.5" quat="1 0 0 0" />
<geom name="ps_Cube2" type="box" size="125.0 0.05 1.5" pos="0.9 -125.7 1.5" quat="1 0 0 0" />
<geom name="ps_Cube3" type="box" size="125.0 0.05 1.5" pos="104.4 -21.6 1.5" quat="0.707107 0 0 -0.707107" />
<geom name="ps_Cube4" type="box" size="125.0 0.05 1.5" pos="-109.0 -21.6 1.5" quat="0.707107 0 0 -0.707107" />
</worldbody></mujoco>""",
                encoding="utf-8",
            )

            with mock.patch.object(
                PHYSICS,
                "TOWN10_SOURCE_SCENE_SHA256",
                PHYSICS._file_sha256(scene),
            ):
                default_revision = MODULE.world_revision_for_files(
                    world_id="g1:town10",
                    native_scene=scene,
                    canonical_model=model,
                    canonical_meshes=meshes,
                )
                transformed_revision = MODULE.world_revision_for_files(
                    world_id="g1:town10",
                    native_scene=scene,
                    canonical_model=model,
                    canonical_meshes=meshes,
                    scene_transform=PHYSICS.TOWN10_OPEN_BOUNDARY_TRANSFORM,
                )
                payload = PHYSICS.physics_revision_payload(
                    model,
                    meshes,
                    scene,
                    scene_transform=PHYSICS.TOWN10_OPEN_BOUNDARY_TRANSFORM,
                )

            self.assertNotEqual(default_revision, transformed_revision)
            self.assertEqual(
                payload["scene_transform"],
                PHYSICS.TOWN10_OPEN_BOUNDARY_TRANSFORM,
            )
            self.assertEqual(
                payload["removed_environment_geoms"],
                list(PHYSICS.TOWN10_PERIMETER_WALL_NAMES),
            )


if __name__ == "__main__":
    unittest.main()
