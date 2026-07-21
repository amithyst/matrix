from __future__ import annotations

import importlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import socket
import stat
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = REPO_ROOT / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

CORE = importlib.import_module("matrix_game_control")
PROVIDER = importlib.import_module("matrix_game_control_input")
OVERLAY = importlib.import_module("matrix_ue_overlay")
RUNTIME_SPEC = importlib.util.spec_from_file_location(
    "matrix_game_control_integration_runtime",
    SCRIPTS / "run_matrix_sonic.py",
)
assert RUNTIME_SPEC is not None and RUNTIME_SPEC.loader is not None
RUNTIME = importlib.util.module_from_spec(RUNTIME_SPEC)
RUNTIME_SPEC.loader.exec_module(RUNTIME)


def immediate_config() -> object:
    return CORE.ControlConfig(
        max_speed_mps=0.3,
        max_acceleration_mps2=100.0,
        max_deceleration_mps2=100.0,
        max_turn_rate_rad_s=100.0,
        max_step_s=1.0,
    )


def provider_snapshot(
    *,
    sequence: int,
    timestamp: float,
    w: bool = False,
    camera_yaw_rad: float = 0.0,
) -> object:
    return PROVIDER.build_snapshot(
        sequence=sequence,
        timestamp_monotonic_s=timestamp,
        keyboard=PROVIDER.KeyboardMouseSample(w=w, focused=True),
        gamepad=PROVIDER.GamepadSample(),
        input_source="keyboard",
        camera_yaw_rad=camera_yaw_rad,
        camera_available=True,
    )


@unittest.skipUnless(
    hasattr(socket, "SOCK_SEQPACKET") and hasattr(socket, "SO_PEERCRED"),
    "Linux Unix seqpacket credentials are required",
)
class GameControlPipelineIntegrationTest(unittest.TestCase):
    @staticmethod
    def planner_client(planner_frames: list[dict[str, object]]):
        class FakeSocket:
            def setsockopt(self, *_args) -> None:
                pass

            def bind(self, _endpoint: str) -> None:
                pass

            def send(self, _payload: bytes) -> None:
                pass

            def close(self, **_kwargs) -> None:
                pass

        fake_socket = FakeSocket()

        class FakeContext:
            @classmethod
            def instance(cls):
                return cls()

            def socket(self, _kind: int):
                return fake_socket

        return RUNTIME.NativePlannerClient(
            "tcp://127.0.0.1:5556",
            zmq_module=SimpleNamespace(Context=FakeContext, PUB=1, LINGER=2),
            build_command_message=lambda **_kwargs: b"command",
            build_planner_message=lambda **kwargs: (
                planner_frames.append(kwargs) or b"planner"
            ),
        )

    def test_runtime_rejects_same_uid_peer_with_wrong_supervised_pid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "game.sock"
            expected_pid = os.getpid() + 100_000
            runtime = RUNTIME.GameInputRuntime(
                socket_path,
                CORE.GameControlCore(immediate_config()),
                expected_peer_pid=expected_pid,
            )
            runtime.open()
            client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                client.connect(os.fspath(socket_path))
                command = runtime.poll(now_s=10.0, dt_s=0.1)
                self.assertTrue(command.safe_stop)
                self.assertEqual(command.reason, "peer_pid_mismatch")
                telemetry = runtime.telemetry(now_s=10.0)
                self.assertEqual(telemetry["accepted_connections"], 0)
                self.assertEqual(telemetry["peer_pid_mismatches"], 1)
                self.assertEqual(telemetry["expected_peer_pid"], expected_pid)
                self.assertIsNone(telemetry["peer_pid"])
            finally:
                client.close()
                runtime.close()

    def test_provider_to_seqpacket_to_core_to_planner_and_eof_stop(self) -> None:
        planner_frames: list[dict[str, object]] = []
        planner = self.planner_client(planner_frames)
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "game.sock"
            runtime = RUNTIME.GameInputRuntime(
                socket_path,
                CORE.GameControlCore(immediate_config()),
            )
            runtime.open()
            publisher = PROVIDER.UnixSeqpacketPublisher(socket_path)
            try:
                neutral = provider_snapshot(sequence=100, timestamp=10.0)
                self.assertTrue(publisher.send(neutral, now=10.0))
                idle = runtime.poll(now_s=10.0, dt_s=0.1)
                self.assertEqual(idle.mode, "idle")

                moving_snapshot = provider_snapshot(
                    sequence=101,
                    timestamp=10.01,
                    w=True,
                    camera_yaw_rad=math.pi / 2.0,
                )
                self.assertTrue(publisher.send(moving_snapshot, now=10.01))
                gait_entry = runtime.poll(now_s=10.01, dt_s=0.1)
                self.assertEqual(gait_entry.mode, "move")
                self.assertAlmostEqual(gait_entry.speed_mps, 0.1)
                moving = runtime.poll(now_s=10.011, dt_s=0.1)
                self.assertEqual(moving.mode, "move")
                planner.send_game_command(moving)
                runtime.record_published_command(moving)

                self.assertEqual(planner_frames[-1]["mode"], 2)
                # Unmodified keyboard WASD reaches native WALK's lower bound.
                self.assertAlmostEqual(planner_frames[-1]["speed"], 0.8)
                self.assertAlmostEqual(
                    planner_frames[-1]["movement"][0], 0.0, places=7
                )
                self.assertAlmostEqual(
                    planner_frames[-1]["movement"][1], 1.0, places=7
                )
                self.assertEqual(
                    planner_frames[-1]["movement"],
                    planner_frames[-1]["facing"],
                )
                telemetry = runtime.telemetry(now_s=10.011)
                self.assertEqual(telemetry["moving_command_frames"], 1)
                self.assertEqual(telemetry["locomotion_mode"], 2)
                self.assertEqual(telemetry["locomotion_mode_name"], "WALK")

                publisher.close()
                stopped = runtime.poll(now_s=10.012, dt_s=0.001)
                self.assertEqual(stopped.mode, "deadman")
                self.assertTrue(stopped.safe_stop)
                self.assertEqual(stopped.reason, "peer_closed")
                self.assertEqual(stopped.speed_mps, 0.0)
                planner.send_game_command(stopped)
                self.assertEqual(planner_frames[-1]["mode"], 0)
                self.assertEqual(planner_frames[-1]["movement"], [0.0, 0.0, 0.0])
                self.assertEqual(planner_frames[-1]["speed"], -1.0)
                self.assertEqual(runtime.telemetry(now_s=10.012)["disconnects"], 1)
            finally:
                publisher.close()
                runtime.close()
        with mock.patch.object(RUNTIME.time, "sleep"):
            planner.close()

    def test_reconnected_peer_held_w_requires_neutral_rearm(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "game.sock"
            runtime = RUNTIME.GameInputRuntime(
                socket_path,
                CORE.GameControlCore(immediate_config()),
            )
            runtime.open()
            first = PROVIDER.UnixSeqpacketPublisher(socket_path)
            second = PROVIDER.UnixSeqpacketPublisher(socket_path)
            try:
                self.assertTrue(
                    first.send(
                        provider_snapshot(sequence=100, timestamp=10.0),
                        now=10.0,
                    )
                )
                self.assertEqual(runtime.poll(now_s=10.0, dt_s=0.1).mode, "idle")
                self.assertTrue(
                    first.send(
                        provider_snapshot(
                            sequence=101,
                            timestamp=10.01,
                            w=True,
                        ),
                        now=10.01,
                    )
                )
                self.assertEqual(runtime.poll(now_s=10.01, dt_s=0.1).mode, "move")

                first.close()
                eof_stop = runtime.poll(now_s=10.011, dt_s=0.001)
                self.assertEqual(eof_stop.reason, "peer_closed")
                self.assertEqual(eof_stop.speed_mps, 0.0)

                self.assertTrue(
                    second.send(
                        provider_snapshot(
                            sequence=200,
                            timestamp=10.02,
                            w=True,
                        ),
                        now=10.02,
                    )
                )
                held_on_connect = runtime.poll(now_s=10.02, dt_s=0.1)
                self.assertEqual(held_on_connect.reason, "awaiting_neutral")
                self.assertEqual(held_on_connect.speed_mps, 0.0)

                self.assertTrue(
                    second.send(
                        provider_snapshot(sequence=201, timestamp=10.03),
                        now=10.03,
                    )
                )
                self.assertEqual(runtime.poll(now_s=10.03, dt_s=0.1).mode, "idle")
                self.assertTrue(
                    second.send(
                        provider_snapshot(
                            sequence=202,
                            timestamp=10.04,
                            w=True,
                        ),
                        now=10.04,
                    )
                )
                self.assertEqual(runtime.poll(now_s=10.04, dt_s=0.1).mode, "move")
                telemetry = runtime.telemetry(now_s=10.04)
                self.assertEqual(telemetry["accepted_connections"], 2)
                self.assertEqual(telemetry["disconnects"], 1)
            finally:
                first.close()
                second.close()
                runtime.close()

    def test_faulty_packet_batch_cannot_rearm_with_later_neutral_and_w(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            socket_path = Path(temporary) / "game.sock"
            runtime = RUNTIME.GameInputRuntime(
                socket_path,
                CORE.GameControlCore(immediate_config()),
            )
            runtime.open()
            client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            try:
                client.connect(os.fspath(socket_path))
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=100,
                    timestamp=10.0,
                )))
                self.assertEqual(runtime.poll(now_s=10.0, dt_s=0.1).mode, "idle")
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=101,
                    timestamp=10.01,
                    w=True,
                )))
                self.assertEqual(runtime.poll(now_s=10.01, dt_s=0.1).mode, "move")

                # All three packets are drained in one poll. The later neutral
                # and W must not erase the malformed packet's hard-stop frame.
                client.send(b"{}")
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=102,
                    timestamp=10.011,
                )))
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=103,
                    timestamp=10.012,
                    w=True,
                )))
                stopped = runtime.poll(now_s=10.012, dt_s=0.001)
                self.assertTrue(stopped.safe_stop)
                self.assertEqual(stopped.reason, "protocol_error")
                self.assertEqual(stopped.speed_mps, 0.0)
                self.assertEqual(runtime.telemetry(now_s=10.012)["protocol_errors"], 1)

                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=104,
                    timestamp=10.013,
                    w=True,
                )))
                held = runtime.poll(now_s=10.013, dt_s=0.001)
                self.assertEqual(held.reason, "awaiting_neutral")
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=105,
                    timestamp=10.014,
                )))
                self.assertEqual(runtime.poll(now_s=10.014, dt_s=0.001).mode, "idle")
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=106,
                    timestamp=10.015,
                    w=True,
                )))
                self.assertEqual(runtime.poll(now_s=10.015, dt_s=0.1).mode, "move")

                # A replay rejection has the same batch-level stop invariant.
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=106,
                    timestamp=10.016,
                )))
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=107,
                    timestamp=10.016,
                )))
                client.send(CORE.encode_input_packet(provider_snapshot(
                    sequence=108,
                    timestamp=10.017,
                    w=True,
                )))
                rejected = runtime.poll(now_s=10.017, dt_s=0.001)
                self.assertEqual(rejected.reason, "input_rejected")
                self.assertEqual(rejected.speed_mps, 0.0)
                self.assertEqual(runtime.rejected_packets, 1)
            finally:
                client.close()
                runtime.close()


class LauncherArgumentChainIntegrationTest(unittest.TestCase):
    GAME_CONTROL_DEPENDENCIES = (
        "matrix_game_control_input.py",
        "matrix_calibration_overlay.py",
        "matrix_mc_commands.py",
        "matrix_world_state.py",
        "prepare_sonic_physics_model.py",
        "compose_custom_scene.py",
    )

    @staticmethod
    def write(path: Path, contents: str = "", *, executable: bool = False) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")
        if executable:
            path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def make_project(self, project: Path) -> dict[str, Path]:
        scripts = project / "scripts"
        scripts.mkdir(parents=True)
        for name in (
            "run_matrix_sonic.sh",
            "run_sim.sh",
            "matrix_mouse_settings.py",
            "matrix_restart_request.py",
            "matrix_ue_overlay.py",
            "matrix_calibration_overlay.py",
            "matrix_mc_commands.py",
            "matrix_world_state.py",
            "compose_custom_scene.py",
            "prepare_sonic_physics_model.py",
        ):
            shutil.copy2(SCRIPTS / name, scripts / name)
        self.write(
            scripts / "matrix_local_env.sh",
            "load_matrix_local_env() { return 0; }\n",
        )
        for name in (
            "matrix_game_control.py",
            "matrix_game_control_input.py",
            "run_matrix_sonic.py",
            "supervise_matrix_ue.py",
        ):
            self.write(scripts / name, "# integration fixture\n")

        lock = project / "config/runtime/matrix-sonic.lock.json"
        lock.parent.mkdir(parents=True)
        shutil.copy2(REPO_ROOT / "config/runtime/matrix-sonic.lock.json", lock)
        shutil.copy2(
            REPO_ROOT / "config/runtime/matrix-centered-camera-overlay-v3.json",
            project / "config/runtime/matrix-centered-camera-overlay-v3.json",
        )
        self.write(
            project / "config/config.json",
            json.dumps(
                {
                    "robot": {
                        "position": {"x": 0, "y": 0, "z": 0},
                        "mujoco_running": True,
                    }
                }
            ),
        )
        self.write(
            project / "src/robot_mujoco/simulate/config.yaml",
            'robot_scene: "old.xml"\nrobot: "xgb"\n',
        )
        (project / "src/robot_mujoco/simulate/build").mkdir(parents=True)
        self.write(
            project / "src/robot_mc/run_mc.sh",
            "#!/usr/bin/env bash\nexport ROBOT_TYPE=XG\n",
            executable=True,
        )
        self.write(
            project / "src/robot_mc/build/export/config/xg-user-parameters.yaml",
            "motor_platform_type: 8\n",
        )
        self.write(
            project
            / "src/robot_mc/build/export/config/xg_wheel-user-parameters.yaml",
            "motor_platform_type: 8\n",
        )
        self.write(
            project
            / "src/robot_mc/build/export/config/zg_wheels-user-parameters.yaml",
            "motor_platform_type: 8\n",
        )
        self.write(project / "scene/scene.json", "{}\n")
        scene_xml = "<mujoco model=\"fixture\"/>\n"
        for robot_type in ("go2", "go2w", "xgb", "xgw", "zgws"):
            self.write(
                project
                / "src/robot_mujoco/zsibot_robots"
                / robot_type
                / "scene_terrain_apart2.xml",
                scene_xml,
            )
            self.write(
                project
                / "src/robot_mujoco/zsibot_robots"
                / robot_type
                / f"{robot_type}.xml",
                '<mujoco><worldbody><body name="base_link" pos="0 0 0"/></worldbody></mujoco>\n',
            )
            self.write(
                project
                / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model"
                / robot_type
                / "scene_terrain_apart2.xml",
                scene_xml,
            )
        self.write(
            project / "src/UeSim/Linux/zsibot_mujoco_ue.sh",
            "#!/usr/bin/env bash\nexit 0\n",
            executable=True,
        )
        self.write(
            project
            / "src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux"
            / "zsibot_mujoco_ue",
            "fixture UE executable\n",
            executable=True,
        )

        sonic = project / "fake-sonic"
        for relative in (
            "gear_sonic/scripts/run_sim_loop.py",
            "gear_sonic/utils/mujoco_sim/base_sim.py",
            "gear_sonic/utils/teleop/zmq/zmq_planner_sender.py",
            "gear_sonic_deploy/target/release/g1_deploy_onnx_ref",
            "gear_sonic_deploy/thirdparty/unitree_sdk2/lib/x86_64/libunitree_sdk2.a",
            "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml",
        ):
            self.write(sonic / relative, "fixture\n")
        (sonic / "gear_sonic/data/robot_model/model_data/g1/meshes").mkdir(
            parents=True
        )

        custom_urdf = project / "fixture/g1.urdf"
        self.write(custom_urdf, '<robot name="g1"/>\n')
        capture = project / "capture.json"
        ue_capture = project / "ue-capture.json"
        stale_status = project / "outputs/stale-game-input.json"
        self.write(stale_status, '{"stale": true}\n')

        fake_bin = project / "fake-bin"
        fake_bin.mkdir()
        self.write(
            fake_bin / "pkill",
            "#!/usr/bin/env bash\nexit 0\n",
            executable=True,
        )
        self.write(
            fake_bin / "xset",
            """#!/usr/bin/env bash
set -euo pipefail
if [[ "${XSET_FAIL_QUERY:-0}" == "1" && "${1:-}" == "q" ]]; then
    exit 1
fi
printf '%s\\n' "$*" >> "${XSET_LOG:?}"
case "${1:-}" in
    q)
        read -r acceleration threshold < "${XSET_STATE_FILE:?}"
        printf 'Pointer Control:\\n  acceleration:  %s    threshold:  %s\\n' \
            "$acceleration" "$threshold"
        ;;
    m)
        printf '%s %s\\n' "${2:?}" "${3:?}" > "${XSET_STATE_FILE:?}"
        ;;
    *)
        exit 2
        ;;
esac
""",
            executable=True,
        )
        self.write(
            fake_bin / "cmp",
            """#!/usr/bin/env bash
if [[ "${FAIL_CONFIG_RESTORE:-0}" == "1" ]]; then
    exit 1
fi
if [[ -n "${SIGNAL_LAUNCHER_DURING_RESTORE:-}" \
    && -n "${SIGNAL_RESTORE_MARKER:-}" \
    && ! -e "$SIGNAL_RESTORE_MARKER" ]]; then
    mkdir "$SIGNAL_RESTORE_MARKER"
    kill "-$SIGNAL_LAUNCHER_DURING_RESTORE" "$PPID"
    sleep 0.2
fi
exec /usr/bin/cmp "$@"
""",
            executable=True,
        )
        self.write(
            fake_bin / "jq",
            """#!/usr/bin/python3
import json
from pathlib import Path
import sys

args = sys.argv[1:]
source = Path(args[-1])
payload = json.loads(source.read_text(encoding="utf-8"))
if "-r" in args:
    expression = args[args.index("-r") + 1]
    if expression == ".robot.position.x":
        print(payload["robot"]["position"]["x"])
    elif expression == ".robot.position.y":
        print(payload["robot"]["position"]["y"])
    else:
        print("")
else:
    print(json.dumps(payload))
""",
            executable=True,
        )
        fake_python = fake_bin / "runtime-python"
        self.write(
            fake_python,
            """#!/usr/bin/python3
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import time

script = Path(sys.argv[1]).name
args = sys.argv[2:]

if script == "-":
    # run_sim uses the locked runtime Python to merge a late UE failure into
    # the already-written SONIC status.  The fixture mirrors that narrow helper
    # without importing the placeholder runtime module.
    status_path = Path(args[0])
    failure_path = Path(args[1])
    payload = json.loads(status_path.read_text(encoding="utf-8"))
    failure = json.loads(failure_path.read_text(encoding="utf-8"))
    label = f"native_child_exit:{failure['name']}:{failure['exit_code']}"
    failures = payload.setdefault("acceptance_failures", [])
    if label not in failures:
        failures.append(label)
    payload["failed_child_name"] = failure["name"]
    payload["failed_child_exit_code"] = failure["exit_code"]
    payload["pre_external_termination_reason"] = payload.get("termination_reason")
    payload["termination_reason"] = "child_exit"
    payload["passed"] = False
    payload["completed"] = False
    status_path.write_text(json.dumps(payload), encoding="utf-8")
elif script == "matrix_world_state.py":
    os.execv(
        "/usr/bin/python3",
        ["/usr/bin/python3", "-I", sys.argv[1], *args],
    )
elif script == "compose_custom_scene.py":
    target = Path(args[1])
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args[0], target)
elif script == "prepare_sonic_physics_model.py":
    output = Path(args[args.index("--output-dir") + 1])
    native = Path(args[args.index("--native-scene") + 1])
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(native, output / native.name)
elif script == "supervise_matrix_ue.py":
    separator = args.index("--")
    ue_capture = {"command": args[separator + 1:]}
    project_root = Path(
        os.environ.get(
            "MATRIX_PROJECT_ROOT", Path(os.environ["UE_CAPTURE_PATH"]).parent
        )
    )
    overlay_active = (
        project_root
        / "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive"
    )
    ue_capture["overlay_active_at_start"] = overlay_active.is_dir()
    ue_capture["overlay_inventory_at_start"] = (
        sorted(path.name for path in overlay_active.iterdir())
        if overlay_active.is_dir()
        else []
    )
    pointer_state = os.environ.get("XSET_STATE_FILE")
    if pointer_state:
        ue_capture["pointer_state_at_start"] = Path(pointer_state).read_text(
            encoding="utf-8"
        ).strip()
    Path(os.environ["UE_CAPTURE_PATH"]).write_text(
        json.dumps(ue_capture), encoding="utf-8"
    )
    if any(
        value.startswith("LD_PRELOAD=") for value in ue_capture["command"]
    ) and os.environ.get("FAKE_UE_MATERIAL_FIX_LOG") != "missing":
        with Path(args[args.index("--log") + 1]).open(
            "a", encoding="utf-8"
        ) as log_stream:
            log_stream.write(
                "matrix-ue-material-fix: installed audited Matrix 0.1.2 "
                "material bridge\\n"
            )
    overlay_log_mode = os.environ.get("FAKE_UE_OVERLAY_LOG", "")
    if overlay_log_mode:
        stem = "pakchunk99-MatrixCentered-Linux_P"
        lines = "LogPakFile: Display: Found Pak file " + stem + ".pak attempting to mount\\n"
        if overlay_log_mode == "failed":
            lines += "LogPakFile: Error: Failed to mount " + stem + ".utoc\\n"
        elif overlay_log_mode == "spoof":
            lines = "NotLogPakFile: Display: Found Pak file " + stem + ".pak attempting to mount\\n"
            lines += "NotLogPakFile: Display: Mounted IoStore container " + stem + ".utoc\\n"
            lines += "LogPakFile: Display: Found Pak file " + stem + ".pak.bad attempting to mount\\n"
            lines += "LogPakFile: Display: Mounted IoStore container " + stem + ".utoc.bad\\n"
        else:
            lines += "LogPakFile: Display: Mounted IoStore container " + stem + ".utoc\\n"
        with Path(args[args.index("--log") + 1]).open(
            "a", encoding="utf-8"
        ) as log_stream:
            log_stream.write(lines)
    pid_file = Path(args[args.index("--pid-file") + 1])
    pid_file.write_text(str(os.getpid()), encoding="utf-8")
    for line in sys.stdin:
        if line.strip() == "stop":
            break
    late_ue_exit = os.environ.get("FAKE_UE_LATE_FAILURE_EXIT_CODE")
    if late_ue_exit:
        failure_file = Path(args[args.index("--failure-file") + 1])
        failure_file.write_text(
            json.dumps({"name": "ue", "exit_code": int(late_ue_exit)}),
            encoding="utf-8",
        )
        raise SystemExit(int(late_ue_exit))
elif script == "run_matrix_sonic.py":
    socket_path = Path(args[args.index("--game-input-socket") + 1])
    status_path = Path(os.environ["MATRIX_GAME_INPUT_STATUS_FILE"])
    capture = {
        "argv": args,
        "control_source_env": os.environ.get("MATRIX_SONIC_CONTROL_SOURCE"),
        "input_source_env": os.environ.get("MATRIX_GAME_INPUT_SOURCE"),
        "yaw_source_env": os.environ.get("MATRIX_GAME_CAMERA_YAW_SOURCE"),
        "socket_env": os.environ.get("MATRIX_GAME_INPUT_SOCKET"),
        "world_persistence_env": os.environ.get("MATRIX_GAME_WORLD_PERSISTENCE"),
        "auto_respawn_env": os.environ.get("MATRIX_GAME_AUTO_RESPAWN"),
        "socket_parent_mode": oct(socket_path.parent.stat().st_mode & 0o777),
        "stale_status_existed": status_path.exists(),
    }
    Path(os.environ["CAPTURE_PATH"]).write_text(
        json.dumps(capture), encoding="utf-8"
    )
    generation_file = os.environ.get("GENERATION_FILE")
    if generation_file:
        generation_path = Path(generation_file)
        generation = int(generation_path.read_text()) + 1 if generation_path.exists() else 1
        generation_path.write_text(str(generation), encoding="utf-8")
    internal_reason = os.environ.get("FAKE_WORLD_INTERNAL_RESTART")
    if internal_reason:
        sonic_status = Path(args[args.index("--status-file") + 1])
        sonic_status.write_text(
            json.dumps(
                {
                    "acceptance_failures": [],
                    "completed": False,
                    "failed_child_exit_code": None,
                    "failed_child_name": None,
                    "game_auto_respawn": os.environ.get(
                        "MATRIX_GAME_AUTO_RESPAWN"
                    ) == "1",
                    "game_world_state": {
                        "has_last_exit": True,
                        "last_error": None,
                    },
                    "internal_restart": {
                        "requested": True,
                        "reason": internal_reason,
                    },
                    "passed": False,
                    "termination_reason": internal_reason,
                    "termination_signal": None,
                }
            ),
            encoding="utf-8",
        )
        raise SystemExit(75)
    marker_value = os.environ.get("TRIGGER_RESTART_MARKER")
    if marker_value and not Path(marker_value).exists():
        Path(marker_value).write_text("requested", encoding="utf-8")
        os.environ["FAKE_SONIC_STATUS_FILE"] = args[
            args.index("--status-file") + 1
        ]
        provider = Path(args[args.index("--game-input-provider") + 1])
        os.execv(
            sys.argv[0],
            [
                sys.argv[0],
                str(provider),
                "--restart-request-file",
                args[args.index("--game-restart-request-file") + 1],
                "--restart-capability-file",
                args[args.index("--game-restart-capability-file") + 1],
                "--restart-launcher-pid",
                args[args.index("--game-restart-launcher-pid") + 1],
            ],
        )
elif script == "matrix_game_control_input.py":
    request = Path(args[args.index("--restart-request-file") + 1])
    capability = Path(args[args.index("--restart-capability-file") + 1])
    launcher_pid = int(args[args.index("--restart-launcher-pid") + 1])
    payload = {
        "version": 1,
        "action": "restart-whole-runtime",
        "launcher_pid": launcher_pid,
        "provider_pid": os.getpid(),
        "nonce": capability.read_text(encoding="ascii").strip(),
    }
    temporary = request.with_name(f".{request.name}.{os.getpid()}")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    temporary.chmod(0o600)
    os.replace(temporary, request)
    def stop_provider(*_args):
        delay = float(os.environ.get("FAKE_PROVIDER_TERM_DELAY_SECONDS", "0"))
        if delay > 0.0:
            time.sleep(delay)
        marker = os.environ.get("FAKE_PROVIDER_TERM_MARKER")
        if marker:
            observed_path = os.environ.get("FAKE_PROVIDER_OBSERVE_PATH")
            observed = (
                Path(observed_path).read_text(encoding="utf-8")
                if observed_path
                else "term-complete"
            )
            Path(marker).write_text(observed, encoding="utf-8")
        sonic_status_file = os.environ.get("FAKE_SONIC_STATUS_FILE")
        if sonic_status_file:
            checkpoint_error = os.environ.get("FAKE_FINAL_CHECKPOINT_ERROR")
            Path(sonic_status_file).write_text(
                json.dumps(
                    {
                        "acceptance_failures": (
                            ["world_state_checkpoint_failed"]
                            if checkpoint_error
                            else []
                        ),
                        "completed": False,
                        "failed_child_exit_code": None,
                        "failed_child_name": None,
                        "game_world_state": {
                            "has_last_exit": True,
                            "last_error": checkpoint_error,
                        },
                        "internal_restart": {
                            "requested": False,
                            "reason": None,
                        },
                        "passed": False,
                        "termination_reason": "signal",
                        "termination_signal": signal.SIGTERM,
                    }
                ),
                encoding="utf-8",
            )
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop_provider)
    while True:
        time.sleep(0.1)
else:
    raise SystemExit(f"unexpected fake runtime invocation: {script}")
""",
            executable=True,
        )
        return {
            "capture": capture,
            "ue_capture": ue_capture,
            "custom_urdf": custom_urdf,
            "fake_bin": fake_bin,
            "fake_python": fake_python,
            "sonic": sonic,
            "stale_status": stale_status,
        }

    def test_outer_game_dependency_preflight_applies_with_persistence_off(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_GAME_CENTERED_CAMERA": "off",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_matrix_sonic.sh"),
                "--scene",
                "21",
                "--control-source",
                "game",
                "--game-world-persistence",
                "off",
                "--game-auto-respawn",
                "off",
            ]

            for dependency in self.GAME_CONTROL_DEPENDENCIES:
                with self.subTest(dependency=dependency):
                    path = project / "scripts" / dependency
                    held = path.with_name(f".{path.name}.missing")
                    path.rename(held)
                    fixture["capture"].unlink(missing_ok=True)
                    try:
                        result = subprocess.run(
                            command,
                            env={
                                **environment,
                                "MATRIX_SONIC_HOST_LOCK": os.fspath(
                                    project / f"launcher-missing-{dependency}.lock"
                                ),
                            },
                            text=True,
                            capture_output=True,
                            timeout=20.0,
                            check=False,
                        )
                    finally:
                        held.rename(path)
                    self.assertEqual(
                        result.returncode,
                        1,
                        msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                    )
                    self.assertIn(
                        f"Matrix game-control dependency is missing: {path}",
                        result.stderr,
                    )
                    self.assertFalse(fixture["capture"].exists())

    def test_run_sim_game_dependency_preflight_applies_with_persistence_off(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_DISABLE_MC": "1",
                "MATRIX_GAME_AUTO_RESPAWN": "0",
                "MATRIX_GAME_CENTERED_CAMERA": "off",
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    project / "outputs/game-input.json"
                ),
                "MATRIX_GAME_NO_INPUT_PROVIDER": "1",
                "MATRIX_GAME_WORLD_PERSISTENCE": "0",
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC": "1",
                "MATRIX_SONIC_CONTROL_SOURCE": "game",
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UNITREE_SDK2_ROOT": os.fspath(
                    fixture["sonic"] / "gear_sonic_deploy/thirdparty/unitree_sdk2"
                ),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_sim.sh"),
                "xgb",
                "21",
                "0",
                "0",
                "1",
            ]

            for dependency in self.GAME_CONTROL_DEPENDENCIES:
                with self.subTest(dependency=dependency):
                    path = project / "scripts" / dependency
                    held = path.with_name(f".{path.name}.missing")
                    path.rename(held)
                    fixture["capture"].unlink(missing_ok=True)
                    try:
                        result = subprocess.run(
                            command,
                            env=environment,
                            text=True,
                            capture_output=True,
                            timeout=20.0,
                            check=False,
                        )
                    finally:
                        held.rename(path)
                    self.assertEqual(
                        result.returncode,
                        1,
                        msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                    )
                    self.assertIn(
                        f"Matrix game-control dependency is missing: {path}",
                        result.stderr,
                    )
                    self.assertFalse(fixture["capture"].exists())

    def test_bounded_game_launcher_rejects_input_provider_bypass(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            self.make_project(project)
            self.write(project / "config/hosts/test.env", "\n")
            environment = {
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_GAME_NO_INPUT_PROVIDER": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--profile",
                    "test",
                    "--control-source",
                    "game",
                    "--game-camera-yaw-source",
                    "x11-mirror",
                    "--max-seconds",
                    "1",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn(
                "cannot disable the supervised input provider",
                result.stderr,
            )

            environment["MATRIX_GAME_NO_INPUT_PROVIDER"] = "0"
            environment["MATRIX_GAME_INPUT_PYTHON"] = "/bin/false"
            interpreter_result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--profile",
                    "test",
                    "--control-source",
                    "game",
                    "--game-camera-yaw-source",
                    "x11-mirror",
                    "--max-seconds",
                    "1",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(interpreter_result.returncode, 2)
            self.assertIn(
                "rejects MATRIX_GAME_INPUT_PYTHON",
                interpreter_result.stderr,
            )

    def test_bounded_qualification_auto_disables_persistent_world_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            self.make_project(project)
            self.write(project / "config/hosts/test.env", "\n")
            environment = {
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_matrix_sonic.sh"),
                "--profile",
                "test",
                "--control-source",
                "game",
                "--game-camera-yaw-source",
                "x11-mirror",
                "--max-seconds",
                "1",
            ]

            automatic = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(automatic.returncode, 2)
            self.assertIn(
                "Bounded qualification cannot disable runtime verification",
                automatic.stderr,
            )
            self.assertNotIn(
                "rejects persistent world state",
                automatic.stderr,
            )

            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-forced-persistence.lock"
            )
            forced = subprocess.run(
                [*command, "--game-world-persistence", "on"],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(forced.returncode, 2)
            self.assertIn(
                "Bounded qualification rejects persistent world state",
                forced.stderr,
            )

    def test_bounded_launcher_rejects_experimental_camera_sources(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            self.make_project(project)
            self.write(project / "config/hosts/test.env", "\n")
            environment = {
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "PATH": os.environ.get("PATH", "/usr/bin:/bin"),
            }
            for source in ("x11-core-gated", "x11-absolute", "ue-final-pov"):
                with self.subTest(source=source):
                    result = subprocess.run(
                        [
                            "/bin/bash",
                            os.fspath(project / "scripts/run_matrix_sonic.sh"),
                            "--profile",
                            "test",
                            "--control-source",
                            "game",
                            "--game-camera-yaw-source",
                            source,
                            "--max-seconds",
                            "1",
                        ],
                        env=environment,
                        text=True,
                        capture_output=True,
                        timeout=20.0,
                        check=False,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertIn(
                        "rejects experimental camera yaw sources",
                        result.stderr,
                    )

    def test_real_launcher_and_run_sim_forward_all_game_arguments(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            mouse_settings = project / "home/.config/matrix/mouse-control.json"
            xset_log = project / "xset.log"
            xset_state = project / "xset.state"
            material_fix = project / "outputs/runtime/matrix-ue-material-fix/libmatrix_ue_material_fix.so"
            self.write(
                mouse_settings,
                json.dumps(
                    {"version": 1, "profile": "remote", "speed_scale": 0.01}
                ),
            )
            self.write(xset_log)
            self.write(xset_state, "2/1 4\n")
            self.write(material_fix, "fixture material fix\n")
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "DISPLAY": ":fixture",
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    fixture["stale_status"]
                ),
                "MATRIX_MOUSE_SETTINGS_FILE": os.fspath(mouse_settings),
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_G1_MATERIAL_PALETTE": (
                    "0.018,0.024,0.035;0.055,0.075,0.11;"
                    "0.9,0.94,1;0.015,0.2,0.95"
                ),
                "MATRIX_G1_MATERIAL_SCOPE_ALPHA": "0.99609375",
                "MATRIX_SKIP_ENV_CHECK": "1",
                # The fixture must not contend with a real Matrix runtime on
                # the host that happens to execute this integration test.
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_EXTRA_EXEC_CMDS": (
                    "set Engine.SpringArmComponent bEnableCameraLag True,"
                    "viewclass OperatorCamera_C"
                ),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XSET_LOG": os.fspath(xset_log),
                "XSET_STATE_FILE": os.fspath(xset_state),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_matrix_sonic.sh"),
                "--scene",
                "21",
                "--skin",
                "matrix-blue",
                "--control-source",
                "game",
                "--game-input-source",
                "keyboard",
                "--game-camera-yaw-source",
                "x11-mirror",
                "--game-look-button",
                "right",
                "--game-initial-yaw",
                "12.5",
                "--game-mouse-sensitivity",
                "0.25",
                "--game-camera-yaw-sign",
                "1",
                "--game-camera-yaw-offset",
                "-90.0",
                "--game-carla-host",
                "127.0.0.2",
                "--game-carla-port",
                "2100",
                "--gamepad-look-yaw-rate",
                "140.0",
                "--gamepad-look-pitch-rate",
                "95.0",
                "--gamepad-look-deadzone",
                "0.13",
                "--gamepad-look-min-pitch",
                "-70.0",
                "--gamepad-look-max-pitch",
                "50.0",
                "--game-max-speed",
                "0.27",
                "--game-input-timeout",
                "0.14",
            ]
            result = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            capture = json.loads(fixture["capture"].read_text(encoding="utf-8"))
            self.assertEqual(capture["control_source_env"], "game")
            self.assertEqual(capture["input_source_env"], "keyboard")
            self.assertEqual(capture["yaw_source_env"], "x11-mirror")
            self.assertEqual(capture["world_persistence_env"], "1")
            self.assertEqual(capture["auto_respawn_env"], "1")
            self.assertEqual(capture["socket_parent_mode"], "0o700")
            self.assertFalse(capture["stale_status_existed"])
            ue_capture = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )
            self.assertEqual(ue_capture["pointer_state_at_start"], "1/1 0")
            self.assertEqual(xset_state.read_text(encoding="utf-8"), "2/1 4\n")
            self.assertEqual(
                xset_log.read_text(encoding="utf-8").splitlines(),
                ["q", "m 1/1 0", "m 2/1 4"],
            )
            self.assertIn(
                "SDL_MOUSE_RELATIVE_SPEED_SCALE=0.010000",
                ue_capture["command"],
            )
            self.assertIn(
                f"LD_PRELOAD={material_fix.resolve()}",
                ue_capture["command"],
            )
            self.assertIn(
                "MATRIX_G1_SKIN=matrix-blue",
                ue_capture["command"],
            )
            self.assertIn(
                "MATRIX_G1_MATERIAL_PALETTE="
                "0.018,0.024,0.035;0.055,0.075,0.11;"
                "0.9,0.94,1;0.015,0.2,0.95",
                ue_capture["command"],
            )
            self.assertIn(
                "MATRIX_G1_MATERIAL_SCOPE_ALPHA=0.99609375",
                ue_capture["command"],
            )
            for direct_hint in (
                "SDL_MOUSE_RELATIVE_MODE_WARP=0",
                "SDL_MOUSE_RELATIVE_SCALING=0",
                "SDL_MOUSE_RELATIVE_SYSTEM_SCALE=0",
            ):
                self.assertIn(direct_hint, ue_capture["command"])
            self.assertIn(
                "-ini:Input:[/Script/Engine.InputSettings]:"
                "bEnableMouseSmoothing=False,[/Script/Engine.InputSettings]:"
                "bEnableFOVScaling=False",
                ue_capture["command"],
            )
            self.assertIn(
                "-ExecCmds=t.MaxFPS 30,r.MotionBlurQuality 0,"
                "set Engine.SpringArmComponent bEnableCameraLag False,"
                "set Engine.SpringArmComponent bEnableCameraRotationLag False,"
                "set Engine.SpringArmComponent bDoCollisionTest True,"
                "viewclass MujocoSim_Custom_C,"
                "set Engine.SpringArmComponent bEnableCameraLag True,"
                "viewclass OperatorCamera_C",
                ue_capture["command"],
            )

            with mock.patch.object(
                sys,
                "argv",
                ["run_matrix_sonic.py", *capture["argv"]],
            ):
                parsed = RUNTIME._parse_args()
            self.assertEqual(parsed.control_source, "game")
            self.assertEqual(parsed.game_input_source, "keyboard")
            self.assertEqual(parsed.game_camera_yaw_source, "x11-mirror")
            self.assertEqual(parsed.game_look_button, "right")
            self.assertEqual(parsed.game_initial_camera_yaw_deg, 12.5)
            self.assertEqual(parsed.game_mouse_sensitivity_deg, 0.25)
            self.assertEqual(parsed.game_applied_mouse_profile, "remote")
            self.assertEqual(parsed.game_applied_mouse_speed_scale, 0.01)
            self.assertEqual(parsed.game_mouse_settings_file, mouse_settings)
            self.assertIsNotNone(parsed.game_restart_request_file)
            self.assertIsNotNone(parsed.game_restart_capability_file)
            self.assertGreater(parsed.game_restart_launcher_pid, 1)
            self.assertEqual(parsed.game_camera_yaw_sign, 1)
            self.assertEqual(parsed.game_camera_yaw_offset_deg, -90.0)
            self.assertEqual(parsed.game_carla_host, "127.0.0.2")
            self.assertEqual(parsed.game_carla_port, 2100)
            self.assertEqual(parsed.gamepad_look_yaw_rate_deg_s, 140.0)
            self.assertEqual(parsed.gamepad_look_pitch_rate_deg_s, 95.0)
            self.assertEqual(parsed.gamepad_look_deadzone, 0.13)
            self.assertEqual(parsed.gamepad_look_min_pitch_deg, -70.0)
            self.assertEqual(parsed.gamepad_look_max_pitch_deg, 50.0)
            self.assertEqual(parsed.game_max_speed, 0.27)
            self.assertEqual(parsed.game_input_timeout, 0.14)
            self.assertEqual(
                parsed.game_world_id,
                "g1_29dof:scene_terrain_apart2",
            )
            self.assertRegex(parsed.game_world_revision, r"^[0-9a-f]{64}$")
            self.assertEqual(
                parsed.game_world_state_file,
                project
                / "home/.local/state/matrix/local/"
                "g1_29dof_scene_terrain_apart2-"
                "c1ddf02ac13d294fcb07af591694e3fb.json",
            )
            self.assertEqual(parsed.game_world_checkpoint_seconds, 0.75)
            self.assertTrue(parsed.game_auto_respawn)
            self.assertFalse(parsed.fail_on_fall)
            self.assertIsNotNone(parsed.ue_pid)
            self.assertGreater(parsed.ue_pid, 1)
            self.assertEqual(parsed.game_max_snapshot_age, 0.15)
            self.assertEqual(
                parsed.game_input_provider,
                project / "scripts/matrix_game_control_input.py",
            )
            self.assertEqual(
                parsed.game_input_provider_python,
                os.fspath(fixture["fake_python"]),
            )
            self.assertEqual(parsed.game_input_status_file, fixture["stale_status"])
            self.assertEqual(os.fspath(parsed.game_input_socket), capture["socket_env"])
            self.assertFalse(parsed.game_input_socket.parent.exists())

            # X11 setup is an experience improvement, never a launch gate.
            # A headless launch and an unreachable X server both continue with
            # an explicit warning and without changing the recorded state.
            xset_log.write_text("", encoding="utf-8")
            xset_state.write_text("2/1 4\n", encoding="utf-8")
            no_display_environment = dict(environment)
            no_display_environment.pop("DISPLAY")
            no_display_environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-no-display.lock"
            )
            no_display = subprocess.run(
                command,
                env=no_display_environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                no_display.returncode,
                0,
                msg=f"stdout:\n{no_display.stdout}\nstderr:\n{no_display.stderr}",
            )
            self.assertIn("because DISPLAY is unset; continuing", no_display.stderr)
            self.assertEqual(xset_log.read_text(encoding="utf-8"), "")
            self.assertEqual(xset_state.read_text(encoding="utf-8"), "2/1 4\n")

            xset_log.write_text("", encoding="utf-8")
            failed_query_environment = dict(environment)
            failed_query_environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-xset-failure.lock"
            )
            failed_query_environment["XSET_FAIL_QUERY"] = "1"
            failed_query = subprocess.run(
                command,
                env=failed_query_environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                failed_query.returncode,
                0,
                msg=(
                    f"stdout:\n{failed_query.stdout}\n"
                    f"stderr:\n{failed_query.stderr}"
                ),
            )
            self.assertIn(
                "could not read X pointer acceleration", failed_query.stderr
            )
            self.assertEqual(xset_log.read_text(encoding="utf-8"), "")
            self.assertEqual(xset_state.read_text(encoding="utf-8"), "2/1 4\n")

    def test_corrupt_mouse_settings_fall_back_to_explicit_local_one_x(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            corrupt = project / "home/.config/matrix/mouse-control.json"
            self.write(corrupt, "{")
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_MOUSE_SETTINGS_FILE": os.fspath(corrupt),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            capture = json.loads(fixture["capture"].read_text(encoding="utf-8"))
            with mock.patch.object(
                sys, "argv", ["run_matrix_sonic.py", *capture["argv"]]
            ):
                parsed = RUNTIME._parse_args()
            self.assertEqual(parsed.game_applied_mouse_profile, "local")
            self.assertEqual(parsed.game_applied_mouse_speed_scale, 1.0)
            ue_capture = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )
            self.assertIn(
                "SDL_MOUSE_RELATIVE_SPEED_SCALE=1.000000",
                ue_capture["command"],
            )
            self.assertIn("using Local 1.00x", result.stderr)

    def test_centered_overlay_is_mounted_for_custom_game_lifetime_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            bundle = project / "centered-bundle"
            bundle.mkdir()
            contents = {
                f"{OVERLAY.STEM}.pak": b"fixture-pak\n",
                f"{OVERLAY.STEM}.utoc": b"fixture-utoc\n",
                f"{OVERLAY.STEM}.ucas": b"fixture-ucas\n",
            }
            for name, data in contents.items():
                (bundle / name).write_bytes(data)
            self.write(
                project / "scripts/matrix_ue_overlay.py",
                """#!/usr/bin/python3
import json
import os
from pathlib import Path
import shutil
import sys

command = sys.argv[1]
args = sys.argv[2:]
def option(name):
    return Path(args[args.index(name) + 1])

active_relative = Path(
    "src/UeSim/Linux/zsibot_mujoco_ue/Saved/Paks/MatrixCenteredCameraActive"
)
expected = {
    "pakchunk99-MatrixCentered-Linux_P.pak",
    "pakchunk99-MatrixCentered-Linux_P.utoc",
    "pakchunk99-MatrixCentered-Linux_P.ucas",
}
payload = {"action": command, "version": 3}
if command == "verify-bundle":
    bundle = option("--bundle")
    if {path.name for path in bundle.iterdir()} != expected:
        raise SystemExit(2)
    payload["bundle"] = str(bundle)
elif command == "purge-stale":
    active = option("--project-root") / active_relative
    payload["purged"] = int(active.exists())
    if active.exists():
        shutil.rmtree(active)
elif command == "install":
    active = option("--project-root") / active_relative
    shutil.copytree(option("--bundle"), active)
    payload["active"] = str(active)
elif command == "remove":
    active = option("--project-root") / active_relative
    if os.environ.get("FAKE_OVERLAY_REMOVE_FAIL") == "1":
        print("[ERROR] injected overlay remove failure", file=sys.stderr)
        raise SystemExit(2)
    payload["removed"] = active.exists()
    if active.exists():
        shutil.rmtree(active)
else:
    raise SystemExit(2)
print(json.dumps(payload, sort_keys=True))
""",
            )
            active = project / OVERLAY.RUNTIME_DIRECTORY
            active.mkdir(parents=True)
            for name, data in contents.items():
                (active / name).write_bytes(data)
            self.assertTrue(active.is_dir())
            ue_log = project / "src/UeSim/Linux/zsibot_mujoco_ue.log"
            self.write(
                ue_log,
                "LogPakFile: Display: Found Pak file "
                f"{OVERLAY.STEM}.pak attempting to mount\n"
                "LogPakFile: Display: Mounted IoStore container "
                f"{OVERLAY.STEM}.utoc\n"
                f"LogPakFile: Error: Failed historical {OVERLAY.STEM}.utoc\n",
            )

            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "FAKE_UE_OVERLAY_LOG": "1",
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_CENTERED_CAMERA_OVERLAY_BUNDLE": os.fspath(bundle),
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    fixture["stale_status"]
                ),
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_OVERLAY_MOUNT_TIMEOUT_SECONDS": "0",
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            ue_capture = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )
            self.assertTrue(ue_capture["overlay_active_at_start"])
            self.assertEqual(
                ue_capture["overlay_inventory_at_start"], sorted(contents)
            )
            exec_cmds = next(
                argument
                for argument in ue_capture["command"]
                if argument.startswith("-ExecCmds=")
            )
            self.assertIn("viewclass Spectator_C", exec_cmds)
            self.assertNotIn("viewclass MujocoSim_Custom_C", exec_cmds)
            self.assertIn(
                "set Engine.SpringArmComponent TargetArmLength 150", exec_cmds
            )
            self.assertFalse(active.exists())
            self.assertEqual(
                {path.name: path.read_bytes() for path in bundle.iterdir()}, contents
            )

            purge_index = result.stdout.index('"action": "purge-stale"')
            verify_index = result.stdout.index('"action": "verify-bundle"')
            install_index = result.stdout.index('"action": "install"')
            ue_index = result.stdout.index("[INFO] UE PID")
            mounted_index = result.stdout.index(
                "Verified Matrix centered-camera IoStore mount"
            )
            remove_index = result.stdout.index('"action": "remove"')
            self.assertLess(purge_index, verify_index)
            self.assertLess(verify_index, install_index)
            self.assertLess(install_index, ue_index)
            self.assertLess(ue_index, mounted_index)
            self.assertLess(mounted_index, remove_index)

            material_fix = project / "libmatrix_ue_material_fix.so"
            self.write(material_fix, "invalid fixture library\n")
            with ue_log.open("a", encoding="utf-8") as stream:
                stream.write(
                    "matrix-ue-material-fix: installed audited Matrix 0.1.2 "
                    "material bridge\n"
                )
            fixture["ue_capture"].unlink(missing_ok=True)
            environment["MATRIX_UE_MATERIAL_FIX_PRELOAD"] = os.fspath(
                material_fix
            )
            environment["MATRIX_G1_SKIN"] = "unitree-stock"
            environment["MATRIX_G1_MATERIAL_PALETTE"] = (
                "0.018,0.024,0.035;0.055,0.075,0.11;"
                "0.9,0.94,1;0.42,0.42,0.42"
            )
            environment["MATRIX_G1_MATERIAL_SCOPE_ALPHA"] = "0.99609375"
            environment["FAKE_UE_MATERIAL_FIX_LOG"] = "missing"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-missing-material-marker.lock"
            )
            environment["MATRIX_UE_MATERIAL_FIX_PRELOAD"] = os.fspath(material_fix)
            missing_material_marker = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(missing_material_marker.returncode, 1)
            self.assertIn(
                "did not emit its current-run installation marker",
                missing_material_marker.stderr,
            )
            self.assertFalse(active.exists())
            environment.pop("MATRIX_UE_MATERIAL_FIX_PRELOAD")
            environment.pop("MATRIX_G1_SKIN")
            environment.pop("MATRIX_G1_MATERIAL_PALETTE")
            environment.pop("MATRIX_G1_MATERIAL_SCOPE_ALPHA")
            environment.pop("FAKE_UE_MATERIAL_FIX_LOG")

            for invalid_distance in ("79", "501", "150,quit", "1e2"):
                with self.subTest(invalid_distance=invalid_distance):
                    fixture["ue_capture"].unlink(missing_ok=True)
                    environment["MATRIX_GAME_CAMERA_DISTANCE_CM"] = invalid_distance
                    environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                        project / f"launcher-distance-{invalid_distance}.lock"
                    )
                    rejected = subprocess.run(
                        [
                            "/bin/bash",
                            os.fspath(project / "scripts/run_matrix_sonic.sh"),
                            "--scene",
                            "21",
                            "--control-source",
                            "game",
                        ],
                        env=environment,
                        text=True,
                        capture_output=True,
                        timeout=20.0,
                        check=False,
                    )
                    self.assertEqual(rejected.returncode, 1)
                    self.assertIn(
                        "MATRIX_GAME_CAMERA_DISTANCE_CM must be a plain",
                        rejected.stderr,
                    )
                    self.assertFalse(fixture["ue_capture"].exists())
                    self.assertFalse(active.exists())

            fixture["ue_capture"].unlink(missing_ok=True)
            environment["MATRIX_GAME_CAMERA_DISTANCE_CM"] = "150"
            environment["FAKE_UE_OVERLAY_LOG"] = "failed"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-current-log-failure.lock"
            )
            failed_mount = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(failed_mount.returncode, 1)
            self.assertIn("UE reported Failed", failed_mount.stderr)
            self.assertTrue(fixture["ue_capture"].exists())
            self.assertFalse(active.exists())

            fixture["ue_capture"].unlink()
            environment["FAKE_UE_OVERLAY_LOG"] = "spoof"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-spoof-log.lock"
            )
            spoofed_mount = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(spoofed_mount.returncode, 1)
            self.assertIn(
                "did not confirm Found and Mounted IoStore",
                spoofed_mount.stderr,
            )
            self.assertFalse(active.exists())

            fixture["ue_capture"].unlink()
            environment["FAKE_UE_OVERLAY_LOG"] = "1"
            environment["FAKE_OVERLAY_REMOVE_FAIL"] = "1"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-remove-failure.lock"
            )
            failed_remove = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(failed_remove.returncode, 1)
            self.assertIn("Matrix cleanup failed", failed_remove.stderr)
            self.assertTrue(active.exists())

            fixture["ue_capture"].unlink()
            environment.pop("FAKE_OVERLAY_REMOVE_FAIL")
            environment["FAKE_UE_OVERLAY_LOG"] = "1"
            environment["MATRIX_GAME_CAMERA_VIEW_CLASS"] = "MujocoSim_Custom_C"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-overlay-viewclass.lock"
            )
            rejected_viewclass = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(rejected_viewclass.returncode, 1)
            self.assertIn(
                "overlay viewclass must be Spectator_C or unset",
                rejected_viewclass.stderr,
            )
            self.assertFalse(fixture["ue_capture"].exists())
            self.assertFalse(active.exists())

    def test_launcher_source_preserves_overlay_audit_and_cleanup_order(self) -> None:
        outer = (SCRIPTS / "run_matrix_sonic.sh").read_text(encoding="utf-8")
        self.assertLess(
            outer.index("export MATRIX_SONIC_HOST_LOCK_FD=9"),
            outer.index("purge-stale"),
        )
        self.assertLess(
            outer.index("purge-stale"),
            outer.index("verify_matrix_sonic_runtime.py"),
        )

        inner = (SCRIPTS / "run_sim.sh").read_text(encoding="utf-8")
        cleanup = inner[inner.index("cleanup() {") : inner.index("handle_signal() {")]
        self.assertLess(
            cleanup.index("stop_supervised_ue"),
            cleanup.index("remove_centered_camera_overlay"),
        )
        launch = inner[inner.index("# 启动流程") :]
        self.assertLess(
            launch.index("install_centered_camera_overlay"),
            launch.index("start_supervised_ue"),
        )
        self.assertLess(
            launch.index("start_supervised_ue"),
            launch.index("verify_centered_camera_overlay_mount"),
        )

    def test_centered_camera_can_be_disabled_or_strictly_overridden(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_GAME_CENTERED_CAMERA": "off",
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_matrix_sonic.sh"),
                "--scene",
                "21",
                "--control-source",
                "game",
            ]
            disabled = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                disabled.returncode,
                0,
                msg=f"stdout:\n{disabled.stdout}\nstderr:\n{disabled.stderr}",
            )
            disabled_ue = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )["command"]
            self.assertIn(
                "-ExecCmds=t.MaxFPS 30,r.MotionBlurQuality 0", disabled_ue
            )
            self.assertFalse(any("SpringArmComponent" in arg for arg in disabled_ue))
            self.assertFalse(any("viewclass" in arg for arg in disabled_ue))

            environment["MATRIX_GAME_CENTERED_CAMERA"] = "yes"
            environment["MATRIX_GAME_CAMERA_VIEW_CLASS"] = "MyRobotView_C"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-overridden.lock"
            )
            overridden = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                overridden.returncode,
                0,
                msg=f"stdout:\n{overridden.stdout}\nstderr:\n{overridden.stderr}",
            )
            overridden_ue = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )["command"]
            exec_cmds = next(
                arg for arg in overridden_ue if arg.startswith("-ExecCmds=")
            )
            self.assertTrue(exec_cmds.endswith(",viewclass MyRobotView_C"))
            self.assertNotIn("viewclass MujocoSim_Custom_C", exec_cmds)

            planner_command = [*command]
            planner_command[-1] = "planner"
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-planner.lock"
            )
            planner = subprocess.run(
                planner_command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(
                planner.returncode,
                0,
                msg=f"stdout:\n{planner.stdout}\nstderr:\n{planner.stderr}",
            )
            planner_ue = json.loads(
                fixture["ue_capture"].read_text(encoding="utf-8")
            )["command"]
            self.assertFalse(any("SpringArmComponent" in arg for arg in planner_ue))
            self.assertFalse(any("viewclass" in arg for arg in planner_ue))

    def test_centered_camera_rejects_invalid_boolean_and_command_injection(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            command = [
                "/bin/bash",
                os.fspath(project / "scripts/run_matrix_sonic.sh"),
                "--scene",
                "21",
                "--control-source",
                "game",
            ]
            environment["MATRIX_GAME_CENTERED_CAMERA"] = "sometimes"
            bad_boolean = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(bad_boolean.returncode, 1)
            self.assertIn(
                "MATRIX_GAME_CENTERED_CAMERA must be a boolean",
                bad_boolean.stderr,
            )

            environment["MATRIX_GAME_CENTERED_CAMERA"] = "true"
            environment["MATRIX_GAME_CAMERA_VIEW_CLASS"] = (
                "MujocoSim_Custom_C,quit"
            )
            environment["MATRIX_SONIC_HOST_LOCK"] = os.fspath(
                project / "launcher-injection.lock"
            )
            injection = subprocess.run(
                command,
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            self.assertEqual(injection.returncode, 1)
            self.assertIn(
                "MATRIX_GAME_CAMERA_VIEW_CLASS must be a short Blueprint class",
                injection.stderr,
            )
            self.assertFalse(fixture["ue_capture"].exists())

    def test_builtin_robot_types_select_their_native_camera_actor(self) -> None:
        expected_classes = {
            "go2": "MujoCoSim_go2_C",
            "go2w": "MujoCoSim_go2w_C",
            "xgb": "MujoCoSim_Xgb_C",
            "xgw": "MujoCoSim_Xgw_C",
            "zgws": "MujoCoSim_Zgws_C",
        }
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_DISABLE_MC": "1",
                "MATRIX_GAME_AUTO_RESPAWN": "0",
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    project / "outputs/game-input.json"
                ),
                "MATRIX_GAME_NO_INPUT_PROVIDER": "1",
                "MATRIX_GAME_WORLD_PERSISTENCE": "0",
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC": "1",
                "MATRIX_SONIC_CONTROL_SOURCE": "game",
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UNITREE_SDK2_ROOT": os.fspath(
                    fixture["sonic"] / "gear_sonic_deploy/thirdparty/unitree_sdk2"
                ),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            for robot_type, expected_class in expected_classes.items():
                with self.subTest(robot_type=robot_type):
                    result = subprocess.run(
                        [
                            "/bin/bash",
                            os.fspath(project / "scripts/run_sim.sh"),
                            robot_type,
                            "21",
                            "0",
                            "0",
                            "1",
                        ],
                        env=environment,
                        text=True,
                        capture_output=True,
                        timeout=20.0,
                        check=False,
                    )
                    self.assertEqual(
                        result.returncode,
                        0,
                        msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                    )
                    runtime_capture = json.loads(
                        fixture["capture"].read_text(encoding="utf-8")
                    )
                    self.assertEqual(runtime_capture["world_persistence_env"], "0")
                    self.assertEqual(runtime_capture["auto_respawn_env"], "0")
                    for flag in (
                        "--game-world-id",
                        "--game-world-revision",
                        "--game-world-state-file",
                        "--game-world-checkpoint-seconds",
                        "--game-auto-respawn",
                    ):
                        self.assertNotIn(flag, runtime_capture["argv"])
                    ue_command = json.loads(
                        fixture["ue_capture"].read_text(encoding="utf-8")
                    )["command"]
                    exec_cmds = next(
                        arg
                        for arg in ue_command
                        if arg.startswith("-ExecCmds=")
                    )
                    self.assertTrue(
                        exec_cmds.endswith(f",viewclass {expected_class}"),
                        exec_cmds,
                    )

    def test_late_ue_failure_invalidates_world_internal_restart_exit_75(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            status_file = project / "outputs/matrix-sonic-status.json"
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "FAKE_UE_LATE_FAILURE_EXIT_CODE": "42",
                "FAKE_WORLD_INTERNAL_RESTART": "game_teleport",
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_DISABLE_MC": "1",
                "MATRIX_GAME_AUTO_RESPAWN": "0",
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    project / "outputs/game-input.json"
                ),
                "MATRIX_GAME_NO_INPUT_PROVIDER": "1",
                "MATRIX_GAME_WORLD_PERSISTENCE": "0",
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC": "1",
                "MATRIX_SONIC_CONTROL_SOURCE": "game",
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_SONIC_STATUS_FILE": os.fspath(status_file),
                "MATRIX_UNITREE_SDK2_ROOT": os.fspath(
                    fixture["sonic"]
                    / "gear_sonic_deploy/thirdparty/unitree_sdk2"
                ),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }

            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_sim.sh"),
                    "xgb",
                    "21",
                    "0",
                    "0",
                    "1",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )

            self.assertEqual(
                result.returncode,
                2,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            status = json.loads(status_file.read_text(encoding="utf-8"))
            self.assertEqual(status["pre_external_termination_reason"], "game_teleport")
            self.assertEqual(status["termination_reason"], "child_exit")
            self.assertEqual(status["failed_child_name"], "ue")
            self.assertEqual(status["failed_child_exit_code"], 42)

    def test_outer_launcher_rejects_exit_75_status_with_late_child_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            generations = project / "generations.txt"
            self.write(
                project / "scripts/run_sim.sh",
                """#!/usr/bin/env bash
set -euo pipefail
printf 'x' >> "${GENERATION_FILE:?}"
mkdir -p "$(dirname "${MATRIX_SONIC_STATUS_FILE:?}")"
printf '%s\n' '{"internal_restart":{"requested":true,"reason":"game_teleport"},"game_world_state":{"has_last_exit":true,"last_error":null},"game_auto_respawn":true,"termination_reason":"child_exit","termination_signal":null,"failed_child_name":"ue","failed_child_exit_code":42}' > "$MATRIX_SONIC_STATUS_FILE"
exit 75
""",
                executable=True,
            )
            environment = {
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }

            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )

            self.assertEqual(
                result.returncode,
                75,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "x")
            self.assertIn(
                "Refusing unverified Matrix world reload request",
                result.stderr,
            )
            self.assertNotIn("Validated Matrix world reload", result.stdout)

    def test_outer_launcher_rejects_exit_75_with_termination_signal(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            generations = project / "generations.txt"
            self.write(
                project / "scripts/run_sim.sh",
                """#!/usr/bin/env bash
set -euo pipefail
printf 'x' >> "${GENERATION_FILE:?}"
mkdir -p "$(dirname "${MATRIX_SONIC_STATUS_FILE:?}")"
printf '%s\n' '{"internal_restart":{"requested":true,"reason":"game_teleport"},"game_world_state":{"has_last_exit":true,"last_error":null},"game_auto_respawn":true,"termination_reason":"game_teleport","termination_signal":15,"failed_child_name":null,"failed_child_exit_code":null}' > "$MATRIX_SONIC_STATUS_FILE"
exit 75
""",
                executable=True,
            )
            environment = {
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }

            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )

            self.assertEqual(
                result.returncode,
                75,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "x")
            self.assertIn(
                "Refusing unverified Matrix world reload request",
                result.stderr,
            )
            self.assertNotIn("Validated Matrix world reload", result.stdout)

    def test_outer_launcher_accepts_clean_exit_75_and_carries_rate_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            generations = project / "generations.txt"
            rate_trace = project / "rate-trace.txt"
            self.write(
                project / "scripts/run_sim.sh",
                """#!/usr/bin/env bash
set -euo pipefail
generation=1
if [[ -f "${GENERATION_FILE:?}" ]]; then
    generation=$(( $(<"$GENERATION_FILE") + 1 ))
fi
printf '%s' "$generation" > "$GENERATION_FILE"
printf '%s,%s\n' \
    "${MATRIX_GAME_INTERNAL_RESTART_COUNT:-missing}" \
    "${MATRIX_GAME_INTERNAL_RESTART_WINDOW_EPOCH:-missing}" \
    >> "${RATE_TRACE_FILE:?}"
if [[ "$generation" == "1" ]]; then
    mkdir -p "$(dirname "${MATRIX_SONIC_STATUS_FILE:?}")"
    printf '%s\n' '{"internal_restart":{"requested":true,"reason":"game_teleport"},"game_world_state":{"has_last_exit":true,"last_error":null},"game_auto_respawn":true,"termination_reason":"game_teleport","termination_signal":null,"failed_child_name":null,"failed_child_exit_code":null}' > "$MATRIX_SONIC_STATUS_FILE"
    exit 75
fi
exit 0
""",
                executable=True,
            )
            environment = {
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "RATE_TRACE_FILE": os.fspath(rate_trace),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }

            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )

            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "2")
            first, second = rate_trace.read_text(encoding="utf-8").splitlines()
            self.assertEqual(first, "missing,missing")
            count, window = second.split(",", 1)
            self.assertEqual(count, "1")
            self.assertRegex(window, r"^[0-9]+$")
            self.assertGreater(int(window), 0)
            self.assertIn("Validated Matrix world reload", result.stdout)
            self.assertIn("count=1/6", result.stdout)

    def test_private_request_restarts_whole_runtime_after_clean_restore(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            temporary_dir = project / "tmp"
            temporary_dir.mkdir()
            marker = project / "restart-once.marker"
            generations = project / "generations.txt"
            mutable = [
                project / "config/config.json",
                project / "src/robot_mujoco/simulate/config.yaml",
                project / "src/robot_mc/run_mc.sh",
            ]
            originals = {path: path.read_bytes() for path in mutable}
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "TRIGGER_RESTART_MARKER": os.fspath(marker),
                "TMPDIR": os.fspath(temporary_dir),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                0,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "2")
            self.assertIn(
                "Validated full Matrix runtime restart request", result.stdout
            )
            for path, expected in originals.items():
                self.assertEqual(path.read_bytes(), expected, msg=os.fspath(path))

    def test_private_request_refuses_restart_when_final_checkpoint_failed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            temporary_dir = project / "tmp"
            temporary_dir.mkdir()
            marker = project / "restart-once.marker"
            generations = project / "generations.txt"
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "FAKE_FINAL_CHECKPOINT_ERROR": "simulated durable write failure",
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "TRIGGER_RESTART_MARKER": os.fspath(marker),
                "TMPDIR": os.fspath(temporary_dir),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }

            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )

            self.assertEqual(
                result.returncode,
                143,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "1")
            self.assertIn(
                "Refusing Matrix restart without a verified final world checkpoint",
                result.stderr,
            )
            self.assertNotIn(
                "Verified final Matrix world checkpoint",
                result.stdout,
            )

    def test_internal_restart_timeout_keeps_supervising_original_runtime(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            temporary_dir = project / "tmp"
            temporary_dir.mkdir()
            restart_marker = project / "restart-once.marker"
            provider_term_marker = project / "provider-term-complete"
            generations = project / "generations.txt"
            mutable = [
                project / "config/config.json",
                project / "src/robot_mujoco/simulate/config.yaml",
                project / "src/robot_mc/run_mc.sh",
            ]
            originals = {path: path.read_bytes() for path in mutable}
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "FAKE_PROVIDER_TERM_DELAY_SECONDS": "0.6",
                "FAKE_PROVIDER_TERM_MARKER": os.fspath(provider_term_marker),
                "FAKE_PROVIDER_OBSERVE_PATH": os.fspath(mutable[1]),
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_RUN_SIM_STOP_TIMEOUT_SECONDS": "0.05",
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "TMPDIR": os.fspath(temporary_dir),
                "TRIGGER_RESTART_MARKER": os.fspath(restart_marker),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                143,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "1")
            observed_during_timeout = provider_term_marker.read_text(
                encoding="utf-8"
            )
            self.assertNotEqual(
                observed_during_timeout,
                originals[mutable[1]].decode("utf-8"),
            )
            self.assertIn("scene_terrain_apart2.xml", observed_during_timeout)
            self.assertEqual(result.stderr.count("restart timed out"), 1)
            self.assertNotIn("external-signal cleanup", result.stderr)
            for path, expected in originals.items():
                self.assertEqual(path.read_bytes(), expected, msg=os.fspath(path))

    def test_restore_verification_failure_never_execs_next_generation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            temporary_dir = project / "tmp"
            temporary_dir.mkdir()
            marker = project / "restart-once.marker"
            generations = project / "generations.txt"
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "FAIL_CONFIG_RESTORE": "1",
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "TRIGGER_RESTART_MARKER": os.fspath(marker),
                "TMPDIR": os.fspath(temporary_dir),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                143,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "1")
            self.assertIn("Refusing restart", result.stderr)

    def test_external_signal_during_restore_cancels_pending_restart(self) -> None:
        for signal_name, expected_exit_code in (
            ("INT", 130),
            ("TERM", 143),
            ("HUP", 129),
        ):
            with (
                self.subTest(signal=signal_name),
                tempfile.TemporaryDirectory() as temporary,
            ):
                project = Path(temporary) / "matrix"
                fixture = self.make_project(project)
                runtime_dir = project / "runtime"
                runtime_dir.mkdir()
                temporary_dir = project / "tmp"
                temporary_dir.mkdir()
                restart_marker = project / "restart-once.marker"
                signal_marker = project / "restore-signal-fired"
                generations = project / "generations.txt"
                mutable = [
                    project / "config/config.json",
                    project / "src/robot_mujoco/simulate/config.yaml",
                    project / "src/robot_mc/run_mc.sh",
                ]
                originals = {path: path.read_bytes() for path in mutable}
                environment = {
                    "CAPTURE_PATH": os.fspath(fixture["capture"]),
                    "GENERATION_FILE": os.fspath(generations),
                    "HOME": os.fspath(project / "home"),
                    "LANG": "C.UTF-8",
                    "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                    "MATRIX_SKIP_ENV_CHECK": "1",
                    "MATRIX_SONIC_HOST_LOCK": os.fspath(
                        project / "launcher.lock"
                    ),
                    "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                    "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                    "MATRIX_UE_STARTUP_SECONDS": "0",
                    "MATRIX_VERIFY_RUNTIME": "0",
                    "PATH": os.fspath(fixture["fake_bin"])
                    + os.pathsep
                    + os.environ.get("PATH", "/usr/bin:/bin"),
                    "SIGNAL_LAUNCHER_DURING_RESTORE": signal_name,
                    "SIGNAL_RESTORE_MARKER": os.fspath(signal_marker),
                    "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                    "TMPDIR": os.fspath(temporary_dir),
                    "TRIGGER_RESTART_MARKER": os.fspath(restart_marker),
                    "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                    "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
                }
                result = subprocess.run(
                    [
                        "/bin/bash",
                        os.fspath(project / "scripts/run_matrix_sonic.sh"),
                        "--scene",
                        "21",
                        "--control-source",
                        "game",
                    ],
                    env=environment,
                    text=True,
                    capture_output=True,
                    timeout=30.0,
                    check=False,
                )
                self.assertEqual(
                    result.returncode,
                    expected_exit_code,
                    msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
                )
                self.assertEqual(generations.read_text(encoding="utf-8"), "1")
                self.assertTrue(signal_marker.is_dir())
                self.assertIn("External stop cancelled", result.stderr)
                for path, expected in originals.items():
                    self.assertEqual(path.read_bytes(), expected, msg=os.fspath(path))

    def test_external_signal_during_final_restore_overrides_runtime_exit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            temporary_dir = project / "tmp"
            temporary_dir.mkdir()
            signal_marker = project / "restore-signal-fired"
            generations = project / "generations.txt"
            mutable = [
                project / "config/config.json",
                project / "src/robot_mujoco/simulate/config.yaml",
                project / "src/robot_mc/run_mc.sh",
            ]
            originals = {path: path.read_bytes() for path in mutable}
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "GENERATION_FILE": os.fspath(generations),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_G1_URDF": os.fspath(fixture["custom_urdf"]),
                "MATRIX_SKIP_ENV_CHECK": "1",
                "MATRIX_SONIC_HOST_LOCK": os.fspath(project / "launcher.lock"),
                "MATRIX_SONIC_PYTHON": os.fspath(fixture["fake_python"]),
                "MATRIX_SONIC_ROOT": os.fspath(fixture["sonic"]),
                "MATRIX_UE_STARTUP_SECONDS": "0",
                "MATRIX_VERIFY_RUNTIME": "0",
                "PATH": os.fspath(fixture["fake_bin"])
                + os.pathsep
                + os.environ.get("PATH", "/usr/bin:/bin"),
                "SIGNAL_LAUNCHER_DURING_RESTORE": "TERM",
                "SIGNAL_RESTORE_MARKER": os.fspath(signal_marker),
                "SIM_LAUNCHER_SKIP_CUSTOM_URDF_WRAPPER": "1",
                "TMPDIR": os.fspath(temporary_dir),
                "UE_CAPTURE_PATH": os.fspath(fixture["ue_capture"]),
                "XDG_RUNTIME_DIR": os.fspath(runtime_dir),
            }
            result = subprocess.run(
                [
                    "/bin/bash",
                    os.fspath(project / "scripts/run_matrix_sonic.sh"),
                    "--scene",
                    "21",
                    "--control-source",
                    "game",
                ],
                env=environment,
                text=True,
                capture_output=True,
                timeout=30.0,
                check=False,
            )
            self.assertEqual(
                result.returncode,
                143,
                msg=f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}",
            )
            self.assertEqual(generations.read_text(encoding="utf-8"), "1")
            self.assertTrue(signal_marker.is_dir())
            for path, expected in originals.items():
                self.assertEqual(path.read_bytes(), expected, msg=os.fspath(path))


if __name__ == "__main__":
    unittest.main()
