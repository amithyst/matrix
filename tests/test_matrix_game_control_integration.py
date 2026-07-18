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

                self.assertEqual(planner_frames[-1]["mode"], 1)
                # Unmodified keyboard WASD is the ordinary-walk midpoint
                # between SONIC's 0.10 m/s floor and the 0.30 m/s run cap.
                self.assertAlmostEqual(planner_frames[-1]["speed"], 0.2)
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
                self.assertEqual(
                    runtime.telemetry(now_s=10.011)["moving_command_frames"], 1
                )

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
    @staticmethod
    def write(path: Path, contents: str = "", *, executable: bool = False) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")
        if executable:
            path.chmod(path.stat().st_mode | stat.S_IXUSR)

    def make_project(self, project: Path) -> dict[str, Path]:
        scripts = project / "scripts"
        scripts.mkdir(parents=True)
        for name in ("run_matrix_sonic.sh", "run_sim.sh"):
            shutil.copy2(SCRIPTS / name, scripts / name)
        self.write(
            scripts / "matrix_local_env.sh",
            "load_matrix_local_env() { return 0; }\n",
        )
        for name in (
            "compose_custom_scene.py",
            "matrix_game_control.py",
            "matrix_game_control_input.py",
            "prepare_sonic_physics_model.py",
            "run_matrix_sonic.py",
            "supervise_matrix_ue.py",
        ):
            self.write(scripts / name, "# integration fixture\n")

        lock = project / "config/runtime/matrix-sonic.lock.json"
        lock.parent.mkdir(parents=True)
        shutil.copy2(REPO_ROOT / "config/runtime/matrix-sonic.lock.json", lock)
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
        self.write(project / "scene/scene.json", "{}\n")
        scene_xml = "<mujoco model=\"fixture\"/>\n"
        self.write(
            project
            / "src/robot_mujoco/zsibot_robots/xgb/scene_terrain_apart2.xml",
            scene_xml,
        )
        self.write(
            project
            / "src/UeSim/Linux/zsibot_mujoco_ue/Content/model/xgb/scene_terrain_apart2.xml",
            scene_xml,
        )
        self.write(
            project / "src/UeSim/Linux/zsibot_mujoco_ue.sh",
            "#!/usr/bin/env bash\nexit 0\n",
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
import sys

script = Path(sys.argv[1]).name
args = sys.argv[2:]

if script == "compose_custom_scene.py":
    target = Path(args[1])
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args[0], target)
elif script == "prepare_sonic_physics_model.py":
    output = Path(args[args.index("--output-dir") + 1])
    native = Path(args[args.index("--native-scene") + 1])
    output.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(native, output / native.name)
elif script == "supervise_matrix_ue.py":
    pid_file = Path(args[args.index("--pid-file") + 1])
    pid_file.write_text(str(os.getpid()), encoding="utf-8")
    for line in sys.stdin:
        if line.strip() == "stop":
            break
elif script == "run_matrix_sonic.py":
    socket_path = Path(args[args.index("--game-input-socket") + 1])
    status_path = Path(os.environ["MATRIX_GAME_INPUT_STATUS_FILE"])
    capture = {
        "argv": args,
        "control_source_env": os.environ.get("MATRIX_SONIC_CONTROL_SOURCE"),
        "input_source_env": os.environ.get("MATRIX_GAME_INPUT_SOURCE"),
        "yaw_source_env": os.environ.get("MATRIX_GAME_CAMERA_YAW_SOURCE"),
        "socket_env": os.environ.get("MATRIX_GAME_INPUT_SOCKET"),
        "socket_parent_mode": oct(socket_path.parent.stat().st_mode & 0o777),
        "stale_status_existed": status_path.exists(),
    }
    Path(os.environ["CAPTURE_PATH"]).write_text(
        json.dumps(capture), encoding="utf-8"
    )
else:
    raise SystemExit(f"unexpected fake runtime invocation: {script}")
""",
            executable=True,
        )
        return {
            "capture": capture,
            "custom_urdf": custom_urdf,
            "fake_bin": fake_bin,
            "fake_python": fake_python,
            "sonic": sonic,
            "stale_status": stale_status,
        }

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

    def test_real_launcher_and_run_sim_forward_all_game_arguments(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            project = Path(temporary) / "matrix"
            fixture = self.make_project(project)
            runtime_dir = project / "runtime"
            runtime_dir.mkdir()
            environment = {
                "CAPTURE_PATH": os.fspath(fixture["capture"]),
                "HOME": os.fspath(project / "home"),
                "LANG": "C.UTF-8",
                "MATRIX_GAME_INPUT_STATUS_FILE": os.fspath(
                    fixture["stale_status"]
                ),
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
            self.assertEqual(capture["socket_parent_mode"], "0o700")
            self.assertFalse(capture["stale_status_existed"])

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


if __name__ == "__main__":
    unittest.main()
