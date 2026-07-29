import importlib.util
import json
import socket
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "matrix_sonic_amp_worker.py"
SPEC = importlib.util.spec_from_file_location("matrix_sonic_amp_worker", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
worker = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = worker
SPEC.loader.exec_module(worker)


def config_mapping():
    return {
        "policy_joint_names": list(worker.G1_29_JOINT_NAMES),
        "obs_config": {
            "history_length": 4,
            "policy": [
                {"name": "RootAngVelB"},
                {"name": "ProjectedGravityB"},
                {"name": "Command"},
                {"name": "JointPos"},
                {"name": "JointVel"},
                {"name": "PrevActions"},
            ],
        },
        "obs_joint_pos_relative": True,
        "default_joint_pos": [float(index) / 10.0 for index in range(29)],
        "action_scale": [0.5] * 29,
        "stiffness": [20.0 + index for index in range(29)],
        "damping": [1.0 + index / 10.0 for index in range(29)],
        "action_clip": 2.0,
        "sim": {"control_dt": 0.02},
    }


def snapshot(marker=0.0):
    return worker.LowStateSnapshot.validated(
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        body_gyro_rad_s=(marker, marker + 1.0, marker + 2.0),
        joint_pos_rad=np.arange(29, dtype=np.float32) + marker,
        joint_vel_rad_s=np.arange(29, dtype=np.float32) + marker + 100.0,
        received_monotonic=10.0 + marker,
    )


class RecordingRunner:
    def __init__(self, outputs):
        self.outputs = list(outputs)
        self.inputs = []

    def __call__(self, observation):
        self.inputs.append(np.asarray(observation).copy())
        return np.asarray(self.outputs.pop(0), dtype=np.float32)


class PolicyCoreTests(unittest.TestCase):
    def test_observation_is_four_frames_oldest_to_newest(self):
        config = worker.PolicyConfig.from_mapping(config_mapping())
        runner = RecordingRunner([np.zeros(29), np.full(29, 7.0)])
        policy = worker.AmpPolicyCore(config, runner)

        first = snapshot(1.0)
        second = snapshot(2.0)
        policy.infer(first)
        policy.infer(second)

        self.assertEqual(runner.inputs[0].shape, (1, 384))
        initial_frames = runner.inputs[0].reshape(4, 96)
        np.testing.assert_array_equal(initial_frames[0], initial_frames[1])
        np.testing.assert_array_equal(initial_frames[1], initial_frames[2])
        np.testing.assert_array_equal(initial_frames[2], initial_frames[3])

        later_frames = runner.inputs[1].reshape(4, 96)
        np.testing.assert_array_equal(later_frames[0], initial_frames[1])
        np.testing.assert_array_equal(later_frames[1], initial_frames[2])
        np.testing.assert_array_equal(later_frames[2], initial_frames[3])
        # Frame 3 is the new sample, and frame 2 remains the preceding sample.
        self.assertEqual(float(later_frames[2, 0]), 1.0)
        self.assertEqual(float(later_frames[3, 0]), 2.0)
        np.testing.assert_array_equal(later_frames[3, 6:9], np.zeros(3))
        # PrevActions is the preceding raw actor output, never a q target.
        np.testing.assert_array_equal(later_frames[3, 67:96], np.zeros(29))

    def test_action_target_clips_then_scales_and_prev_action_is_action_space(self):
        config = worker.PolicyConfig.from_mapping(config_mapping())
        raw = np.linspace(-4.0, 4.0, 29, dtype=np.float32)
        runner = RecordingRunner([raw, np.zeros(29)])
        policy = worker.AmpPolicyCore(config, runner)

        result = policy.infer(snapshot())
        clipped = np.clip(raw, -2.0, 2.0)
        np.testing.assert_allclose(
            result.target_joint_pos,
            config.default_joint_pos + config.action_scale * clipped,
        )
        np.testing.assert_array_equal(result.raw_action, raw)

        policy.infer(snapshot(1.0))
        second_input = runner.inputs[1].reshape(4, 96)
        np.testing.assert_array_equal(second_input[-1, 67:96], clipped)

    def test_identity_imu_projects_world_gravity_down(self):
        np.testing.assert_allclose(
            worker.projected_gravity_body((1.0, 0.0, 0.0, 0.0)),
            (0.0, 0.0, -1.0),
        )
        self.assertEqual(worker.root_up_z_from_imu((1.0, 0.0, 0.0, 0.0)), 1.0)


class HandoffProtocolTests(unittest.TestCase):
    def test_go_is_only_transition_that_constructs_writer(self):
        publisher_calls = []
        events = []
        publisher = object()

        def create_publisher():
            publisher_calls.append("created")
            return publisher

        state = worker.HandoffStateMachine(
            create_publisher,
            lambda event, fields: events.append((event, dict(fields))),
        )
        # Construction and readiness reporting are both writer-free.
        self.assertEqual(publisher_calls, [])
        self.assertIsNone(state.publisher)
        state.announce_ready()
        self.assertEqual(publisher_calls, [])
        self.assertEqual(events, [("READY_NO_WRITER", {"writer_created": False})])

        state.command(worker.decode_command(b'{"command":"GO"}'))
        self.assertEqual(publisher_calls, ["created"])
        self.assertIs(state.publisher, publisher)
        self.assertEqual(state.state, worker.HandoffStateMachine.ACTIVE)
        state.record_successful_write()
        state.record_successful_write()
        self.assertEqual([event for event, _fields in events].count("FIRST_WRITE"), 1)

        state.command(worker.decode_command(b"PAUSE"))
        self.assertEqual(state.state, worker.HandoffStateMachine.PAUSED)
        self.assertIs(state.publisher, publisher)
        self.assertEqual(
            events[-1],
            (
                "PAUSED_RESIDENT_WRITER",
                {
                    "writer_created": True,
                    "write_authorized": False,
                    "writer_reused": True,
                },
            ),
        )
        state.command(worker.decode_command(b"GO"))
        self.assertEqual(publisher_calls, ["created"])
        self.assertIs(state.publisher, publisher)
        state.record_successful_write()
        self.assertEqual([event for event, _fields in events].count("FIRST_WRITE"), 2)

        state.command(worker.decode_command(b"STOP"))
        self.assertEqual(state.state, worker.HandoffStateMachine.STOPPED)
        self.assertIsNone(state.publisher)
        self.assertEqual(events[-1], ("STOPPED", {"writer_created": False}))

    @unittest.skipUnless(hasattr(socket, "SOCK_SEQPACKET"), "requires SOCK_SEQPACKET")
    def test_json_protocol_preserves_packet_boundary_and_schema(self):
        receiver, sender = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            sender.send(worker.encode_packet("READY_NO_WRITER", writer_created=False))
            payload = json.loads(receiver.recv(4096).decode("utf-8"))
        finally:
            receiver.close()
            sender.close()
        self.assertEqual(payload["schema"], worker.CONTROL_SCHEMA)
        self.assertEqual(payload["event"], "READY_NO_WRITER")
        self.assertFalse(payload["writer_created"])

    def test_protocol_rejects_unknown_command(self):
        with self.assertRaises(ValueError):
            worker.decode_command(b'{"command":"RESET"}')


class FakeMotorCommand:
    def __init__(self):
        self.mode = 99
        self.q = 99.0
        self.dq = 99.0
        self.tau = 99.0
        self.kp = 99.0
        self.kd = 99.0
        self.reserve = 99


class FakeLowCommand:
    def __init__(self):
        self.mode_pr = 99
        self.mode_machine = 99
        self.motor_cmd = [FakeMotorCommand() for _ in range(35)]
        self.reserve = [0, 0, 0, 0]
        self.crc = 0


class FakeCrc:
    def __init__(self):
        self.calls = []

    def Crc(self, command):
        self.calls.append(command)
        return 12345


class LowCommandTests(unittest.TestCase):
    def test_python_hg_crc_matches_vendor_reference_vector(self):
        command = FakeLowCommand()
        command.mode_pr = 0
        command.mode_machine = 5
        for index, motor in enumerate(command.motor_cmd):
            motor.mode = 1 if index < 29 else 0
            motor.q = index * 0.01
            motor.dq = index * -0.02
            motor.tau = index * 0.03
            motor.kp = 40.0 + index
            motor.kd = 2.0 + index * 0.1
            motor.reserve = index
        command.reserve = [11, 22, 33, 44]
        # Generated with unitree_sdk2py 1.0.1 CRC._crc_py over the same
        # documented <2B2x + 35*(B3x5fI) + 5I byte layout.
        self.assertEqual(worker.HgLowCmdCrc().Crc(command), 0x0CB8F6BA)

    def test_all_35_slots_are_initialized_and_crc_is_set(self):
        runtime = worker.UnitreeDdsRuntime.__new__(worker.UnitreeDdsRuntime)
        runtime._low_cmd_factory = FakeLowCommand
        runtime._crc = FakeCrc()
        config = worker.PolicyConfig.from_mapping(config_mapping())
        target = np.arange(29, dtype=np.float32) / 3.0

        command = runtime.make_low_cmd(target, config, snapshot())

        self.assertEqual(command.mode_pr, 0)
        for index in range(29):
            motor = command.motor_cmd[index]
            self.assertEqual(motor.mode, 1)
            self.assertAlmostEqual(motor.q, float(target[index]))
            self.assertEqual(motor.dq, 0.0)
            self.assertEqual(motor.tau, 0.0)
            self.assertAlmostEqual(motor.kp, float(config.kp[index]))
            self.assertAlmostEqual(motor.kd, float(config.kd[index]))
            self.assertEqual(motor.reserve, 0)
        for motor in command.motor_cmd[29:]:
            self.assertEqual(
                (motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd, motor.reserve),
                (0, 0.0, 0.0, 0.0, 0.0, 0.0, 0),
            )
        self.assertEqual(command.crc, 12345)
        self.assertEqual(runtime._crc.calls, [command])


if __name__ == "__main__":
    unittest.main()
