import hashlib
import json
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import matrix_sonic_host_worker as worker  # noqa: E402


TEST_RESIDENT_POLICIES = (
    {
        "name": "host:test",
        "execution_provider": "CPUExecutionProvider",
        "warmed": True,
    },
)


def snapshot(
    marker=0.0,
    *,
    received=None,
    quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    body_gyro_rad_s=None,
    joint_vel_rad_s=None,
):
    return worker.LowStateSnapshot.validated(
        quaternion_wxyz=quaternion_wxyz,
        body_gyro_rad_s=(
            (marker, marker + 1.0, marker + 2.0)
            if body_gyro_rad_s is None
            else body_gyro_rad_s
        ),
        joint_pos_rad=np.arange(29, dtype=np.float32) + marker,
        joint_vel_rad_s=(
            np.arange(29, dtype=np.float32) + marker + 100.0
            if joint_vel_rad_s is None
            else joint_vel_rad_s
        ),
        received_monotonic=time.monotonic() if received is None else received,
        mode_machine=7,
    )


def amp_config_mapping():
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
        "default_joint_pos": [float(index) / 100.0 for index in range(29)],
        "action_scale": [0.25] * 29,
        "stiffness": [20.0 + index for index in range(29)],
        "damping": [1.0 + index / 10.0 for index in range(29)],
        "action_clip": 2.0,
        "sim": {"control_dt": 0.02},
    }


class RecordingRunner:
    def __init__(self, label, outputs):
        self.label = label
        self.outputs = list(outputs)
        self.inputs = []

    def __call__(self, observation):
        self.inputs.append(np.asarray(observation).copy())
        return np.asarray(self.outputs.pop(0), dtype=np.float32)


class HostPolicyTests(unittest.TestCase):
    def test_unitree_dds_publisher_lease_releases_for_next_episode(self):
        created = []

        class SdkPublisher:
            def __init__(self, topic, message_type):
                self.topic = topic
                self.message_type = message_type
                self.initialized = False
                self.closed = False
                self.writes = []
                created.append(self)

            def Init(self):
                self.initialized = True

            def Write(self, command):
                self.writes.append(command)
                return True

            def Close(self):
                self.closed = True

        runtime = worker.UnitreeDdsRuntime.__new__(worker.UnitreeDdsRuntime)
        runtime._channel_publisher = SdkPublisher
        runtime._low_cmd_type = object()
        runtime._publisher = None

        first = runtime.create_publisher()
        self.assertTrue(first.Write("episode-1"))
        self.assertIs(runtime._publisher, first)
        first.Close()
        self.assertIsNone(runtime._publisher)
        self.assertTrue(created[0].closed)

        second = runtime.create_publisher()
        self.assertIsNot(second, first)
        self.assertEqual(len(created), 2)
        self.assertTrue(second.Write("episode-2"))
        second.Close()
        self.assertIsNone(runtime._publisher)

    def test_model_sha256_helper_reads_exact_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "policy.onnx"
            payload = b"host-policy-test-bytes"
            path.write_bytes(payload)
            self.assertEqual(
                worker.file_sha256(path), hashlib.sha256(payload).hexdigest()
            )

    def test_required_sha256_rejects_mismatch_and_malformed_digest(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "artifact"
            path.write_bytes(b"immutable-policy")
            expected = hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(
                worker.require_matching_sha256(path, expected, "test artifact"),
                expected,
            )
            with self.assertRaisesRegex(ValueError, "mismatch"):
                worker.require_matching_sha256(path, "0" * 64, "test artifact")
            with self.assertRaisesRegex(ValueError, "64 lowercase"):
                worker.require_matching_sha256(path, "not-a-sha", "test artifact")

    def test_six_frame_history_is_oldest_to_newest(self):
        runner = RecordingRunner("prone_v1", [np.zeros(23), np.ones(23)])
        policy = worker.HostPolicyCore(worker.HostControlConfig.create(), runner)

        policy.infer(snapshot(1.0))
        policy.infer(snapshot(2.0))

        self.assertEqual(runner.inputs[0].shape, (1, 456))
        first = runner.inputs[0].reshape(6, 76)
        for index in range(1, 6):
            np.testing.assert_array_equal(first[0], first[index])
        second = runner.inputs[1].reshape(6, 76)
        for index in range(5):
            np.testing.assert_array_equal(second[index], first[index + 1])
        self.assertEqual(float(second[-2, 0]), 0.25)
        self.assertEqual(float(second[-1, 0]), 0.5)
        np.testing.assert_array_equal(second[-1, 52:75], np.zeros(23))
        self.assertEqual(float(second[-1, 75]), 0.25)

    def test_incremental_target_uses_current_q_and_holds_six_uncontrolled_joints(self):
        raw = np.linspace(-200.0, 200.0, 23, dtype=np.float32)
        runner = RecordingRunner("prone_v1", [raw])
        config = worker.HostControlConfig.create(
            action_rescale=0.25, action_clip=100.0
        )
        state = snapshot(3.0)
        policy = worker.HostPolicyCore(config, runner)

        result = policy.infer(state)

        clipped = np.clip(raw, -100.0, 100.0)
        np.testing.assert_allclose(
            result.target_joint_pos[worker.HOST_TO_MATRIX_INDICES],
            state.joint_pos_rad[worker.HOST_TO_MATRIX_INDICES] + 0.25 * clipped,
        )
        np.testing.assert_array_equal(
            result.target_joint_pos[worker.HOST_HELD_MATRIX_INDICES],
            state.joint_pos_rad[worker.HOST_HELD_MATRIX_INDICES],
        )

    def test_timeout_requests_supervisor_then_advance_uses_physical_state(self):
        first = RecordingRunner("prone_v1", [np.zeros(23)])
        second = RecordingRunner("prone_v2", [np.zeros(23)])
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(first, second),
            fallback_after_s=8.0,
        )
        initial = snapshot(0.0)
        half_sqrt_two = 2.0**-0.5
        current = snapshot(
            9.0,
            quaternion_wxyz=(half_sqrt_two, half_sqrt_two, 0.0, 0.0),
            body_gyro_rad_s=(0.0, 0.0, 0.0),
            joint_vel_rad_s=np.zeros(29),
        )
        cascade.start(initial, 100.0)

        self.assertIsNone(cascade.maybe_request_fallback(current, 107.99))
        request = cascade.maybe_request_fallback(current, 108.0)

        self.assertEqual(cascade.index, 0)
        self.assertTrue(request["requires_supervisor_authorization"])
        self.assertEqual(request["policy"], "prone_v1")
        self.assertEqual(request["next_policy"], "prone_v2")
        self.assertIsNone(cascade.maybe_request_fallback(current, 109.0))

        event = cascade.advance(current, 109.0)

        self.assertEqual(cascade.index, 1)
        self.assertEqual(event["from_policy"], "prone_v1")
        self.assertEqual(event["to_policy"], "prone_v2")
        self.assertTrue(event["physical_continuation"])
        output = cascade.infer(current)
        np.testing.assert_array_equal(
            output.target_joint_pos[worker.HOST_HELD_MATRIX_INDICES],
            current.joint_pos_rad[worker.HOST_HELD_MATRIX_INDICES],
        )

    def test_advance_before_fallback_due_fails_closed(self):
        first = RecordingRunner("prone_v1", [np.zeros(23)])
        second = RecordingRunner("prone_v2", [np.zeros(23)])
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(first, second),
            fallback_after_s=8.0,
        )
        cascade.start(snapshot(0.0), 100.0)

        with self.assertRaisesRegex(RuntimeError, "before fallback was due"):
            cascade.advance(snapshot(1.0), 101.0)

    def test_policy_switch_blends_from_measured_joint_pose(self):
        first = RecordingRunner("prone_v1", [np.zeros(23)])
        second = RecordingRunner(
            "prone_v2", [np.ones(23), np.ones(23), np.ones(23)]
        )
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(action_rescale=0.25),
            runners=(first, second),
            fallback_after_s=8.0,
            policy_switch_blend_s=0.4,
        )
        half_sqrt_two = 2.0**-0.5
        switch_state = snapshot(
            1.0,
            # The transition is authorized at t=108 with the oldest LowState
            # still allowed by the worker.  The first target must nevertheless
            # begin at measured-q rather than 62.5% through the blend.
            received=107.75,
            quaternion_wxyz=(half_sqrt_two, half_sqrt_two, 0.0, 0.0),
            body_gyro_rad_s=(0.0, 0.0, 0.0),
            joint_vel_rad_s=np.zeros(29),
        )
        cascade.start(snapshot(0.0), 100.0)
        self.assertIsNotNone(cascade.maybe_request_fallback(switch_state, 108.0))
        cascade.advance(switch_state, 108.0)

        first_output = cascade.infer(switch_state)
        np.testing.assert_allclose(
            first_output.target_joint_pos, switch_state.joint_pos_rad
        )
        halfway_state = worker.LowStateSnapshot.validated(
            quaternion_wxyz=switch_state.quaternion_wxyz,
            body_gyro_rad_s=switch_state.body_gyro_rad_s,
            joint_pos_rad=switch_state.joint_pos_rad,
            joint_vel_rad_s=np.zeros(29),
            received_monotonic=108.2,
            mode_machine=7,
        )
        halfway_output = cascade.infer(halfway_state)
        np.testing.assert_allclose(
            halfway_output.target_joint_pos[worker.HOST_TO_MATRIX_INDICES],
            switch_state.joint_pos_rad[worker.HOST_TO_MATRIX_INDICES] + 0.125,
            atol=1e-6,
        )


class FakePublisher:
    def __init__(self):
        self.commands = []
        self.closed = False

    def Write(self, command):
        self.commands.append(command)
        return True

    def Close(self):
        self.closed = True


class FakeMotor:
    def __init__(self):
        self.mode = 99
        self.q = 99.0
        self.dq = 99.0
        self.tau = 99.0
        self.kp = 99.0
        self.kd = 99.0
        self.reserve = 99


class FakeCommand:
    def __init__(self):
        self.mode_pr = 99
        self.mode_machine = 99
        self.motor_cmd = [FakeMotor() for _ in range(35)]
        self.crc = 0


class FakeCrc:
    def __init__(self):
        self.calls = []

    def Crc(self, command):
        self.calls.append(command)
        return 54321


class FakeDds:
    def __init__(self, event_log=None):
        self.publisher_calls = 0
        self.publisher = None
        self.commands = []
        self.event_log = event_log

    def create_publisher(self):
        if self.event_log is not None:
            self.event_log.append("create_publisher")
        self.publisher_calls += 1
        self.publisher = FakePublisher()
        return self.publisher

    def make_low_cmd(self, target, config, state):
        command = FakeCommand()
        command.mode_pr = 0
        command.mode_machine = state.mode_machine
        for motor in command.motor_cmd:
            motor.mode = 0
            motor.q = motor.dq = motor.tau = motor.kp = motor.kd = 0.0
            motor.reserve = 0
        for index in range(29):
            motor = command.motor_cmd[index]
            motor.mode = 1
            motor.q = float(target[index])
            motor.kp = float(config.kp[index])
            motor.kd = float(config.kd[index])
        self.commands.append(command)
        return command

    @staticmethod
    def write(publisher, command):
        return publisher.Write(command)


class FakeKungFuPolicy:
    def __init__(self, event_log=None):
        self.config = SimpleNamespace(
            kp=np.full(29, 33.0, dtype=np.float32),
            kd=np.full(29, 3.0, dtype=np.float32),
        )
        self.reference = SimpleNamespace(frame=0, source_frames=15793)
        self.reference_is_frozen = False
        self.event_log = event_log
        self.start_calls = []
        self.infer_calls = []

    def start(self, *, base_quat, joint_pos):
        if self.event_log is not None:
            self.event_log.append("kungfu_start")
        self.start_calls.append(
            (np.asarray(base_quat).copy(), np.asarray(joint_pos).copy())
        )

    def infer(self, *, base_quat, base_ang_vel, joint_pos, joint_vel):
        self.infer_calls.append(
            (
                np.asarray(base_quat).copy(),
                np.asarray(base_ang_vel).copy(),
                np.asarray(joint_pos).copy(),
                np.asarray(joint_vel).copy(),
            )
        )
        return SimpleNamespace(
            target_joint_pos=np.full(29, 0.125, dtype=np.float32)
        )


@unittest.skipUnless(hasattr(socket, "SOCK_SEQPACKET"), "requires SOCK_SEQPACKET")
class WorkerHandoffTests(unittest.TestCase):
    def test_writer_free_policy_selection_changes_the_next_resident_episode(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        kungfu = FakeKungFuPolicy()
        host_runner = RecordingRunner(
            "host_selected",
            [np.zeros(23, dtype=np.float32) for _ in range(200)],
        )
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(host_runner,),
            fallback_after_s=8.0,
        )
        resident_manifest = (
            {
                "name": "host:test",
                "execution_provider": "CPUExecutionProvider",
                "warmed": True,
            },
            {
                "name": "kungfu:test",
                "execution_provider": "CPUExecutionProvider",
                "warmed": True,
            },
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    kungfu_policy=kungfu,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    initial_controller="kungfu",
                    resident_policies=resident_manifest,
                )
            )
        )

        def receive_event(expected):
            while True:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                if packet["event"] == expected:
                    return packet

        thread.start()
        try:
            ready = receive_event("READY_NO_WRITER")
            self.assertEqual(ready["selected_policy_id"], "kungfu")
            supervisor.send(
                json.dumps(
                    {
                        "schema": "matrix.sonic_host_worker.control.v1",
                        "command": "SELECT_POLICY",
                        "slot": "recovery",
                        "policy_id": "host",
                        "transition_id": "slot-test-1",
                    }
                ).encode("utf-8")
            )
            selected = receive_event("POLICY_SELECTED")
            self.assertEqual(selected["policy_id"], "host")
            self.assertEqual(selected["previous_policy_id"], "kungfu")
            self.assertTrue(selected["models_reused"])
            self.assertFalse(selected["writer_active"])

            supervisor.send(
                json.dumps(
                    {
                        "schema": "matrix.sonic_host_worker.control.v1",
                        "command": "GO",
                        "episode_id": 1,
                    }
                ).encode("utf-8")
            )
            receive_event("FIRST_WRITE")
            self.assertGreaterEqual(len(host_runner.inputs), 1)
            self.assertEqual(kungfu.start_calls, [])
            supervisor.send(b"STOP")
            receive_event("STOPPED")
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
        finally:
            supervisor.close()
            child.close()

    def test_pause_then_go_reuses_resident_worker_and_policy(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        event_log = []
        dds = FakeDds(event_log)
        kungfu = FakeKungFuPolicy(event_log)
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("unused_host", [np.zeros(23)]),),
            fallback_after_s=8.0,
        )
        resident_manifest = (
            {
                "name": "host:test",
                "execution_provider": "CPUExecutionProvider",
                "warmed": True,
            },
            {
                "name": "kungfu:test",
                "execution_provider": "CPUExecutionProvider",
                "warmed": True,
            },
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    kungfu_policy=kungfu,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    initial_controller="kungfu",
                    resident_policies=resident_manifest,
                )
            )
        )

        def send(command, episode_id):
            supervisor.send(
                json.dumps(
                    {
                        "schema": "matrix.sonic_host_worker.control.v1",
                        "command": command,
                        "episode_id": episode_id,
                    }
                ).encode("utf-8")
            )

        def receive_event(expected):
            while True:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                if packet["event"] == expected:
                    return packet

        thread.start()
        try:
            ready = receive_event("READY_NO_WRITER")
            self.assertTrue(ready["models_loaded_once"])
            self.assertTrue(ready["models_warmed"])
            self.assertEqual(ready["resident_policy_count"], 2)
            self.assertEqual(ready["registered_policy_ids"], ["host", "kungfu"])
            self.assertEqual(ready["initial_policy_id"], "kungfu")

            send("GO", 1)
            first = receive_event("FIRST_WRITE")
            self.assertEqual(first["episode_id"], 1)
            first_publisher = dds.publisher
            self.assertEqual(len(kungfu.start_calls), 1)

            send("PAUSE", 1)
            paused = receive_event("PAUSED_RESIDENT_WRITER")
            self.assertEqual(paused["episode_id"], 1)
            self.assertTrue(paused["writer_created"])
            self.assertFalse(paused["write_authorized"])
            self.assertTrue(paused["writer_reused"])
            self.assertFalse(first_publisher.closed)
            self.assertTrue(thread.is_alive())

            state_store.set(snapshot())
            send("GO", 2)
            second = receive_event("FIRST_WRITE")
            self.assertEqual(second["episode_id"], 2)
            self.assertEqual(len(kungfu.start_calls), 2)
            self.assertEqual(dds.publisher_calls, 1)
            self.assertIs(dds.publisher, first_publisher)
            self.assertFalse(first_publisher.closed)
            self.assertTrue(thread.is_alive())

            send("PAUSE", 2)
            paused = receive_event("PAUSED_RESIDENT_WRITER")
            self.assertEqual(paused["episode_id"], 2)
            self.assertTrue(paused["writer_created"])
            self.assertFalse(paused["write_authorized"])
            self.assertTrue(paused["writer_reused"])
            self.assertFalse(first_publisher.closed)
            supervisor.send(b"STOP")
            receive_event("STOPPED")
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
            self.assertTrue(first_publisher.closed)
        finally:
            supervisor.close()
            child.close()

    def test_kungfu_starts_before_publisher_and_writes_29dof_pd(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        event_log = []
        dds = FakeDds(event_log)
        kungfu = FakeKungFuPolicy(event_log)
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("unused_host", [np.zeros(23)]),),
            fallback_after_s=8.0,
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    kungfu_policy=kungfu,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    initial_controller="kungfu",
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            ready = json.loads(supervisor.recv(4096).decode("utf-8"))
            self.assertEqual(ready["event"], "READY_NO_WRITER")
            self.assertEqual(event_log, [])
            supervisor.send(b"GO")
            packets = []
            while not any(packet["event"] == "FIRST_WRITE" for packet in packets):
                packets.append(json.loads(supervisor.recv(4096).decode("utf-8")))
            self.assertEqual(event_log[:2], ["kungfu_start", "create_publisher"])
            self.assertEqual(len(kungfu.start_calls), 1)
            self.assertGreaterEqual(len(kungfu.infer_calls), 1)
            command = dds.commands[0]
            self.assertTrue(all(motor.q == 0.125 for motor in command.motor_cmd[:29]))
            self.assertTrue(all(motor.kp == 33.0 for motor in command.motor_cmd[:29]))
            status = next(
                (packet for packet in packets if packet["event"] == "STATUS"),
                None,
            )
            if status is None:
                while status is None:
                    packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                    if packet["event"] == "STATUS":
                        status = packet
            self.assertEqual(status["controller"], worker.KUNGFU_GETUP_CONTROLLER)
            self.assertFalse(status["reference_frozen"])
            supervisor.send(b"STOP")
            packet = json.loads(supervisor.recv(4096).decode("utf-8"))
            while packet["event"] != "STOPPED":
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertEqual(result, [0])
            self.assertTrue(dds.publisher.closed)
        finally:
            supervisor.close()
            child.close()

    def test_writer_free_standby_emits_heartbeat_without_publisher(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(1.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("prone_v1", [np.zeros(23)]),),
            fallback_after_s=8.0,
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            ready = json.loads(supervisor.recv(4096).decode("utf-8"))
            self.assertEqual(ready["event"], "READY_NO_WRITER")
            heartbeat = json.loads(supervisor.recv(4096).decode("utf-8"))
            self.assertEqual(heartbeat["event"], "STATUS")
            self.assertEqual(heartbeat["controller"], "WRITER_FREE_STANDBY")
            self.assertFalse(heartbeat["writer_created"])
            self.assertEqual(dds.publisher_calls, 0)
            self.assertEqual(dds.commands, [])
            supervisor.send(b"STOP")
            packet = json.loads(supervisor.recv(4096).decode("utf-8"))
            while packet["event"] != "STOPPED":
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
        finally:
            supervisor.close()
            child.close()

    def test_no_publisher_or_write_before_go_then_stop_closes_writer(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        runner = RecordingRunner("prone_v1", [np.zeros(23)] * 20)
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(runner,),
            fallback_after_s=8.0,
        )
        result = []

        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=10.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            ready = json.loads(supervisor.recv(4096).decode("utf-8"))
            self.assertEqual(ready["event"], "READY_NO_WRITER")
            self.assertEqual(dds.publisher_calls, 0)
            self.assertEqual(dds.commands, [])

            supervisor.send(b"GO")
            deadline = time.monotonic() + 1.0
            events = []
            while "FIRST_WRITE" not in events and time.monotonic() < deadline:
                events.append(json.loads(supervisor.recv(4096))["event"])
            self.assertIn("FIRST_WRITE", events)
            self.assertEqual(dds.publisher_calls, 1)
            self.assertGreaterEqual(len(dds.commands), 1)

            supervisor.send(b"STOP")
            stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            while stopped["event"] != "STOPPED":
                stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
            self.assertTrue(dds.publisher.closed)
        finally:
            supervisor.close()
            child.close()

    def test_supervisor_authorizes_fallback_without_creating_second_writer(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        first = RecordingRunner("prone_v1", [np.zeros(23)] * 200)
        second = RecordingRunner("prone_v2", [np.zeros(23)] * 200)
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(first, second),
            fallback_after_s=0.05,
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            self.assertEqual(
                json.loads(supervisor.recv(4096).decode("utf-8"))["event"],
                "READY_NO_WRITER",
            )
            supervisor.send(b"GO")
            events = []
            fallback_due = None
            while fallback_due is None:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                events.append(packet["event"])
                if packet["event"] == "POLICY_FALLBACK_DUE":
                    fallback_due = packet
            self.assertIn("FIRST_WRITE", events)
            self.assertTrue(fallback_due["requires_supervisor_authorization"])
            self.assertEqual(dds.publisher_calls, 1)

            supervisor.send(b"ADVANCE_POLICY")
            switched = None
            while switched is None:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                if packet["event"] == "POLICY_SWITCH":
                    switched = packet
            self.assertEqual(switched["from_policy"], "prone_v1")
            self.assertEqual(switched["to_policy"], "prone_v2")
            self.assertEqual(dds.publisher_calls, 1)

            supervisor.send(b"STOP")
            stopped = None
            while stopped is None or stopped["event"] != "STOPPED":
                stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
        finally:
            supervisor.close()
            child.close()

    def test_joint_pose_hold_reuses_writer_and_holds_measured_q(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        measured = snapshot(joint_vel_rad_s=np.zeros(29))
        state_store.set(measured)
        dds = FakeDds()
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("prone_v1", [np.zeros(23)] * 200),),
            fallback_after_s=8.0,
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=20.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            self.assertEqual(
                json.loads(supervisor.recv(4096).decode("utf-8"))["event"],
                "READY_NO_WRITER",
            )
            supervisor.send(b"GO")
            event = None
            while event != "FIRST_WRITE":
                event = json.loads(supervisor.recv(4096).decode("utf-8"))["event"]

            measured = snapshot(joint_vel_rad_s=np.zeros(29))
            state_store.set(measured)
            supervisor.send(b"ENTER_JOINT_HOLD")
            held = None
            while held is None:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                if packet["event"] == "JOINT_HOLD_FIRST_WRITE":
                    held = packet
            self.assertTrue(held["writer_reused"])
            self.assertTrue(held["measured_joint_target"])
            self.assertEqual(held["measured_joint_count"], 29)
            self.assertTrue(held["capture_once"])
            self.assertTrue(held["target_velocity_zero"])
            self.assertTrue(held["feedforward_torque_zero"])
            self.assertLessEqual(
                held["lowstate_capture_age_s"],
                worker.JOINT_HOLD_CAPTURE_MAX_AGE_S,
            )
            self.assertEqual(dds.publisher_calls, 1)
            for index in range(29):
                self.assertAlmostEqual(
                    dds.commands[-1].motor_cmd[index].q,
                    float(measured.joint_pos_rad[index]),
                )

            command_count = len(dds.commands)
            state_store.set(
                snapshot(marker=10.0, joint_vel_rad_s=np.zeros(29))
            )
            deadline = time.monotonic() + 1.0
            while len(dds.commands) == command_count and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertGreater(len(dds.commands), command_count)
            for index in range(29):
                self.assertAlmostEqual(
                    dds.commands[-1].motor_cmd[index].q,
                    float(measured.joint_pos_rad[index]),
                )

            supervisor.send(b"STOP")
            stopped = None
            while stopped is None or stopped["event"] != "STOPPED":
                stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
        finally:
            supervisor.close()
            child.close()

    def test_enter_amp_hold_reuses_writer_and_acks_only_after_amp_write(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        host_runner = RecordingRunner("prone_v1", [np.zeros(23)] * 200)
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(host_runner,),
            fallback_after_s=8.0,
        )
        amp_runner = RecordingRunner("amp_hold", [np.zeros(29)] * 200)
        amp_config = worker.PolicyConfig.from_mapping(amp_config_mapping())
        amp_policy = worker.AmpPolicyCore(amp_config, amp_runner)
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=amp_policy,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=3.0,
                    status_hz=20.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            ready = json.loads(supervisor.recv(4096).decode("utf-8"))
            self.assertEqual(ready["event"], "READY_NO_WRITER")
            supervisor.send(b"GO")
            event = None
            while event != "FIRST_WRITE":
                event = json.loads(supervisor.recv(4096).decode("utf-8"))["event"]
            self.assertEqual(dds.publisher_calls, 1)
            publisher = dds.publisher
            self.assertFalse(publisher.closed)
            self.assertAlmostEqual(
                dds.commands[-1].motor_cmd[0].kp,
                float(cascade.config.kp[0]),
            )

            supervisor.send(b"ENTER_AMP_HOLD")
            hold_event = None
            while hold_event is None or hold_event["event"] != "AMP_HOLD_FIRST_WRITE":
                hold_event = json.loads(supervisor.recv(4096).decode("utf-8"))

            self.assertEqual(dds.publisher_calls, 1)
            self.assertIs(dds.publisher, publisher)
            self.assertFalse(publisher.closed)
            self.assertTrue(hold_event["writer_reused"])
            self.assertTrue(hold_event["host_target_cleared"])
            self.assertTrue(hold_event["history_reset_from_latest_lowstate"])
            self.assertEqual(hold_event["command"], [0.0, 0.0, 0.0])
            amp_command = dds.commands[-1]
            self.assertAlmostEqual(
                amp_command.motor_cmd[0].kp,
                float(amp_config.kp[0]),
            )
            np.testing.assert_allclose(
                [amp_command.motor_cmd[index].q for index in range(29)],
                amp_config.default_joint_pos,
            )
            # reset_history duplicated one latest LowState frame four times and
            # cleared PrevActions before the first zero-command inference.
            first_amp_observation = amp_runner.inputs[0].reshape(4, 96)
            for index in range(1, 4):
                np.testing.assert_array_equal(
                    first_amp_observation[0], first_amp_observation[index]
                )
            np.testing.assert_array_equal(
                first_amp_observation[-1, 67:96], np.zeros(29)
            )

            supervisor.send(b"STOP")
            stopped = None
            while stopped is None or stopped["event"] != "STOPPED":
                stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
            self.assertTrue(publisher.closed)
        finally:
            supervisor.close()
            child.close()

    def test_amp_first_recovery_gets_up_and_holds_with_one_writer(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("unused_host", [np.zeros(23)] * 100),),
            fallback_after_s=8.0,
        )
        amp_runner = RecordingRunner("amp_getup", [np.zeros(29)] * 200)
        amp_config = worker.PolicyConfig.from_mapping(amp_config_mapping())
        amp_policy = worker.AmpPolicyCore(amp_config, amp_runner)
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=amp_policy,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=3.0,
                    status_hz=20.0,
                    initial_controller="amp",
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            self.assertEqual(
                json.loads(supervisor.recv(4096).decode("utf-8"))["event"],
                "READY_NO_WRITER",
            )
            supervisor.send(b"GO")
            packets = []
            while not any(packet["event"] == "FIRST_WRITE" for packet in packets):
                packets.append(json.loads(supervisor.recv(4096).decode("utf-8")))

            self.assertEqual(dds.publisher_calls, 1)
            self.assertGreaterEqual(len(dds.commands), 1)
            self.assertAlmostEqual(
                dds.commands[-1].motor_cmd[0].kp,
                float(amp_config.kp[0]),
            )
            self.assertGreaterEqual(len(amp_runner.inputs), 1)

            supervisor.send(b"ENTER_AMP_HOLD")
            hold_event = None
            while hold_event is None or hold_event["event"] != "AMP_HOLD_FIRST_WRITE":
                hold_event = json.loads(supervisor.recv(4096).decode("utf-8"))

            self.assertEqual(dds.publisher_calls, 1)
            self.assertEqual(
                hold_event["previous_controller"], worker.AMP_GETUP_CONTROLLER
            )
            self.assertFalse(hold_event["host_target_cleared"])
            self.assertTrue(hold_event["writer_reused"])
            self.assertFalse(hold_event["history_reset_from_latest_lowstate"])

            supervisor.send(b"STOP")
            stopped = None
            while stopped is None or stopped["event"] != "STOPPED":
                stopped = json.loads(supervisor.recv(4096).decode("utf-8"))
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [0])
            self.assertTrue(dds.publisher.closed)
        finally:
            supervisor.close()
            child.close()

    def test_enter_amp_hold_without_preload_fails_closed(self):
        supervisor, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        supervisor.settimeout(2.0)
        state_store = worker.LatestLowState()
        state_store.set(snapshot())
        dds = FakeDds()
        cascade = worker.HostPolicyCascade(
            config=worker.HostControlConfig.create(),
            runners=(RecordingRunner("prone_v1", [np.zeros(23)] * 100),),
            fallback_after_s=8.0,
        )
        result = []
        thread = threading.Thread(
            target=lambda: result.append(
                worker.run_worker(
                    cascade=cascade,
                    amp_hold_policy=None,
                    dds=dds,
                    state_store=state_store,
                    control=child,
                    publish_hz=50.0,
                    lowstate_timeout_s=2.0,
                    status_hz=10.0,
                    resident_policies=TEST_RESIDENT_POLICIES,
                )
            )
        )
        thread.start()
        try:
            self.assertEqual(
                json.loads(supervisor.recv(4096).decode("utf-8"))["event"],
                "READY_NO_WRITER",
            )
            supervisor.send(b"GO")
            event = None
            while event != "FIRST_WRITE":
                event = json.loads(supervisor.recv(4096).decode("utf-8"))["event"]
            supervisor.send(b"ENTER_AMP_HOLD")
            events = []
            messages = []
            while "STOPPED" not in events:
                packet = json.loads(supervisor.recv(4096).decode("utf-8"))
                events.append(packet["event"])
                if packet["event"] == "ERROR":
                    messages.append(packet["message"])
            thread.join(timeout=1.0)
            self.assertFalse(thread.is_alive())
            self.assertEqual(result, [2])
            self.assertIn("AMP hold policy was not preloaded", messages)
            self.assertTrue(dds.publisher.closed)
            self.assertEqual(dds.publisher_calls, 1)
        finally:
            supervisor.close()
            child.close()

    def test_host_protocol_accepts_shared_amp_schema(self):
        packet = json.dumps(
            {"schema": worker.AMP_CONTROL_SCHEMA, "command": "GO"}
        ).encode("utf-8")
        self.assertEqual(worker.decode_command(packet), "GO")

    def test_host_protocol_accepts_amp_hold_transition_command(self):
        self.assertEqual(
            worker.decode_command(b"ENTER_AMP_HOLD"), "ENTER_AMP_HOLD"
        )
        packet = json.dumps(
            {
                "schema": worker.AMP_CONTROL_SCHEMA,
                "command": "ENTER_AMP_HOLD",
            }
        ).encode("utf-8")
        self.assertEqual(worker.decode_command(packet), "ENTER_AMP_HOLD")

    def test_host_protocol_accepts_joint_hold_transition_command(self):
        self.assertEqual(
            worker.decode_command(b"ENTER_JOINT_HOLD"), "ENTER_JOINT_HOLD"
        )

    def test_runtime_cli_defaults_to_official_play_action_rescale(self):
        args = worker.build_parser().parse_args(
            [
                "--model",
                "/models/prone.onnx",
                "--interface",
                "lo",
                "--control-socket",
                "/tmp/recovery.sock",
            ]
        )
        self.assertEqual(args.action_rescale, 0.30)

    def test_only_fixed_joint_hold_bridges_prewarmer_lowstate_stall(self):
        self.assertEqual(
            worker.effective_lowstate_timeout_s(
                worker.HOST_GETUP_CONTROLLER, 0.25
            ),
            0.25,
        )
        self.assertEqual(
            worker.effective_lowstate_timeout_s(
                worker.AMP_GETUP_CONTROLLER, 0.25
            ),
            0.25,
        )
        self.assertEqual(
            worker.effective_lowstate_timeout_s(
                worker.JOINT_POSE_HOLD_CONTROLLER, 0.25
            ),
            1.0,
        )
        self.assertEqual(
            worker.effective_lowstate_timeout_s(
                worker.JOINT_POSE_HOLD_CONTROLLER, 2.0
            ),
            2.0,
        )


class LowCommandTests(unittest.TestCase):
    def test_shared_dds_builder_zeros_35_slots_and_sets_crc(self):
        runtime = worker.UnitreeDdsRuntime.__new__(worker.UnitreeDdsRuntime)
        runtime._low_cmd_factory = FakeCommand
        runtime._crc = FakeCrc()
        config = worker.HostControlConfig.create()
        target = np.arange(29, dtype=np.float32) / 5.0

        command = runtime.make_low_cmd(target, config, snapshot())

        for index, motor in enumerate(command.motor_cmd):
            if index < 29:
                self.assertEqual(motor.mode, 1)
                self.assertAlmostEqual(motor.q, float(target[index]))
                self.assertAlmostEqual(motor.kp, float(config.kp[index]))
                self.assertAlmostEqual(motor.kd, float(config.kd[index]))
            else:
                self.assertEqual(
                    (motor.mode, motor.q, motor.dq, motor.tau, motor.kp, motor.kd),
                    (0, 0.0, 0.0, 0.0, 0.0, 0.0),
                )
            self.assertEqual(motor.reserve, 0)
        self.assertEqual(command.crc, 54321)
        self.assertEqual(runtime._crc.calls, [command])


if __name__ == "__main__":
    unittest.main()
