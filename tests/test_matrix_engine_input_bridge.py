from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if os.fspath(SCRIPTS) not in sys.path:
    sys.path.insert(0, os.fspath(SCRIPTS))

import matrix_engine_input_bridge as MODULE  # noqa: E402


class MatrixEngineInputBridgeTest(unittest.TestCase):
    def test_linux_abi_struct_sizes_are_frozen(self) -> None:
        self.assertEqual(MODULE._UINPUT_SETUP.size, 92)
        self.assertEqual(MODULE._UINPUT_ABS_SETUP.size, 28)
        self.assertEqual(MODULE._INPUT_EVENT.size, 24)

    def test_packet_decoder_rejects_duplicates_and_nonfinite_json(self) -> None:
        valid = {
            "protocol": MODULE.PROTOCOL,
            "sequence": 1,
            "capability": "a" * 64,
            "action": "status",
            "payload": {},
        }
        self.assertEqual(
            MODULE._decode_packet(
                json.dumps(valid, separators=(",", ":")).encode("utf-8")
            ),
            valid,
        )
        with self.assertRaisesRegex(MODULE.EngineInputError, "duplicate"):
            MODULE._decode_packet(
                (
                    '{"protocol":"matrix-engine-input/v1","protocol":"x",'
                    '"sequence":1,"capability":"x","action":"status",'
                    '"payload":{}}'
                ).encode("utf-8")
            )
        with self.assertRaisesRegex(
            MODULE.EngineInputError,
            "invalid JSON constant",
        ):
            MODULE._decode_packet(
                (
                    '{"protocol":"matrix-engine-input/v1","sequence":1,'
                    '"capability":"x","action":"mouse",'
                    '"payload":{"dx":NaN}}'
                ).encode("utf-8")
            )

    def test_action_validators_are_exact_and_bounded(self) -> None:
        self.assertEqual(
            MODULE._validate_mouse(
                {
                    "dx": 12.4,
                    "dy": -3.6,
                    "button": "left",
                    "seconds": 0.08,
                }
            ),
            {
                "dx": 12,
                "dy": -4,
                "button": "left",
                "seconds": 0.08,
            },
        )
        key = MODULE._validate_key(
            {
                "key": "w",
                "modifiers": ["shift"],
                "seconds": 0.5,
                "double": True,
                "tap_gap": 0.08,
            }
        )
        self.assertEqual(key["modifiers"], ("shift",))
        gamepad = MODULE._validate_gamepad(
            {
                "axes": {
                    "forward": 1.0,
                    "right": -0.5,
                    "look_yaw": 0.25,
                    "look_pitch": -0.25,
                },
                "buttons": ["south", "start"],
                "seconds": 0.5,
            }
        )
        self.assertEqual(gamepad["buttons"], ("south", "start"))
        with self.assertRaisesRegex(MODULE.EngineInputError, "payload fields"):
            MODULE._validate_mouse(
                {
                    "dx": 0.0,
                    "dy": 0.0,
                    "button": None,
                    "seconds": 0.1,
                    "extra": True,
                }
            )
        with self.assertRaisesRegex(MODULE.EngineInputError, "modifier"):
            MODULE._validate_key(
                {
                    "key": "w",
                    "modifiers": ["shift", "shift"],
                    "seconds": 0.5,
                    "double": False,
                    "tap_gap": 0.08,
                }
            )

    def test_capability_requires_private_user_owned_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "engine.cap"
            path.write_text("b" * 64 + "\n", encoding="ascii")
            path.chmod(0o600)
            self.assertEqual(
                MODULE._private_capability(path, os.getuid()),
                "b" * 64,
            )
            path.chmod(0o640)
            with self.assertRaisesRegex(PermissionError, "private owned"):
                MODULE._private_capability(path, os.getuid())

    def test_controller_releases_every_held_input(self) -> None:
        devices: list[mock.Mock] = []

        def device(*_args, **_kwargs):
            value = mock.Mock()
            devices.append(value)
            return value

        with mock.patch.object(MODULE, "UInputDevice", side_effect=device):
            controller = MODULE.EngineInputController(Path("/dev/uinput"))
        controller._sleep = mock.Mock()

        controller.mouse(dx=20, dy=-5, button="left", seconds=0.08)
        controller.key(
            key="w",
            modifiers=("shift",),
            seconds=0.5,
            double=True,
            tap_gap=0.08,
        )
        controller.gamepad(
            axes={
                "forward": 1.0,
                "right": -0.5,
                "look_yaw": 0.25,
                "look_pitch": -0.25,
            },
            buttons=("south",),
            seconds=0.5,
        )

        pointer, gamepad = devices
        pointer.emit.assert_any_call(MODULE.EV_REL, MODULE.REL_X, 20)
        pointer.emit.assert_any_call(MODULE.EV_KEY, MODULE.BTN_LEFT, 1)
        pointer.emit.assert_any_call(MODULE.EV_KEY, MODULE.BTN_LEFT, 0)
        pointer.emit.assert_any_call(MODULE.EV_KEY, MODULE.KEY_W, 1)
        pointer.emit.assert_any_call(MODULE.EV_KEY, MODULE.KEY_W, 0)
        self.assertEqual(
            pointer.method_calls[:5],
            [
                mock.call.emit(MODULE.EV_KEY, MODULE.BTN_LEFT, 1),
                mock.call.sync(),
                mock.call.emit(MODULE.EV_REL, MODULE.REL_X, 20),
                mock.call.emit(MODULE.EV_REL, MODULE.REL_Y, -5),
                mock.call.sync(),
            ],
        )
        self.assertEqual(
            controller._sleep.call_args_list[:2],
            [
                mock.call(MODULE.MOUSE_PRESS_LEAD_SECONDS),
                mock.call(0.08),
            ],
        )
        gamepad.emit.assert_any_call(
            MODULE.EV_ABS,
            MODULE.ABS_Y,
            -32767,
        )
        gamepad.emit.assert_any_call(
            MODULE.EV_KEY,
            MODULE.BTN_SOUTH,
            1,
        )
        gamepad.emit.assert_any_call(
            MODULE.EV_KEY,
            MODULE.BTN_SOUTH,
            0,
        )
        self.assertEqual(controller._pointer_pressed, set())
        self.assertEqual(controller._gamepad_pressed, set())
        self.assertTrue(all(value == 0 for value in controller._axis_values.values()))
        self.assertEqual(controller.actions, 3)


if __name__ == "__main__":
    unittest.main()
