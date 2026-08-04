"""Bridge the legacy SONIC PICO robot-model import to decoupled_wbc."""

from __future__ import annotations

import importlib
from pathlib import Path
from types import ModuleType
import sys

import cmeel_pth  # noqa: F401 - activates locked native Pinocchio packages.


LEGACY_PACKAGE = "gear_sonic.data.robot_model.instantiation"
LEGACY_MODULE = f"{LEGACY_PACKAGE}.g1"
CURRENT_MODULE = "decoupled_wbc.control.robot_model.instantiation.g1"


if LEGACY_MODULE not in sys.modules:
    current = importlib.import_module(CURRENT_MODULE)

    def instantiate_g1_robot_model(
        waist_location: str = "lower_body", high_elbow_pose: bool = False
    ):
        sonic_root = Path(current.__file__).resolve().parents[4]
        waist_location_enum = {
            "lower_body": current.WaistLocation.LOWER_BODY,
            "upper_body": current.WaistLocation.UPPER_BODY,
            "lower_and_upper_body": current.WaistLocation.LOWER_AND_UPPER_BODY,
        }[waist_location]
        elbow_pose_enum = (
            current.ElbowPose.HIGH if high_elbow_pose else current.ElbowPose.LOW
        )
        supplemental_info = current.G1SupplementalInfo(
            waist_location=waist_location_enum,
            elbow_pose=elbow_pose_enum,
        )
        return current.RobotModel(
            sonic_root
            / "decoupled_wbc/control/robot_model/model_data/g1/g1_29dof_with_hand.urdf",
            sonic_root / "gear_sonic/data/robot_model/model_data/g1",
            supplemental_info=supplemental_info,
        )

    current.instantiate_g1_robot_model = instantiate_g1_robot_model
    parent = ModuleType(LEGACY_PACKAGE)
    parent.__path__ = []
    parent.g1 = current
    sys.modules[LEGACY_PACKAGE] = parent
    sys.modules[LEGACY_MODULE] = current
