"""Bridge the legacy SONIC PICO robot-model import to decoupled_wbc."""

from __future__ import annotations

import importlib
from types import ModuleType
import sys


LEGACY_PACKAGE = "gear_sonic.data.robot_model.instantiation"
LEGACY_MODULE = f"{LEGACY_PACKAGE}.g1"
CURRENT_MODULE = "decoupled_wbc.control.robot_model.instantiation.g1"


if LEGACY_MODULE not in sys.modules:
    current = importlib.import_module(CURRENT_MODULE)
    parent = ModuleType(LEGACY_PACKAGE)
    parent.__path__ = []
    parent.g1 = current
    sys.modules[LEGACY_PACKAGE] = parent
    sys.modules[LEGACY_MODULE] = current
