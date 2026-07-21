from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "matrix_mujoco_contacts.py"
SPEC = importlib.util.spec_from_file_location("matrix_mujoco_contacts", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
contacts = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = contacts
SPEC.loader.exec_module(contacts)


@dataclass
class Contact:
    geom1: int
    geom2: int
    frame: tuple[float, ...]
    dist: float = 0.0


class FakeModel:
    # world=0, pelvis=1, foot=2, torso=3, scene platform=4
    nbody = 5
    body_parentid = (0, 0, 1, 1, 0)
    # foot geom, torso geom, world floor geom, external platform geom
    geom_bodyid = (2, 3, 0, 4)


class FakeData:
    def __init__(self, *values: Contact):
        self.contact = values
        self.ncon = len(values)


VERTICAL = (0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
HORIZONTAL = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)


class MuJoCoContactTests(unittest.TestCase):
    def supported(self, *values: Contact) -> bool:
        return contacts.has_external_foot_support(
            FakeModel(),
            FakeData(*values),
            foot_body_ids={2},
            robot_root_body_id=1,
        )

    def test_ground_contact_is_support(self):
        self.assertTrue(self.supported(Contact(0, 2, VERTICAL)))

    def test_external_platform_contact_is_support(self):
        self.assertTrue(self.supported(Contact(0, 3, VERTICAL)))

    def test_robot_self_contact_is_not_support(self):
        self.assertFalse(self.supported(Contact(0, 1, VERTICAL)))

    def test_wall_contact_is_not_support(self):
        self.assertFalse(self.supported(Contact(0, 2, HORIZONTAL)))

    def test_positive_gap_is_not_support(self):
        self.assertFalse(self.supported(Contact(0, 2, VERTICAL, dist=0.001)))

    def grounded(self, *values: Contact) -> bool:
        return contacts.has_external_ground_support(
            FakeModel(),
            FakeData(*values),
            robot_root_body_id=1,
        )

    def test_torso_on_floor_is_grounded_but_not_foot_support(self):
        torso_floor = Contact(1, 2, VERTICAL)
        self.assertTrue(self.grounded(torso_floor))
        self.assertFalse(self.supported(torso_floor))

    def test_self_collision_is_not_grounded(self):
        self.assertFalse(self.grounded(Contact(0, 1, VERTICAL)))

    def test_wall_contact_is_not_grounded(self):
        self.assertFalse(self.grounded(Contact(1, 2, HORIZONTAL)))


if __name__ == "__main__":
    unittest.main()
