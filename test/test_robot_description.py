from pathlib import Path
from unittest import TestCase

import numpy as np
import yaml

from onshape_to_robot.robot_description import (
    RobotSDF,
    RobotURDF,
)

DATA_PATH = Path(__file__).parent / "data" / "robot_description.yaml"
with DATA_PATH.open("r") as stream:
    TEST_CASES = yaml.load(stream, Loader=yaml.Loader)


class TestRobotURDF(TestCase):
    def setUp(self) -> None:
        self.maxDiff = None
        self.robot = RobotURDF("myrobot")
        self.data = TEST_CASES["urdf"]

    def assert_xml(self, test_case: str) -> None:
        self.assertEqual(self.robot.xml, self.data[test_case])

    def test_append_inertial(self):
        self.robot.append_inertial(
            mass=1.2,
            com=[1, 2, 3, 4, 5, 6],
            inertia=np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
        )
        self.assert_xml("inertia")

    def test_append_inertial_no_inertia(self) -> None:
        self.robot.append_inertial(mass=1.2)
        self.assert_xml("no_inertia")

    def test_append_material(self) -> None:
        self.robot.append_material("mymaterial", [1, 2, 3, 4])
        self.assert_xml("material")


class TestRobotSDF(TestCase):
    def setUp(self) -> None:
        self.maxDiff = None
        self.robot = RobotSDF("myrobot")
        self.data = TEST_CASES["sdf"]

    def assert_xml(self, test_case: str) -> None:
        self.assertEqual(self.robot.xml, self.data[test_case])

    def test_append_inertial(self) -> None:
        self.robot.append_inertial(
            1.2, inertia=np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        )
        self.assert_xml("inertia")

    def test_append_material(self) -> None:
        self.robot.append_material("mymaterial", [1, 2, 3, 4])
        self.assert_xml("material")

    def test_append_inertial_no_inertia(self) -> None:
        self.robot.append_inertial(mass=1.2)
        self.assert_xml("no_inertia")
