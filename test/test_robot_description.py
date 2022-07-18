from unittest import TestCase

import numpy as np
from onshape_to_robot.robot_description import RobotSDF, RobotURDF


class TestRobotURDF(TestCase):
    def setUp(self):
        self.maxDiff = None
        self.robot = RobotURDF("myrobot")

    def assert_xml(self, text):
        self.assertEqual(self.robot.xml, text)

    def test_append_inertial(self):
        self.robot.append_inertial(
            mass=1.2,
            com=[1,2,3,4,5,6],
            inertia=np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]]),
        )
        self.assert_xml(
            """\
<robot name="myrobot">
<inertial>
  <origin xyz="1.00000 2.00000 3.00000"/>
  <mass value="1.20000"/>
  <inertia ixx="1.00000" ixy="2.00000" ixz="3.00000" iyy="5.00000" iyz="6.00000" izz="9.00000"/>
</inertial>
"""
        )

    def test_append_inertial_no_inertia(self):
        self.robot.append_inertial(
            mass=1.2,
        )
        self.assert_xml(
            """\
<robot name="myrobot">
<inertial>
  <mass value="1.20000"/>
  <inertia ixx="0.00000" ixy="0.00000" ixz="0.00000" iyy="0.00000" iyz="0.00000" izz="0.00000"/>
</inertial>
"""
        )

    def test_append_material(self):
        self.robot.append_material("mymaterial", [1, 2, 3, 4])
        self.assert_xml(
            """\
<robot name="myrobot">
<material name="mymaterial_material">
  <color rgba="1.000 2.000 3.000 4.000"/>
  <script>
    <name>myrobot/mymaterial</name>
  </script>
</material>
"""
        )


class TestRobotSDF(TestCase):
    def setUp(self):
        self.maxDiff = None
        self.robot = RobotSDF("myrobot")

    def assert_xml(self, text):
        self.assertEqual(self.robot.xml, text)

    def test_append_inertial(self):
        self.robot.append_inertial(
            1.2, inertia=np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        )
        self.assert_xml(
            """\
<sdf version="1.6">
<model name="myrobot">
<inertial>
  <mass>1.20000</mass>
  <inertia>
    <ixx>1.00000</ixx>
    <ixy>4.00000</ixy>
    <ixz>7.00000</ixz>
    <iyy>5.00000</iyy>
    <iyz>8.00000</iyz>
    <izz>9.00000</izz>
  </inertia>
</inertial>
"""
        )

    def test_append_material(self):
        self.robot.append_material("mymaterial", [1, 2, 3, 4])
        self.assert_xml(
            """\
<sdf version="1.6">
<model name="myrobot">
<material name="mymaterial_material">
  <ambient>1.000 2.000 3.000 4.000</ambient>
  <diffuse>1.000 2.000 3.000 4.000</diffuse>
  <script>
    <name>myrobot/mymaterial</name>
  </script>
</material>
"""
        )

    def test_append_inertial_no_inertia(self):
        self.robot.append_inertial(
            mass=1.2,
        )
        self.assert_xml(
            """\
<sdf version="1.6">
<model name="myrobot">
<inertial>
  <mass>1.20000</mass>
  <inertia>
    <ixx>0.00000</ixx>
    <ixy>0.00000</ixy>
    <ixz>0.00000</ixz>
    <iyy>0.00000</iyy>
    <iyz>0.00000</iyz>
    <izz>0.00000</izz>
  </inertia>
</inertial>
"""
        )
