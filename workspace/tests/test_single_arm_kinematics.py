# pyright: reportMissingImports=false

import unittest

import numpy as np
from control.kinematics import (
    ArmGeometry,
    JointVector,
    PlanarArmKinematics,
    ToolPose,
)


class SingleArmKinematicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.geometry = ArmGeometry(
            calibration_id="unit-test",
            shoulder_height_mm=50.0,
            shoulder_radial_offset_mm=0.0,
            shoulder_lateral_offset_mm=0.0,
            upper_arm_mm=100.0,
            forearm_mm=100.0,
            tool_forward_mm=0.0,
            tool_normal_mm=0.0,
            joint_directions=(1, 1, 1, 1, 1),
            joint_zero_offsets_deg=(0.0, 0.0, 0.0, 0.0, 0.0),
            joint_limits_deg=((-180.0, 180.0),) * 5,
        )
        self.model = PlanarArmKinematics(self.geometry)

    def test_forward_uses_explicit_single_follower_five_axis_order(self) -> None:
        joints = JointVector((0.0, 0.0, -90.0, 90.0, 0.0))

        pose = self.model.forward(joints)

        self.assertAlmostEqual(pose.position_mm[0], 100.0, places=6)
        self.assertAlmostEqual(pose.position_mm[1], 0.0, places=6)
        self.assertAlmostEqual(pose.position_mm[2], -50.0, places=6)
        self.assertAlmostEqual(pose.pitch_deg, 0.0, places=6)
        self.assertAlmostEqual(pose.yaw_deg, 0.0, places=6)

    def test_tool_offset_keeps_forward_and_normal_components_separate(self) -> None:
        geometry = ArmGeometry(
            **{
                **self.geometry.as_dict(),
                "tool_forward_mm": 20.0,
                "tool_normal_mm": 10.0,
            }
        )
        model = PlanarArmKinematics(geometry)

        pose = model.forward(JointVector((0.0, 0.0, 0.0, 0.0, 0.0)))

        self.assertEqual(pose.position_mm, (220.0, 0.0, 60.0))

    def test_inverse_round_trip_preserves_tcp_pitch_and_base_yaw(self) -> None:
        expected = JointVector((30.0, 25.0, -70.0, 35.0, 20.0))
        pose = self.model.forward(expected)

        actual = self.model.inverse(pose)
        replay = self.model.forward(actual)

        for observed, target in zip(replay.position_mm, pose.position_mm, strict=True):
            self.assertAlmostEqual(observed, target, places=6)
        self.assertAlmostEqual(replay.pitch_deg, pose.pitch_deg, places=6)
        self.assertAlmostEqual(replay.yaw_deg, pose.yaw_deg, places=6)

    def test_joint_vector_accepts_numeric_scalars_from_lerobot_bus(self) -> None:
        mapping = {
            feature: np.float32(index)
            for index, feature in enumerate(
                (
                    "shoulder_pan.pos",
                    "shoulder_lift.pos",
                    "elbow_flex.pos",
                    "wrist_flex.pos",
                    "wrist_roll.pos",
                )
            )
        }

        joints = JointVector.from_mapping(mapping)

        self.assertEqual(joints.values_deg, (0.0, 1.0, 2.0, 3.0, 4.0))

    def test_joint_vector_rejects_gripper_and_implicit_mapping_order(self) -> None:
        mapping = {
            "shoulder_pan.pos": 0.0,
            "shoulder_lift.pos": 0.0,
            "elbow_flex.pos": 0.0,
            "wrist_flex.pos": 0.0,
            "wrist_roll.pos": 0.0,
            "gripper.pos": 0.0,
        }

        with self.assertRaisesRegex(ValueError, "extra=.*gripper.pos"):
            JointVector.from_mapping(mapping)

    def test_inverse_rejects_unreachable_pose(self) -> None:
        pose = ToolPose(
            frame="robot_base_mm",
            position_mm=(500.0, 0.0, 50.0),
            pitch_deg=0.0,
            yaw_deg=0.0,
        )

        with self.assertRaisesRegex(ValueError, "不可达"):
            self.model.inverse(pose)


if __name__ == "__main__":
    unittest.main()
