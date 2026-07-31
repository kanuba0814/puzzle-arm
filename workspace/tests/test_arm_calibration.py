# pyright: reportMissingImports=false

import copy
import unittest
from pathlib import Path

from control.calibration import ArmCalibration
from control.kinematics import JOINT_FEATURES


class ArmCalibrationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.valid = {
            "schema_version": 1,
            "calibration_id": "single-arm-test-v1",
            "robot_type": "so101_follower_5d",
            "source_arm": "so101_follower_modified_single_arm",
            "frame": "robot_base_mm",
            "joint_features": list(JOINT_FEATURES),
            "geometry_mm": {
                "shoulder_height": 50.0,
                "shoulder_radial_offset": 0.0,
                "shoulder_lateral_offset": 0.0,
                "upper_arm": 100.0,
                "forearm": 100.0,
                "tool_forward": 20.0,
                "tool_normal": 10.0,
            },
            "joint_mapping": {
                "directions": [1, 1, 1, 1, 1],
                "zero_offsets_deg": [0.0, 0.0, 0.0, 0.0, 0.0],
                "soft_limits_deg": [[-180.0, 180.0]] * 5,
            },
            "quality": {
                "status": "accepted",
                "fit_samples": 8,
                "holdout_samples": 4,
                "holdout_xy_rms_mm": 4.0,
                "holdout_xy_max_mm": 7.0,
                "holdout_z_max_abs_mm": 2.5,
            },
            "evidence_sha256": ["a" * 64, "b" * 64],
        }

    def test_accepted_single_follower_calibration_builds_geometry(self) -> None:
        calibration = ArmCalibration.from_mapping(self.valid)

        geometry = calibration.require_accepted_geometry()

        self.assertEqual(geometry.calibration_id, "single-arm-test-v1")
        self.assertEqual(geometry.tool_forward_mm, 20.0)
        self.assertEqual(geometry.tool_normal_mm, 10.0)

    def test_generated_single_arm_calibration_passes_real_quality_gate(self) -> None:
        calibration = ArmCalibration.load(
            Path(
                "workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"
            )
        )

        geometry = calibration.require_accepted_geometry()

        self.assertEqual(geometry.joint_directions, (-1, -1, -1, -1, -1))
        self.assertAlmostEqual(geometry.tool_forward_mm, 101.27)
        self.assertAlmostEqual(geometry.tool_normal_mm, 18.1)
        self.assertLessEqual(calibration.quality.holdout_xy_rms_mm, 5.0)
        self.assertLessEqual(calibration.quality.holdout_xy_max_mm, 8.0)
        self.assertLessEqual(calibration.quality.holdout_z_max_abs_mm, 3.0)

    def test_rejected_legacy_quality_cannot_enable_motion(self) -> None:
        rejected = copy.deepcopy(self.valid)
        rejected["quality"].update(
            {
                "status": "rejected",
                "holdout_xy_rms_mm": 9.011,
                "holdout_xy_max_mm": 11.421,
                "holdout_z_max_abs_mm": 5.52,
            }
        )

        calibration = ArmCalibration.from_mapping(rejected)

        with self.assertRaisesRegex(RuntimeError, "标定状态不是 accepted"):
            calibration.require_accepted_geometry()

    def test_claimed_accepted_file_still_must_meet_quality_thresholds(self) -> None:
        poor = copy.deepcopy(self.valid)
        poor["quality"]["holdout_xy_rms_mm"] = 5.1

        calibration = ArmCalibration.from_mapping(poor)

        with self.assertRaisesRegex(RuntimeError, "XY RMS"):
            calibration.require_accepted_geometry()

    def test_unknown_or_six_axis_fields_are_rejected(self) -> None:
        unknown = copy.deepcopy(self.valid)
        unknown["geometry_mm"]["tool_forward"] = "UNKNOWN"
        with self.assertRaisesRegex(ValueError, "tool_forward"):
            ArmCalibration.from_mapping(unknown)

        six_axis = copy.deepcopy(self.valid)
        six_axis["joint_features"].append("gripper.pos")
        with self.assertRaisesRegex(ValueError, "五轴字段"):
            ArmCalibration.from_mapping(six_axis)


if __name__ == "__main__":
    unittest.main()
