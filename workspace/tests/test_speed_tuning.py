from __future__ import annotations

import unittest

from control.motion_controller import MotionConfig


class SpeedTuningContractTest(unittest.TestCase):
    def test_phase_speed_and_dwell_are_explicit(self) -> None:
        config = MotionConfig(
            15.0,
            10.0,
            10.0,
            3.0,
            1,
            0.05,
            8.0,
            -20.0,
            -30.0,
            180.0,
            goal_velocity_by_profile={
                "EMPTY_NORMAL": 250,
                "EMPTY_SLOW": 180,
                "HOLDING_SLOW": 100,
            },
            waypoint_min_dwell_s_by_profile={
                "EMPTY_NORMAL": 0.15,
                "EMPTY_SLOW": 0.20,
                "HOLDING_SLOW": 0.30,
            },
        )

        self.assertEqual(config.goal_velocity_by_profile["EMPTY_NORMAL"], 250)
        self.assertEqual(config.goal_velocity_by_profile["EMPTY_SLOW"], 180)
        self.assertEqual(config.goal_velocity_by_profile["HOLDING_SLOW"], 100)
        self.assertEqual(config.waypoint_min_dwell_s_by_profile["HOLDING_SLOW"], 0.30)

    def test_negative_waypoint_dwell_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "不能小于零"):
            MotionConfig(
                15.0,
                10.0,
                10.0,
                3.0,
                1,
                0.05,
                8.0,
                -20.0,
                -30.0,
                180.0,
                waypoint_min_dwell_s_by_profile={"EMPTY_NORMAL": -0.01},
            )


if __name__ == "__main__":
    unittest.main()
