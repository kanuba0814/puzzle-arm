# pyright: reportMissingImports=false

import unittest

from control.kinematics import JOINT_FEATURES
from integration.park_sequence import JointSpaceParkMover, ParkMoveErrorCode


PARK = {
    "shoulder_pan": -10.0,
    "shoulder_lift": -20.0,
    "elbow_flex": 30.0,
    "wrist_flex": 40.0,
    "wrist_roll": -80.0,
}
HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -2.0,
    "elbow_flex": 3.0,
    "wrist_flex": 4.0,
    "wrist_roll": -8.0,
}


class FakeRobot:
    def __init__(self, joints: dict[str, float], lag: float = 0.0) -> None:
        self.joints = {f"{name}.pos": value for name, value in joints.items()}
        self.lag = lag
        self.sent_actions: list[dict[str, float]] = []

    def get_observation(self) -> dict[str, float]:
        return dict(self.joints)

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        self.sent_actions.append(dict(action))
        self.joints = {name: value + self.lag for name, value in action.items()}
        return dict(action)


class ParkSequenceTest(unittest.TestCase):
    def _mover(self, robot: FakeRobot) -> JointSpaceParkMover:
        return JointSpaceParkMover(
            robot,
            park_deg=PARK,
            home_deg=HOME,
            park_tolerance_deg=4,
            follow_tolerance_deg=8,
            step_deg=1,
        )

    def test_park_to_home_preserves_park_pan_until_upright(self) -> None:
        robot = FakeRobot(PARK)
        result = self._mover(robot).park_to_home()
        self.assertTrue(result.success)
        pan = "shoulder_pan.pos"
        first_home_pan = next(
            index
            for index, action in enumerate(robot.sent_actions)
            if action[pan] != PARK["shoulder_pan"]
        )
        self.assertGreater(first_home_pan, 0)
        self.assertTrue(
            all(
                action[pan] == PARK["shoulder_pan"]
                for action in robot.sent_actions[:first_home_pan]
            )
        )
        self.assertEqual(set(robot.sent_actions[-1]), set(JOINT_FEATURES))

    def test_start_outside_park_is_rejected(self) -> None:
        shifted = dict(PARK)
        shifted["shoulder_pan"] += 4.1
        result = self._mover(FakeRobot(shifted)).park_to_home()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, ParkMoveErrorCode.PARK_MISMATCH)

    def test_following_error_is_rejected(self) -> None:
        result = self._mover(FakeRobot(PARK, lag=9)).park_to_home()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, ParkMoveErrorCode.PARK_TRANSITION_FAILED)

    def test_home_to_park_requires_final_park_tolerance(self) -> None:
        result = self._mover(FakeRobot(HOME, lag=5)).home_to_park()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, ParkMoveErrorCode.PARK_TRANSITION_FAILED)
        self.assertIn("单步跟随超时", result.message)


if __name__ == "__main__":
    unittest.main()
