# pyright: reportMissingImports=false

import itertools
from itertools import pairwise
import math
import unittest

from control.kinematics import JointVector, ToolPose
from control.motion_controller import MotionConfig
from integration.sweep_planner import (
    PieceRef,
    plan_sweep_order,
    preflight_continuous_sweep,
    preflight_sweep_ik,
)


class SweepPlannerTest(unittest.TestCase):
    def test_four_piece_route_matches_brute_force_minimum_and_is_stable(self) -> None:
        pieces = (
            PieceRef("d", (4.0, 1.0)),
            PieceRef("b", (1.0, 3.0)),
            PieceRef("a", (2.0, 0.0)),
            PieceRef("c", (5.0, 5.0)),
        )
        start = (0.0, 0.0)
        actual = plan_sweep_order(pieces, start)

        def cost(route: tuple[PieceRef, ...]) -> float:
            points = (
                (start,) + tuple(piece.center_table_mm for piece in route) + (start,)
            )
            return sum(math.dist(left, right) for left, right in pairwise(points))

        expected_cost = min(cost(route) for route in itertools.permutations(pieces))
        self.assertAlmostEqual(cost(actual), expected_cost)
        self.assertEqual(actual, plan_sweep_order(pieces, start))

    def test_tie_breaks_by_piece_id(self) -> None:
        route = plan_sweep_order(
            (PieceRef("b", (1.0, 0.0)), PieceRef("a", (1.0, 0.0))), (0.0, 0.0)
        )
        self.assertEqual([piece.piece_id for piece in route], ["a", "b"])

    def test_preflight_aborts_before_any_action_when_any_pose_unreachable(self) -> None:
        class Kinematics:
            def __init__(self) -> None:
                self.calls = 0

            def inverse(self, pose: ToolPose) -> object:
                self.calls += 1
                if pose.position_mm[0] == 2.0:
                    raise ValueError("unreachable")
                return object()

        model = Kinematics()
        poses = [
            ToolPose("robot_base_mm", (1.0, 0.0, 20.0), 0.0, 0.0),
            ToolPose("robot_base_mm", (2.0, 0.0, 20.0), 0.0, 0.0),
        ]
        with self.assertRaisesRegex(ValueError, "unreachable"):
            preflight_sweep_ik(model, poses)
        self.assertEqual(model.calls, 2)

    def test_continuous_preflight_checks_interpolated_ik_and_joint_step(self) -> None:
        class Kinematics:
            def inverse(self, pose: ToolPose) -> JointVector:
                if 4.0 <= pose.position_mm[0] <= 6.0:
                    raise ValueError("interpolated obstacle")
                value = pose.position_mm[0]
                return JointVector((value, value, value, value, value))

        config = MotionConfig(2, 5, 10, 5, 1, 0.01, 1, -20, -30, 180)
        with self.assertRaisesRegex(ValueError, "连续段 0 路径点"):
            preflight_continuous_sweep(
                Kinematics(),
                config,
                (
                    ToolPose("robot_base_mm", (0, 0, 20), 0, 0),
                    ToolPose("robot_base_mm", (10, 0, 20), 0, 0),
                ),
            )

    def test_continuous_preflight_rejects_joint_step(self) -> None:
        class Kinematics:
            def inverse(self, pose: ToolPose) -> JointVector:
                value = 100.0 if pose.position_mm[0] >= 1.0 else 0.0
                return JointVector((value, value, value, value, value))

        config = MotionConfig(10, 5, 10, 5, 1, 0.01, 1, -20, -30, 180)
        with self.assertRaisesRegex(ValueError, "关节步长"):
            preflight_continuous_sweep(
                Kinematics(),
                config,
                (
                    ToolPose("robot_base_mm", (0, 0, 20), 0, 0),
                    ToolPose("robot_base_mm", (1, 0, 20), 0, 0),
                ),
            )


if __name__ == "__main__":
    unittest.main()
