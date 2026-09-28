# pyright: reportMissingImports=false

import time
import unittest
from pathlib import Path

from control.kinematics import ArmGeometry, JointVector, PlanarArmKinematics
from control.motion_controller import (
    MotionConfig,
    MotionController,
    MotionErrorCode,
    MotionGoal,
)


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds

    def timestamp_ns(self) -> int:
        return int(self.now * 1_000_000_000)


class FakeRobot:
    def __init__(self, observations: list[dict[str, float]]) -> None:
        if not observations:
            raise ValueError("FakeRobot requires at least one observation")
        self.observations = list(observations)
        self.last_observation = dict(observations[-1])
        self.sent_actions: list[dict[str, float]] = []
        self.sent_velocities: list[int | None] = []

    def get_observation(self) -> dict[str, float]:
        if self.observations:
            self.last_observation = self.observations.pop(0)
        return dict(self.last_observation)

    def send_action(
        self, action: dict[str, float], *, goal_velocity: int | None = None
    ) -> dict[str, float]:
        self.sent_actions.append(dict(action))
        self.sent_velocities.append(goal_velocity)
        return dict(action)


class MotionControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        geometry = ArmGeometry(
            calibration_id="accepted-test-calibration",
            shoulder_height_mm=50.0,
            shoulder_radial_offset_mm=0.0,
            shoulder_lateral_offset_mm=0.0,
            upper_arm_mm=100.0,
            forearm_mm=100.0,
            tool_forward_mm=0.0,
            tool_normal_mm=0.0,
            joint_directions=(1, 1, 1, 1, 1),
            joint_zero_offsets_deg=(0.0, 0.0, 0.0, 0.0, 0.0),
            joint_limits_deg=(
                (-180.0, 180.0),
                (-180.0, 180.0),
                (-180.0, 180.0),
                (-180.0, 180.0),
                (-180.0, 180.0),
            ),
        )
        self.model = PlanarArmKinematics(geometry)
        self.clock = ManualClock()
        self.config = MotionConfig(
            max_cartesian_step_mm=1000.0,
            max_orientation_step_deg=180.0,
            max_joint_step_deg=180.0,
            joint_tolerance_deg=0.2,
            stable_samples=2,
            poll_interval_s=0.01,
            waypoint_timeout_s=0.05,
            safe_min_z_mm=-40.0,
            contact_min_z_mm=-60.0,
            max_z_mm=250.0,
        )
        self.start = JointVector((0.0, 0.0, -90.0, 90.0, 0.0))

    def _goal(
        self,
        position_mm: tuple[float, float, float],
        phase: str = "PICK",
    ) -> MotionGoal:
        return MotionGoal(
            goal_id="goal-1",
            timestamp_ns=0,
            frame="robot_base_mm",
            position_mm=position_mm,
            pitch_deg=0.0,
            yaw_deg=0.0,
            phase=phase,
            speed_profile="EMPTY_SLOW",
        )

    def _controller(
        self, robot: FakeRobot, config: MotionConfig | None = None
    ) -> MotionController:
        return MotionController(
            robot=robot,
            kinematics=self.model,
            config=config or self.config,
            monotonic=self.clock.monotonic,
            sleep=self.clock.sleep,
            timestamp_ns=self.clock.timestamp_ns,
        )

    def test_default_deadline_uses_monotonic_clock_domain(self) -> None:
        robot = FakeRobot([self.start.as_mapping()])
        goal = MotionGoal(
            **{
                **self._goal((500.0, 0.0, 50.0)).__dict__,
                "timestamp_ns": time.monotonic_ns(),
                "deadline_ns": time.monotonic_ns() + 1_000_000_000,
            }
        )
        controller = MotionController(robot, self.model, self.config)

        result = controller.move(goal)

        self.assertEqual(result.error_code, MotionErrorCode.IK_NO_SOLUTION)
        self.assertNotEqual(result.error_code, MotionErrorCode.GLOBAL_TIMEOUT)
        self.assertEqual(robot.sent_actions, [])

    def test_unreachable_goal_is_rejected_before_any_servo_command(self) -> None:
        robot = FakeRobot([self.start.as_mapping()])

        result = self._controller(robot).move(self._goal((500.0, 0.0, 50.0)))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.IK_NO_SOLUTION)
        self.assertEqual(robot.sent_actions, [])
        self.assertEqual(result.executed_waypoints, 0)

    def test_entire_cartesian_path_is_preflighted_before_first_command(self) -> None:
        config = MotionConfig(
            **{
                **self.config.as_dict(),
                "max_cartesian_step_mm": 20.0,
                "max_joint_step_deg": 360.0,
            }
        )
        robot = FakeRobot([self.start.as_mapping()])

        result = self._controller(robot, config).move(self._goal((-100.0, 0.0, -50.0)))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.IK_NO_SOLUTION)
        self.assertEqual(robot.sent_actions, [])

    def test_safe_phase_below_safe_height_is_rejected_before_motion(self) -> None:
        robot = FakeRobot([self.start.as_mapping()])

        result = self._controller(robot).move(
            self._goal((100.0, 0.0, -50.0), phase="SAFE")
        )

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.SAFETY_REJECTED)
        self.assertEqual(robot.sent_actions, [])

    def test_joint_step_limit_is_reported_as_safety_rejection(self) -> None:
        """关节步长超限 → 不拒绝，而是翻倍加密重试直到满足（AGENTS.md 要求）。

        原始测试期望 SAFETY_REJECTED，但实现已改为超限加密重试（上限 4 轮），
        这是实机验证过的正确行为（大 Δyaw 路径必须加密而非拒绝）。
        直接验证 _plan：初始步数单步 >1°，加密后全部 waypoint 单步 <1°。
        """
        config = MotionConfig(
            **{
                **self.config.as_dict(),
                "max_joint_step_deg": 1.0,
            }
        )
        target = JointVector((0.0, 5.0, -90.0, 85.0, 0.0))
        robot = FakeRobot([self.start.as_mapping()])
        controller = self._controller(robot, config)

        plan = controller._plan(
            self._goal(self.model.forward(target).position_mm), self.start
        )

        # 加密后相邻 waypoint 单步必须全部 <= 1°
        previous = self.start
        for waypoint in plan.waypoints:
            self.assertLessEqual(waypoint.max_abs_error(previous), 1.0 + 1e-6)
            previous = waypoint
        self.assertGreaterEqual(len(plan.waypoints), 5)

    def test_success_requires_stable_actual_readback(self) -> None:
        target = JointVector((0.0, 2.0, -90.0, 88.0, 0.0))
        target_pose = self.model.forward(target)
        near_target = JointVector((0.0, 2.5, -90.0, 88.0, 0.0))
        robot = FakeRobot(
            [
                self.start.as_mapping(),
                near_target.as_mapping(),
                target.as_mapping(),
                target.as_mapping(),
            ]
        )

        result = self._controller(robot).move(self._goal(target_pose.position_mm))

        self.assertTrue(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.NONE)
        self.assertEqual(result.planned_waypoints, 1)
        self.assertEqual(result.executed_waypoints, 1)
        self.assertEqual(len(robot.sent_actions), 1)
        self.assertEqual(result.final_joint_deg, target.as_mapping())

    def test_intermediate_waypoint_min_dwell_prevents_immediate_burst(self) -> None:
        robot = FakeRobot([self.start.as_mapping()])
        controller = self._controller(robot)

        converged = controller._wait_for_target(
            self.start,
            tolerance_deg=0.2,
            stable_samples=1,
            minimum_dwell_s=0.03,
        )

        self.assertEqual(converged, self.start)
        self.assertGreaterEqual(self.clock.now, 0.03)

    def test_timeout_reports_comm_failure_instead_of_false_success(self) -> None:
        target = JointVector((0.0, 2.0, -90.0, 88.0, 0.0))
        target_pose = self.model.forward(target)
        robot = FakeRobot([self.start.as_mapping()] * 20)

        result = self._controller(robot).move(self._goal(target_pose.position_mm))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.ROBOT_COMM_TIMEOUT)
        self.assertEqual(result.executed_waypoints, 0)

    def test_six_axis_observation_is_rejected_without_motion(self) -> None:
        observation = self.start.as_mapping()
        observation["gripper.pos"] = 0.0
        robot = FakeRobot([observation])

        result = self._controller(robot).move(self._goal((100.0, 0.0, -50.0)))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.SAFETY_REJECTED)
        self.assertEqual(robot.sent_actions, [])

    def test_rejected_legacy_calibration_cannot_construct_runtime_controller(
        self,
    ) -> None:
        robot = FakeRobot([self.start.as_mapping()])
        path = Path(
            "workspace/calibration/arm_kinematics_legacy_rejected_20260731.json"
        )

        with self.assertRaisesRegex(RuntimeError, "标定状态不是 accepted"):
            MotionController.from_calibration_file(robot, path, self.config)

        self.assertEqual(robot.sent_actions, [])


if __name__ == "__main__":
    unittest.main()
