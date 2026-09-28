# pyright: reportMissingImports=false

"""USB 串口桥掉线后的有界轻量重连：包装器与控制器集成回归测试。

场景对应 docs/26_USB断连故障研究.md：RP1 集线器 `disabled by hub (EMI?)`
端口禁用 → CH343 约 1s 重枚举 → 旧 fd 永久失效。本测试用可注入时钟模拟
掉线/恢复，验证：非接触相位自动重连恢复、PICK/PLACE 接触段 fail-closed
不重连、重连预算有界、收敛 deadline 宽限、旧连接尽力清理。
"""

import unittest

from control.kinematics import ArmGeometry, JointVector, PlanarArmKinematics
from control.motion_controller import (
    MotionConfig,
    MotionController,
    MotionErrorCode,
    MotionGoal,
)
from control.usb_resilience import ResilientJointRobot


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds

    def timestamp_ns(self) -> int:
        return int(self.now * 1_000_000_000)


JOINT_FIELDS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
)


def _observation(*values: float) -> dict[str, float]:
    return dict(zip(JOINT_FIELDS, values))


class FlakyInnerRobot:
    """模拟 CH343 串口桥：按调用号失败 + 观测序列，重枚举后的新连接健康。"""

    def __init__(
        self,
        observations: list[dict[str, float]],
        *,
        tag: str = "r0",
        fail_on_calls: set[int] | None = None,
        fail_sends_on_calls: set[int] | None = None,
        fail_reads: int = 0,
        fail_sends: int = 0,
    ) -> None:
        if not observations:
            raise ValueError("observations 不能为空")
        self.observations = list(observations)
        self.tag = tag
        self.fail_on_calls = set(fail_on_calls or ())
        self.fail_sends_on_calls = set(fail_sends_on_calls or ())
        self.fail_reads = fail_reads
        self.fail_sends = fail_sends
        self.disconnected = False
        self.reads = 0
        self.sends = 0

    def get_observation(self) -> dict[str, float]:
        self.reads += 1
        if self.fail_reads > 0:
            self.fail_reads -= 1
            raise OSError(f"termios I/O error ({self.tag})")
        if self.reads in self.fail_on_calls:
            raise OSError(f"termios I/O error ({self.tag}) call={self.reads}")
        index = min(len(self.observations) - 1, self.reads - 1)
        return dict(self.observations[index])

    def send_action(
        self, action: dict[str, float], *, goal_velocity: int | None = None
    ) -> dict[str, float]:
        self.sends += 1
        if self.fail_sends > 0:
            self.fail_sends -= 1
            raise RuntimeError(f"usb_submit_urb failed: -19 ({self.tag})")
        if self.sends in self.fail_sends_on_calls:
            raise RuntimeError(
                f"usb_submit_urb failed: -19 ({self.tag}) send={self.sends}"
            )
        return dict(action)

    def disconnect(self) -> None:
        self.disconnected = True


START_OBS = _observation(0.0, 0.0, -90.0, 90.0, 0.0)


class ResilientJointRobotTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = ManualClock()
        self.factory_calls: list[FlakyInnerRobot] = []

    def _make_wrapper(
        self, inner: FlakyInnerRobot, *, max_attempts: int = 2
    ) -> ResilientJointRobot:
        def factory() -> FlakyInnerRobot:
            robot = FlakyInnerRobot(
                observations=[START_OBS],
                tag=f"r{len(self.factory_calls) + 1}",
            )
            self.factory_calls.append(robot)
            return robot

        wrapper = ResilientJointRobot(
            inner,
            factory,
            max_attempts=max_attempts,
            sleep=self.clock.sleep,
            monotonic=self.clock.monotonic,
        )
        return wrapper

    def test_observation_recovers_after_reconnect(self) -> None:
        inner = FlakyInnerRobot(observations=[START_OBS], fail_reads=1)
        wrapper = self._make_wrapper(inner)
        wrapper.set_reconnect_allowed(True)

        observation = wrapper.get_observation()

        self.assertEqual(observation, START_OBS)
        self.assertEqual(len(self.factory_calls), 1)
        self.assertEqual(wrapper.reconnect_epoch(), 1)
        self.assertEqual(self.clock.now, 10.0)
        self.assertTrue(inner.disconnected, "旧连接必须尽力清理")
        self.assertIsNotNone(wrapper.last_reconnect_monotonic())
        self.assertEqual(inner.reads, 1)
        self.assertEqual(self.factory_calls[0].reads, 1)

    def test_reconnect_disallowed_raises_without_factory(self) -> None:
        inner = FlakyInnerRobot(observations=[START_OBS], fail_reads=3)
        wrapper = self._make_wrapper(inner)
        wrapper.set_reconnect_allowed(False)

        with self.assertRaises(OSError):
            wrapper.get_observation()

        self.assertEqual(self.factory_calls, [])
        self.assertEqual(wrapper.reconnect_epoch(), 0)

    def test_reconnect_budget_exhausted_raises(self) -> None:
        # 每次调用最多 2 次重连；设备持续异常（新连接同样失败）时
        # 3 次失败后预算耗尽，直接抛，不再重连。
        inner = FlakyInnerRobot(observations=[START_OBS], fail_reads=999)

        def factory() -> FlakyInnerRobot:
            robot = FlakyInnerRobot(
                observations=[START_OBS],
                fail_reads=999,
                tag=f"r{len(self.factory_calls) + 1}",
            )
            self.factory_calls.append(robot)
            return robot

        wrapper = ResilientJointRobot(
            inner,
            factory,
            max_attempts=2,
            sleep=self.clock.sleep,
            monotonic=self.clock.monotonic,
        )
        wrapper.set_reconnect_allowed(True)

        with self.assertRaises(OSError):
            wrapper.get_observation()

        self.assertEqual(len(self.factory_calls), 2)
        self.assertEqual(wrapper.reconnect_epoch(), 2)

    def test_factory_failure_is_bounded_and_raises_last_error(self) -> None:
        # 设备仍离线（工厂抛错）：有界尝试后抛最后一次错误，不算成功重连。
        inner = FlakyInnerRobot(observations=[START_OBS], fail_reads=999)

        def factory() -> FlakyInnerRobot:
            raise OSError("no such device")

        wrapper = ResilientJointRobot(
            inner,
            factory,
            max_attempts=2,
            sleep=self.clock.sleep,
            monotonic=self.clock.monotonic,
        )
        wrapper.set_reconnect_allowed(True)

        with self.assertRaises(OSError):
            wrapper.get_observation()

        self.assertEqual(wrapper.reconnect_epoch(), 0, "工厂失败不算成功重连")

    def test_send_action_recovers_after_reconnect(self) -> None:
        inner = FlakyInnerRobot(observations=[START_OBS], fail_sends=1)
        wrapper = self._make_wrapper(inner)
        wrapper.set_reconnect_allowed(True)

        action = _observation(1.0, 2.0, 3.0, 4.0, 5.0)
        result = wrapper.send_action(action, goal_velocity=80)

        self.assertEqual(result, action)
        self.assertEqual(wrapper.reconnect_epoch(), 1)
        self.assertEqual(inner.sends, 1)
        self.assertEqual(self.factory_calls[0].sends, 1)

    def test_disconnect_forwards_and_disables_reconnect(self) -> None:
        inner = FlakyInnerRobot(observations=[START_OBS], fail_reads=1)
        wrapper = self._make_wrapper(inner)
        wrapper.set_reconnect_allowed(True)

        wrapper.disconnect()

        self.assertTrue(inner.disconnected)
        wrapper.set_reconnect_allowed(False)  # 关闭后不再重连
        with self.assertRaises(OSError):
            wrapper.get_observation()
        self.assertEqual(self.factory_calls, [])


class MotionControllerReconnectTest(unittest.TestCase):
    """控制器级：掉线期间 move() 能否自动恢复，接触段是否 fail-closed。"""

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
            stable_samples=1,
            poll_interval_s=0.01,
            waypoint_timeout_s=0.05,
            safe_min_z_mm=-50.0,
            contact_min_z_mm=-60.0,
            max_z_mm=250.0,
        )
        self.start = JointVector((0.0, 0.0, -90.0, 90.0, 0.0))
        self.near_target = JointVector((0.0, 2.5, -90.0, 88.0, 0.0))
        self.target = JointVector((0.0, 2.0, -90.0, 88.0, 0.0))

    def _goal(self, phase: str = "PREGRASP") -> MotionGoal:
        position_mm = self.model.forward(self.target).position_mm
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

    def _controller(self, robot: ResilientJointRobot) -> MotionController:
        return MotionController(
            robot=robot,
            kinematics=self.model,
            config=self.config,
            monotonic=self.clock.monotonic,
            sleep=self.clock.sleep,
            timestamp_ns=self.clock.timestamp_ns,
        )

    def _wrapper(self, inner: FlakyInnerRobot) -> ResilientJointRobot:
        def factory() -> FlakyInnerRobot:
            return FlakyInnerRobot(
                observations=[
                    self.near_target.as_mapping(),
                    self.target.as_mapping(),
                ],
                tag=f"recovered-{inner.reads}",
            )

        return ResilientJointRobot(
            inner,
            factory,
            max_attempts=2,
            sleep=self.clock.sleep,
            monotonic=self.clock.monotonic,
        )

    def test_move_recovers_from_usb_dropout_with_deadline_grace(self) -> None:
        # 掉线发生在收敛等待期（第 2 次读）：包装器轻量重连后重试成功，
        # 控制器按重连宽限重新给满收敛窗口，而不是超时失败。
        inner = FlakyInnerRobot(
            observations=[self.start.as_mapping()],
            fail_on_calls={2},
        )
        wrapper = self._wrapper(inner)

        result = self._controller(wrapper).move(self._goal(phase="PREGRASP"))

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.error_code, MotionErrorCode.NONE)
        self.assertEqual(wrapper.reconnect_epoch(), 1)
        self.assertTrue(inner.disconnected, "旧连接必须尽力清理")
        self.assertEqual(result.executed_waypoints, result.planned_waypoints)

    def test_pick_phase_fails_closed_without_reconnect(self) -> None:
        inner = FlakyInnerRobot(
            observations=[self.start.as_mapping()],
            fail_reads=99,
        )
        wrapper = self._wrapper(inner)

        result = self._controller(wrapper).move(self._goal(phase="PICK"))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.ROBOT_COMM_TIMEOUT)
        self.assertEqual(wrapper.reconnect_epoch(), 0, "PICK 接触段禁止自动重连")
        self.assertFalse(inner.disconnected, "未许可时不得动旧连接")

    def test_place_phase_fails_closed_without_reconnect(self) -> None:
        inner = FlakyInnerRobot(
            observations=[self.start.as_mapping()],
            fail_reads=99,
        )
        wrapper = self._wrapper(inner)

        result = self._controller(wrapper).move(self._goal(phase="PLACE"))

        self.assertFalse(result.success)
        self.assertEqual(result.error_code, MotionErrorCode.ROBOT_COMM_TIMEOUT)
        self.assertEqual(wrapper.reconnect_epoch(), 0, "PLACE 接触段禁止自动重连")

    def test_config_can_disable_usb_reconnect_globally(self) -> None:
        config = MotionConfig(
            **{**self.config.as_dict(), "allow_usb_reconnect": False}
        )
        inner = FlakyInnerRobot(
            observations=[self.start.as_mapping()],
            fail_reads=99,
        )
        wrapper = self._wrapper(inner)

        controller = MotionController(
            robot=wrapper,
            kinematics=self.model,
            config=config,
            monotonic=self.clock.monotonic,
            sleep=self.clock.sleep,
            timestamp_ns=self.clock.timestamp_ns,
        )
        result = controller.move(self._goal(phase="PREGRASP"))

        self.assertFalse(result.success)
        self.assertEqual(wrapper.reconnect_epoch(), 0)


if __name__ == "__main__":
    unittest.main()
