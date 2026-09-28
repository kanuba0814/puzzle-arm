# pyright: reportMissingImports=false

"""verify_usb_reconnect.py 编排逻辑测试（不碰硬件，mock 适配器/内核事件）。

覆盖：掉线检测→重连断言→PASS/skipped/FAIL 三态；contact-fail-closed 模式
PICK 禁止重连 + 清理路径 attempt_park 恢复。
"""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / "src"
        / "robot_plugin"
        / "lerobot_robot_so101_5d"
    ),
)

from control.motion_controller import MotionErrorCode  # noqa: E402
from tools import verify_usb_reconnect as tool  # noqa: E402

START = (0.0, 0.0, 120.0)


class FakeMotion:
    def __init__(
        self,
        results: list[object] | None = None,
        on_move: object | None = None,
    ) -> None:
        self.results = list(results or [])
        self.on_move = on_move

    def move(self, goal: object) -> object:
        # 模拟 wrapper 门禁：只有非接触段（PICK/PLACE 之外）掉线才重连。
        if self.on_move is not None and getattr(goal, "phase", None) not in (
            "PICK",
            "PLACE",
        ):
            self.on_move()
        return self.results.pop(0) if self.results else _ok_result()


def _ok_result() -> SimpleNamespace:
    return SimpleNamespace(
        success=True,
        error_code=MotionErrorCode.NONE,
        executed_waypoints=1,
        planned_waypoints=1,
    )


class FakeRobot:
    def __init__(self) -> None:
        self.epoch = 0
        self.allow_calls: list[bool] = []

    def reconnect_epoch(self) -> int:
        return self.epoch

    def set_reconnect_allowed(self, allowed: bool) -> None:
        self.allow_calls.append(allowed)


class FakeAdapter:
    def __init__(self, motion_results: list[object] | None = None) -> None:
        self.robot = FakeRobot()
        self.motion = FakeMotion(
            motion_results,
            on_move=lambda: setattr(self.robot, "epoch", self.robot.epoch + 1),
        )
        self.park_mover = SimpleNamespace(park_to_home=lambda: None)
        self._park_result: tuple[bool, str] = (True, "PASS")
        self.disconnect_called = False

    def home_pose(self) -> SimpleNamespace:
        return SimpleNamespace(
            position_mm=START, pitch_deg=-90.0, yaw_deg=0.0
        )

    def attempt_park(self) -> tuple[bool, str]:
        # 清理路径：重连成功后再完成 PARK（epoch +1）
        self.robot.epoch += 1
        return self._park_result


class RecoveryModeTest(unittest.TestCase):
    def test_dropout_recovered_marks_pass(self) -> None:
        adapter = FakeAdapter([_ok_result(), _ok_result()])
        with mock.patch.object(tool, "kernel_usb_events", side_effect=[(5, 0), (6, 1)]):
            with mock.patch("builtins.input", return_value=""):
                results = tool.run_recovery(adapter, rounds=1)
        self.assertEqual(results[0]["status"], "PASS")
        self.assertTrue(results[0]["dropout_detected"])
        # 两次运动各触发一次重连
        self.assertEqual(results[0]["reconnect_epoch_delta"], 2)
        self.assertEqual(results[0]["move_a"]["success"], True)
        self.assertEqual(results[0]["emi_events_since_boot"], 1)

    def test_no_kernel_dropout_marks_skipped(self) -> None:
        adapter = FakeAdapter([_ok_result(), _ok_result()])
        with mock.patch.object(tool, "kernel_usb_events", return_value=(5, 0)):
            with mock.patch("builtins.input", return_value=""):
                results = tool.run_recovery(adapter, rounds=1)
        self.assertEqual(results[0]["status"], "skipped(no kernel dropout event)")
        self.assertFalse(results[0]["dropout_detected"])

    def test_dropout_without_recovery_marks_fail(self) -> None:
        adapter = FakeAdapter(
            [
                SimpleNamespace(
                    success=False,
                    error_code=MotionErrorCode.ROBOT_COMM_TIMEOUT,
                    executed_waypoints=0,
                    planned_waypoints=1,
                ),
                _ok_result(),
            ]
        )
        with mock.patch.object(tool, "kernel_usb_events", side_effect=[(5, 0), (6, 1)]):
            with mock.patch("builtins.input", return_value=""):
                results = tool.run_recovery(adapter, rounds=1)
        self.assertEqual(results[0]["status"], "FAIL")


class ContactFailClosedTest(unittest.TestCase):
    def test_pick_phase_denies_reconnect_then_cleanup_recovers(self) -> None:
        adapter = FakeAdapter()
        adapter._park_result = (True, "已在正式 PARK")
        motion_result = SimpleNamespace(
            success=False,
            error_code=MotionErrorCode.ROBOT_COMM_TIMEOUT,
            executed_waypoints=0,
            planned_waypoints=1,
        )
        adapter.motion.results = [motion_result]

        with mock.patch("builtins.input", return_value=""):
            result = tool.run_contact_fail_closed(adapter)

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["error_code"], MotionErrorCode.ROBOT_COMM_TIMEOUT)
        # PICK 阶段不重连（0），attempt_park 阶段重连一次（1）
        self.assertEqual(result["reconnect_epoch_delta"], 1)

    def test_pick_phase_motion_sent_is_fail(self) -> None:
        # 若 PICK 相位真的运动了（executed>0），必须判定 FAIL。
        adapter = FakeAdapter()
        adapter.motion.results = [
            SimpleNamespace(
                success=True,
                error_code=MotionErrorCode.NONE,
                executed_waypoints=1,
                planned_waypoints=1,
            )
        ]
        with mock.patch("builtins.input", return_value=""):
            result = tool.run_contact_fail_closed(adapter)
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("未按预期 fail-closed", result["reason"])


if __name__ == "__main__":
    unittest.main()
