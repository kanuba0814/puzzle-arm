"""USB 串口桥掉线后的有界轻量自动重连包装器。

背景（docs/26_USB断连故障研究.md）：机械臂运动/磁铁动作期间，RP1 集线器可能因
信号完整性事件禁用端口（`usb usb3-port2: disabled by hub (EMI?), re-enabling...`），
CH343 约 1 秒后自动重枚举。`cdc_acm` 断开即销毁 tty，旧文件描述符永久失效，
必须等待最多 10 秒让设备重新上线，再重新打开设备并做轻量连接（`reconnect_lite`：不断力矩、
不重配电机，仅对齐 Goal=Present 并校验扭矩全开）后重试当前调用。

安全语义：
- `set_reconnect_allowed(False)`（PICK/PLACE 接触段）时**不重连**，异常原样上抛，
  由上层 fail-closed；不允许把自动重连混入接触动作。
- 每次调用重连预算有界（max_attempts，默认 2）；预算耗尽抛最后一次异常。
- 重连工厂必须是"轻量连接"的机器人构造器；旧连接尽力清理（可能已死，吞错）。
- 重连成功后 `reconnect_epoch()` +1，调用方（MotionController）据此给收敛
  deadline 追加宽限——重连期间舵机仍在按最后 Goal_Position 运动。
"""

from __future__ import annotations

import math
import termios
import time
from collections.abc import Callable, Mapping
from typing import Any


_COMMUNICATION_ERRORS = (OSError, RuntimeError, ValueError, termios.error)


class ResilientJointRobot:
    """实现 JointRobot 协议；通信异常时按门禁执行有界轻量重连并重试。"""

    def __init__(
        self,
        robot: Any,
        robot_factory: Callable[[], Any],
        *,
        max_attempts: int = 2,
        reconnect_wait_s: float = 10.0,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_attempts < 1:
            raise ValueError("max_attempts 必须 >= 1")
        if not math.isfinite(reconnect_wait_s) or reconnect_wait_s < 0.0:
            raise ValueError("reconnect_wait_s 必须是非负有限数")
        self._robot = robot
        self._robot_factory = robot_factory
        self._max_attempts = max_attempts
        self._reconnect_wait_s = reconnect_wait_s
        self._sleep = sleep
        self._monotonic = monotonic
        self._reconnect_allowed = False
        self._epoch = 0
        self._last_reconnect_monotonic: float | None = None

    # ------------------------------------------------------------------
    # 门禁接口（由 MotionController 按相位/高度设置，或清理路径开启）
    # ------------------------------------------------------------------
    def set_reconnect_allowed(self, allowed: bool) -> None:
        self._reconnect_allowed = bool(allowed)

    def reconnect_epoch(self) -> int:
        """成功重连次数；每次 +1，供调用方做 deadline 宽限。"""
        return self._epoch

    def last_reconnect_monotonic(self) -> float | None:
        return self._last_reconnect_monotonic

    # ------------------------------------------------------------------
    # JointRobot 协议
    # ------------------------------------------------------------------
    def get_observation(self) -> Mapping[str, object]:
        return self._call_with_reconnect(lambda robot: robot.get_observation())

    def send_action(
        self,
        action: dict[str, float],
        *,
        goal_velocity: int | None = None,
        goal_velocity_by_joint: Mapping[str, int] | None = None,
    ) -> Mapping[str, object]:
        def send(robot: Any) -> Mapping[str, object]:
            if goal_velocity_by_joint:
                return robot.send_action(
                    action,
                    goal_velocity=goal_velocity,
                    goal_velocity_by_joint=goal_velocity_by_joint,
                )
            return robot.send_action(action, goal_velocity=goal_velocity)

        return self._call_with_reconnect(send)

    def disconnect(self) -> None:
        self._reconnect_allowed = False
        self._robot.disconnect()

    # ------------------------------------------------------------------
    # 内部
    # ------------------------------------------------------------------
    def _call_with_reconnect(self, op: Callable[[Any], Any]) -> Any:
        last_error: BaseException | None = None
        attempts_left = self._max_attempts
        while True:
            try:
                return op(self._robot)
            except _COMMUNICATION_ERRORS as exc:
                last_error = exc
                if not self._reconnect_allowed or attempts_left <= 0:
                    raise
                attempts_left -= 1
                # CH343/ttyACM 断开后通常约 1 秒重枚举；保守等待 10 秒，
                # 给 USB 短时断线完整的重新上线窗口，再重新打开串口。
                self._sleep(self._reconnect_wait_s)
                try:
                    self._reconnect()
                except BaseException as reconnect_error:
                    # 设备可能仍在重枚举/离线：记录后进入下一轮尝试（有界）。
                    last_error = reconnect_error

    def _reconnect(self) -> None:
        old = self._robot
        try:
            old.disconnect()
        except BaseException:
            # 旧连接可能已死（fd 失效），尽力清理即可。
            pass
        self._robot = self._robot_factory()
        self._epoch += 1
        self._last_reconnect_monotonic = self._monotonic()
