# pyright: reportMissingImports=false

"""Checked joint-space PARK↔HOME transition used only by integration."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from enum import Enum

from control.kinematics import JOINT_FEATURES, JointVector
from control.motion_controller import JointRobot


class ParkMoveErrorCode(str, Enum):
    PARK_MISMATCH = "PARK_MISMATCH"
    PARK_TRANSITION_FAILED = "PARK_TRANSITION_FAILED"


class ParkMoveError(RuntimeError):
    def __init__(self, code: ParkMoveErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class ParkMoveResult:
    success: bool
    error_code: ParkMoveErrorCode | None
    message: str


class JointSpaceParkMover:
    """Move through upright pose while requiring actual five-axis readback."""

    def __init__(
        self,
        robot: JointRobot,
        *,
        park_deg: Mapping[str, float],
        home_deg: Mapping[str, float],
        park_tolerance_deg: float,
        follow_tolerance_deg: float,
        step_deg: float = 1.0,
        settle_s: float = 0.0,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._robot = robot
        self._park = self._as_joint_vector(park_deg)
        self._home = self._as_joint_vector(home_deg)
        if min(park_tolerance_deg, follow_tolerance_deg, step_deg) <= 0:
            raise ValueError("PARK 容差和步长必须大于零")
        self._park_tolerance_deg = park_tolerance_deg
        self._follow_tolerance_deg = follow_tolerance_deg
        self._step_deg = step_deg
        self._settle_s = settle_s
        self._monotonic = monotonic
        self._sleep = sleep

    @staticmethod
    def _as_joint_vector(values: Mapping[str, float]) -> JointVector:
        normalized = {
            feature: values[feature.removesuffix(".pos")]
            if feature.removesuffix(".pos") in values
            else values[feature]
            for feature in JOINT_FEATURES
        }
        return JointVector.from_mapping(normalized)

    def is_at_park(self) -> bool:
        return self._read().max_abs_error(self._park) <= self._park_tolerance_deg

    def park_to_home(self) -> ParkMoveResult:
        if not self.is_at_park():
            return ParkMoveResult(
                False, ParkMoveErrorCode.PARK_MISMATCH, "机械臂不在正式 PARK"
            )
        return self._move_via_upright(self._home)

    def home_to_park(self) -> ParkMoveResult:
        result = self._move_via_upright(self._park)
        if not result.success:
            return result
        try:
            final_error_deg = self._read().max_abs_error(self._park)
        except ParkMoveError as error:
            return ParkMoveResult(False, error.code, str(error))
        if final_error_deg > self._park_tolerance_deg:
            return ParkMoveResult(
                False,
                ParkMoveErrorCode.PARK_TRANSITION_FAILED,
                "终点未进入正式 PARK 容差；"
                f"最大关节误差 {final_error_deg:.3f}° > "
                f"{self._park_tolerance_deg:.3f}°",
            )
        return result

    def _move_via_upright(self, destination: JointVector) -> ParkMoveResult:
        upright_values = list(self._home.values_deg)
        upright_values[0] = self._park.values_deg[0]
        upright = JointVector(
            (
                upright_values[0],
                upright_values[1],
                upright_values[2],
                upright_values[3],
                upright_values[4],
            )
        )
        try:
            self._interpolate(self._read(), upright)
            self._interpolate(
                upright,
                destination,
                final_tolerance_deg=(
                    self._park_tolerance_deg if destination == self._park else None
                ),
            )
        except ParkMoveError as error:
            return ParkMoveResult(False, error.code, str(error))
        return ParkMoveResult(True, None, "PARK/HOME 过渡完成")

    def _read(self) -> JointVector:
        try:
            return JointVector.from_mapping(self._robot.get_observation())
        except (TypeError, ValueError) as error:
            raise ParkMoveError(
                ParkMoveErrorCode.PARK_TRANSITION_FAILED, "五轴读回无效"
            ) from error

    def _interpolate(
        self,
        start: JointVector,
        target: JointVector,
        *,
        final_tolerance_deg: float | None = None,
    ) -> None:
        largest_delta = max(
            abs(target_value - start_value)
            for start_value, target_value in zip(
                start.values_deg, target.values_deg, strict=True
            )
        )
        steps = max(1, math.ceil(largest_delta / self._step_deg))
        per_step_timeout_s = 5.0
        for index in range(1, steps + 1):
            ratio = index / steps
            values = tuple(
                start_value + (target_value - start_value) * ratio
                for start_value, target_value in zip(
                    start.values_deg, target.values_deg, strict=True
                )
            )
            command = JointVector(
                (values[0], values[1], values[2], values[3], values[4])
            )
            self._robot.send_action(command.as_mapping())
            # 轮询读回直到接近命令再走下一步（2026-08-01 实机修复：
            # 原来 settle 60ms 未真正生效，误差尚小时会以总线最快速度连续
            # 下发路点。现在先保证最小保持时间，再等待跟随容差，避免机械臂
            # 受重力滞后时出现目标突变和高频通信）。
            sent_at = self._monotonic()
            deadline = sent_at + per_step_timeout_s
            waypoint_tolerance_deg = self._follow_tolerance_deg
            if index == steps and final_tolerance_deg is not None:
                # 回正式 PARK 的最后一个路点不能使用巡航跟随容差；否则
                # 8° 宽容差会让 7° 误差被当成完成，随后正式 PARK 验收才失败。
                waypoint_tolerance_deg = final_tolerance_deg
            while True:
                actual = self._read()
                elapsed = self._monotonic() - sent_at
                error_deg = actual.max_abs_error(command)
                if elapsed >= self._settle_s and error_deg <= waypoint_tolerance_deg:
                    break
                if self._monotonic() >= deadline:
                    raise ParkMoveError(
                        ParkMoveErrorCode.PARK_TRANSITION_FAILED,
                        f"PARK/HOME 单步跟随超时，误差 {error_deg:.2f}° "
                        f"> {waypoint_tolerance_deg:.2f}°",
                    )
                # settle 期间按固定采样周期等待，不能紧密 read-loop；否则
                # “最小保持时间”反而会制造更多串口请求。
                remaining_settle = max(0.0, self._settle_s - elapsed)
                self._sleep(min(0.05, remaining_settle) if remaining_settle else 0.05)
