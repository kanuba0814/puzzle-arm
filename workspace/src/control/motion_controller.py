# pyright: reportMissingImports=false

"""MotionGoal 到 MotionResult 的单臂确定性控制深模块。"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from numbers import Real
from pathlib import Path
from typing import Protocol

from .calibration import ArmCalibration
from .kinematics import JointVector, PlanarArmKinematics, ToolPose
from .motion_detail_log import MotionDetailLogger

_ALLOWED_PHASES = {
    "HOME",
    "SAFE",
    "PREGRASP",
    "PICK",
    "PREPLACE",
    "PLACE",
    "RECOVERY",
}
_ALLOWED_SPEED_PROFILES = {
    "EMPTY_NORMAL",
    "EMPTY_SLOW",
    "HOLDING_SLOW",
    "LOCAL_POLICY",
}


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是有限数值") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数值")
    return number


def _shortest_angle_delta(start_deg: float, target_deg: float) -> float:
    return (target_deg - start_deg + 180.0) % 360.0 - 180.0


class MotionErrorCode(str, Enum):
    NONE = "NONE"
    IK_NO_SOLUTION = "IK_NO_SOLUTION"
    ROBOT_COMM_TIMEOUT = "ROBOT_COMM_TIMEOUT"
    SAFETY_REJECTED = "SAFETY_REJECTED"
    GLOBAL_TIMEOUT = "GLOBAL_TIMEOUT"


class _PlanningFailure(ValueError):
    def __init__(self, error_code: MotionErrorCode, message: str) -> None:
        super().__init__(message)
        self.error_code = error_code


@dataclass(frozen=True)
class MotionGoal:
    goal_id: str
    timestamp_ns: int
    frame: str
    position_mm: tuple[float, float, float]
    pitch_deg: float
    yaw_deg: float
    phase: str
    speed_profile: str
    deadline_ns: int | None = None

    def __post_init__(self) -> None:
        if not self.goal_id:
            raise ValueError("goal_id 不能为空")
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or self.timestamp_ns < 0
        ):
            raise ValueError("timestamp_ns 必须是非负整数")
        if self.frame != "robot_base_mm":
            raise ValueError("MotionGoal.frame 必须是 robot_base_mm")
        if len(self.position_mm) != 3:
            raise ValueError("position_mm 必须包含三个坐标")
        position = tuple(
            _finite(value, f"position_mm[{index}]")
            for index, value in enumerate(self.position_mm)
        )
        object.__setattr__(self, "position_mm", position)
        object.__setattr__(self, "pitch_deg", _finite(self.pitch_deg, "pitch_deg"))
        object.__setattr__(self, "yaw_deg", _finite(self.yaw_deg, "yaw_deg"))
        if self.phase not in _ALLOWED_PHASES:
            raise ValueError(f"未知 phase={self.phase}")
        if self.speed_profile not in _ALLOWED_SPEED_PROFILES:
            raise ValueError(f"未知 speed_profile={self.speed_profile}")
        if self.deadline_ns is not None and (
            isinstance(self.deadline_ns, bool)
            or not isinstance(self.deadline_ns, int)
            or self.deadline_ns < 0
        ):
            raise ValueError("deadline_ns 必须是非负整数")

    def as_pose(self) -> ToolPose:
        return ToolPose(
            frame=self.frame,
            position_mm=self.position_mm,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
        )


@dataclass(frozen=True)
class MotionConfig:
    max_cartesian_step_mm: float
    max_orientation_step_deg: float
    max_joint_step_deg: float
    joint_tolerance_deg: float
    stable_samples: int
    poll_interval_s: float
    waypoint_timeout_s: float
    safe_min_z_mm: float
    contact_min_z_mm: float
    max_z_mm: float
    # 非接触转移的笛卡尔步长：若非 None，phase 不属于 PICK/PLACE 时用它，
    # 减少路点数，让舵机做更长的连续插值。接触目标仍用 max_cartesian_step_mm。
    transit_cartesian_step_mm: float | None = None
    # 中间路点跟随容差：若非 None，中间点只需达到该宽松容差即继续，
    # 舵机自身加速度曲线保持连续，消除逐点停顿。最终点永远用严格 joint_tolerance_deg。
    follow_tolerance_deg: float | None = None
    # 梯形速度规划：每个 speed_profile 的 Goal_Velocity（STS3215 寄存器 46，
    # 单位 0.0293 RPM/LSB 约 0.176°/s·LSB；非 None 时下发，None 保持出厂默认）。
    # 与插件 connect_bumpless 已配置的 Acceleration 配合，由舵机固件生成
    # 加速-匀速-减速的梯形曲线，避免逐点高速冲击。
    goal_velocity_by_profile: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in (
            "max_cartesian_step_mm",
            "max_orientation_step_deg",
            "max_joint_step_deg",
            "joint_tolerance_deg",
            "poll_interval_s",
            "waypoint_timeout_s",
        ):
            value = _finite(getattr(self, name), name)
            if value <= 0.0:
                raise ValueError(f"{name} 必须大于零")
            object.__setattr__(self, name, value)
        for name in ("safe_min_z_mm", "contact_min_z_mm", "max_z_mm"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))
        if not self.contact_min_z_mm <= self.safe_min_z_mm < self.max_z_mm:
            raise ValueError(
                "高度限制必须满足 contact_min_z_mm <= safe_min_z_mm < max_z_mm"
            )
        if (
            isinstance(self.stable_samples, bool)
            or not isinstance(self.stable_samples, int)
            or self.stable_samples < 1
        ):
            raise ValueError("stable_samples 必须是正整数")
        if self.follow_tolerance_deg is not None:
            follow = _finite(self.follow_tolerance_deg, "follow_tolerance_deg")
            if follow <= 0.0:
                raise ValueError("follow_tolerance_deg 必须大于零")
            object.__setattr__(self, "follow_tolerance_deg", follow)
        if self.transit_cartesian_step_mm is not None:
            transit = _finite(
                self.transit_cartesian_step_mm, "transit_cartesian_step_mm"
            )
            if transit <= 0.0:
                raise ValueError("transit_cartesian_step_mm 必须大于零")
            object.__setattr__(self, "transit_cartesian_step_mm", transit)

    def as_dict(self) -> dict[str, object]:
        return {
            "max_cartesian_step_mm": self.max_cartesian_step_mm,
            "max_orientation_step_deg": self.max_orientation_step_deg,
            "max_joint_step_deg": self.max_joint_step_deg,
            "joint_tolerance_deg": self.joint_tolerance_deg,
            "stable_samples": self.stable_samples,
            "poll_interval_s": self.poll_interval_s,
            "waypoint_timeout_s": self.waypoint_timeout_s,
            "safe_min_z_mm": self.safe_min_z_mm,
            "contact_min_z_mm": self.contact_min_z_mm,
            "max_z_mm": self.max_z_mm,
            "transit_cartesian_step_mm": self.transit_cartesian_step_mm,
            "follow_tolerance_deg": self.follow_tolerance_deg,
        }


@dataclass(frozen=True)
class MotionResult:
    goal_id: str
    timestamp_ns: int
    success: bool
    error_code: MotionErrorCode
    message: str
    calibration_id: str
    planned_waypoints: int
    executed_waypoints: int
    final_joint_deg: dict[str, float] | None

    def as_mapping(self) -> dict[str, object]:
        return {
            "goal_id": self.goal_id,
            "timestamp_ns": self.timestamp_ns,
            "success": self.success,
            "error_code": self.error_code.value,
            "message": self.message,
            "calibration_id": self.calibration_id,
            "planned_waypoints": self.planned_waypoints,
            "executed_waypoints": self.executed_waypoints,
            "final_joint_deg": self.final_joint_deg,
        }


@dataclass(frozen=True)
class MotionPlan:
    waypoints: tuple[JointVector, ...]


class JointRobot(Protocol):
    def get_observation(self) -> Mapping[str, object]: ...

    def send_action(
        self, action: dict[str, float], *, goal_velocity: int | None = None
    ) -> Mapping[str, object]: ...


class MotionController:
    """先完整预检，再逐点发送并用实际读回确认收敛。"""

    def __init__(
        self,
        robot: JointRobot,
        kinematics: PlanarArmKinematics,
        config: MotionConfig,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        timestamp_ns: Callable[[], int] = time.monotonic_ns,
        detail_log: MotionDetailLogger | None = None,
    ) -> None:
        self._robot = robot
        self._kinematics = kinematics
        self._config = config
        self._monotonic = monotonic
        self._sleep = sleep
        self._timestamp_ns = timestamp_ns
        self._detail_log = detail_log
        self._goal_id_for_log: str = ""
        self._waypoint_index_for_log: int = -1

    @classmethod
    def from_calibration_file(
        cls,
        robot: JointRobot,
        calibration_path: str | Path,
        config: MotionConfig,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        timestamp_ns: Callable[[], int] = time.monotonic_ns,
    ) -> MotionController:
        """只允许通过质量门禁的标定进入真实控制构造路径。"""
        calibration = ArmCalibration.load(calibration_path)
        geometry = calibration.require_accepted_geometry()
        return cls(
            robot=robot,
            kinematics=PlanarArmKinematics(geometry),
            config=config,
            monotonic=monotonic,
            sleep=sleep,
            timestamp_ns=timestamp_ns,
        )

    def move(self, goal: MotionGoal) -> MotionResult:
        if self._detail_log is not None:
            self._detail_log.goal(
                goal.goal_id, goal.as_pose(), goal.phase, goal.speed_profile
            )
        if self._deadline_expired(goal):
            return self._result(
                goal,
                success=False,
                error_code=MotionErrorCode.GLOBAL_TIMEOUT,
                message="目标在规划前已超过 deadline",
            )
        try:
            current = self._read_joints()
        except ValueError as exc:
            return self._result(
                goal,
                success=False,
                error_code=MotionErrorCode.SAFETY_REJECTED,
                message=f"五轴观测不符合契约: {exc}",
            )
        except Exception as exc:  # hardware adapter errors are surfaced as results
            return self._result(
                goal,
                success=False,
                error_code=MotionErrorCode.ROBOT_COMM_TIMEOUT,
                message=f"读取机械臂状态失败: {exc}",
            )

        try:
            plan = self._plan(goal, current)
        except _PlanningFailure as exc:
            return self._result(
                goal,
                success=False,
                error_code=exc.error_code,
                message=str(exc),
            )
        if self._detail_log is not None:
            target_jv = self._kinematics.inverse(goal.as_pose())
            self._detail_log.plan(
                goal.goal_id,
                steps=len(plan.waypoints),
                joint_span_deg=target_jv.max_abs_error(current),
                start_joints=current.as_mapping(),
                target_joints=target_jv.as_mapping(),
            )

        executed = 0
        final = current
        last_index = len(plan.waypoints) - 1
        for index, waypoint in enumerate(plan.waypoints):
            if self._deadline_expired(goal):
                return self._result(
                    goal,
                    success=False,
                    error_code=MotionErrorCode.GLOBAL_TIMEOUT,
                    message="执行过程中超过 deadline",
                    planned=len(plan.waypoints),
                    executed=executed,
                    final=final,
                )
            try:
                # 梯形速度：按 speed_profile 查表下发 Goal_Velocity，
                # 舵机固件（Acceleration + Goal_Velocity）生成平滑曲线。
                goal_velocity = self._config.goal_velocity_by_profile.get(
                    goal.speed_profile
                )
                # USB 瞬时掉线重试：3s 间隔最多重试 2 次；Goal_Position 写
                # 幂等，重发同一目标安全。
                sent: Mapping[str, object] | None = None
                last_send_error: Exception | None = None
                for _attempt in range(3):
                    try:
                        sent = self._robot.send_action(
                            waypoint.as_mapping(), goal_velocity=goal_velocity
                        )
                        break
                    except (ValueError, RuntimeError, OSError) as send_exc:
                        last_send_error = send_exc
                        self._sleep(3.0)
                if sent is None:
                    raise RuntimeError(f"舵机目标发送失败: {last_send_error}")
                sent = JointVector.from_mapping(sent)
            except (ValueError, RuntimeError, OSError) as exc:
                return self._result(
                    goal,
                    success=False,
                    error_code=MotionErrorCode.SAFETY_REJECTED,
                    message=f"舵机目标发送失败: {exc}",
                    planned=len(plan.waypoints),
                    executed=executed,
                    final=final,
                )
            if sent.max_abs_error(waypoint) > 1e-6:
                return self._result(
                    goal,
                    success=False,
                    error_code=MotionErrorCode.SAFETY_REJECTED,
                    message="JointRobot 修改了已预检目标，拒绝继续执行",
                    planned=len(plan.waypoints),
                    executed=executed,
                    final=final,
                )
            if self._detail_log is not None:
                self._detail_log.waypoint_sent(
                    goal.goal_id, index, len(plan.waypoints), waypoint.as_mapping()
                )
            # 终点永远用严格容差；中间点可用宽松跟随容差，避免逐点停顿。
            # PICK/PLACE 接触目标不得使用跟随容差：负载下会提前判到位。
            use_follow = (
                self._config.follow_tolerance_deg is not None
                and index != last_index
                and goal.phase not in {"PICK", "PLACE"}
            )
            tolerance = (
                self._config.follow_tolerance_deg
                if use_follow and self._config.follow_tolerance_deg is not None
                else self._config.joint_tolerance_deg
            )
            # 详细日志上下文（_wait_for_target 内记录 readback 用）
            self._goal_id_for_log = goal.goal_id
            self._waypoint_index_for_log = index
            converged = self._wait_for_target(
                waypoint,
                tolerance_deg=tolerance,
                stable_samples=1 if use_follow else self._config.stable_samples,
            )
            if converged is None:
                if self._detail_log is not None:
                    self._detail_log.waypoint_timeout(
                        goal.goal_id, index, waypoint.max_abs_error(final)
                    )
                return self._result(
                    goal,
                    success=False,
                    error_code=MotionErrorCode.ROBOT_COMM_TIMEOUT,
                    message="舵机实际位置未在时限内稳定收敛",
                    planned=len(plan.waypoints),
                    executed=executed,
                    final=final,
                )
            if self._detail_log is not None:
                self._detail_log.waypoint_converged(
                    goal.goal_id,
                    index,
                    reads=1,
                    final_error=converged.max_abs_error(waypoint),
                )
            final = converged
            executed += 1

        return self._result(
            goal,
            success=True,
            error_code=MotionErrorCode.NONE,
            message="目标已执行并由实际五轴读回确认",
            planned=len(plan.waypoints),
            executed=executed,
            final=final,
        )

    def _plan(self, goal: MotionGoal, current: JointVector) -> MotionPlan:
        try:
            current_pose = self._kinematics.forward(current)
        except ValueError as exc:
            raise _PlanningFailure(
                MotionErrorCode.SAFETY_REJECTED,
                f"当前五轴状态不能进入普通笛卡尔规划: {exc}",
            ) from exc
        current_z = current_pose.position_mm[2]
        if not self._config.contact_min_z_mm <= current_z <= self._config.max_z_mm:
            raise _PlanningFailure(
                MotionErrorCode.SAFETY_REJECTED,
                f"当前 TCP 高度 {current_z:.3f} mm 超出全局安全范围 ",
            )
        target_pose = goal.as_pose()
        target_z = target_pose.position_mm[2]
        minimum_target_z = (
            self._config.contact_min_z_mm
            if goal.phase in {"PICK", "PLACE"}
            else self._config.safe_min_z_mm
        )
        if not minimum_target_z <= target_z <= self._config.max_z_mm:
            raise _PlanningFailure(
                MotionErrorCode.SAFETY_REJECTED,
                f"phase={goal.phase} 的目标 TCP 高度 {target_z:.3f} mm "
                f"不在 [{minimum_target_z:.3f}, {self._config.max_z_mm:.3f}] mm",
            )
        dx = target_pose.position_mm[0] - current_pose.position_mm[0]
        dy = target_pose.position_mm[1] - current_pose.position_mm[1]
        dz = target_pose.position_mm[2] - current_pose.position_mm[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        pitch_delta = _shortest_angle_delta(
            current_pose.pitch_deg, target_pose.pitch_deg
        )
        yaw_delta = _shortest_angle_delta(current_pose.yaw_deg, target_pose.yaw_deg)
        # 接触目标保持细分段；非接触转移可用更大步长减少加/减速次数。
        cartesian_step_mm = self._config.max_cartesian_step_mm
        if self._config.transit_cartesian_step_mm is not None and goal.phase not in {
            "PICK",
            "PLACE",
        }:
            cartesian_step_mm = self._config.transit_cartesian_step_mm
        # 笛卡尔距离小但关节大幅重构时（例如 HOME→首航点），仅按距离和姿态
        # 分段会使单步关节跳变超限。先解一次终点 IK 估算关节位移，把它计入步数。
        try:
            target_joints = self._kinematics.inverse(target_pose)
        except ValueError as exc:
            raise _PlanningFailure(
                MotionErrorCode.IK_NO_SOLUTION,
                f"目标位姿 IK 不可达: {exc}",
            ) from exc
        joint_span_deg = target_joints.max_abs_error(current)
        steps = max(
            1,
            math.ceil(distance / cartesian_step_mm),
            math.ceil(abs(pitch_delta) / self._config.max_orientation_step_deg),
            math.ceil(abs(yaw_delta) / self._config.max_orientation_step_deg),
            math.ceil(joint_span_deg / self._config.max_joint_step_deg),
        )

        # 终点关节位移只是下界：IK 非线性使中间段单步跳变可能大于均值。
        # 不放宽 max_joint_step_deg，而是超限就翻倍加密重试，最多 4 轮。
        last_failure: _PlanningFailure | None = None
        for _ in range(4):
            try:
                return MotionPlan(
                    tuple(
                        self._interpolate(
                            current,
                            current_pose,
                            steps,
                            (dx, dy, dz),
                            pitch_delta,
                            yaw_delta,
                        )
                    )
                )
            except _PlanningFailure as failure:
                if failure.error_code is not MotionErrorCode.SAFETY_REJECTED:
                    raise
                last_failure = failure
                steps *= 2
        assert last_failure is not None
        raise last_failure

    def _interpolate(
        self,
        current: JointVector,
        current_pose: ToolPose,
        steps: int,
        deltas: tuple[float, float, float],
        pitch_delta: float,
        yaw_delta: float,
    ) -> list[JointVector]:
        """等分插值并逐点做 IK、关节步长预检；任何一点超限即抛。"""
        dx, dy, dz = deltas
        waypoints: list[JointVector] = []
        previous = current
        for index in range(1, steps + 1):
            fraction = index / steps
            pose = ToolPose(
                frame="robot_base_mm",
                position_mm=(
                    current_pose.position_mm[0] + dx * fraction,
                    current_pose.position_mm[1] + dy * fraction,
                    current_pose.position_mm[2] + dz * fraction,
                ),
                pitch_deg=current_pose.pitch_deg + pitch_delta * fraction,
                yaw_deg=current_pose.yaw_deg + yaw_delta * fraction,
            )
            try:
                waypoint = self._kinematics.inverse(pose)
            except ValueError as exc:
                raise _PlanningFailure(
                    MotionErrorCode.IK_NO_SOLUTION,
                    f"路径点 {index}/{steps} IK 不可达: {exc}",
                ) from exc
            joint_step = waypoint.max_abs_error(previous)
            if joint_step > self._config.max_joint_step_deg:
                raise _PlanningFailure(
                    MotionErrorCode.SAFETY_REJECTED,
                    f"路径点 {index}/{steps} 关节步长 "
                    f"{joint_step:.3f}° 超过 "
                    f"{self._config.max_joint_step_deg:.3f}°",
                )
            waypoints.append(waypoint)
            previous = waypoint
        return waypoints

    def _read_joints(self) -> JointVector:
        """读取五轴状态；USB 瞬时掉线时 3s 间隔重试 2 次。

        现场经验：连续长时间运动后机械臂 USB 偶发掉线（termios I/O error），
        多数情况下重试即恢复；重试 2 次仍失败才抛出，由调用方转成
        ROBOT_COMM_TIMEOUT 结果。
        """
        last_error: Exception | None = None
        for attempt in range(3):  # 首次 + 2 次重试
            try:
                observation = self._robot.get_observation()
                joint_fields = {
                    key: value
                    for key, value in observation.items()
                    if key.endswith(".pos")
                }
                return JointVector.from_mapping(joint_fields)
            except (ValueError, RuntimeError, OSError) as exc:
                last_error = exc
                self._sleep(3.0)
        assert last_error is not None
        raise last_error

    def _wait_for_target(
        self,
        target: JointVector,
        *,
        tolerance_deg: float | None = None,
        stable_samples: int | None = None,
    ) -> JointVector | None:
        tolerance = (
            self._config.joint_tolerance_deg if tolerance_deg is None else tolerance_deg
        )
        required_stable = (
            self._config.stable_samples if stable_samples is None else stable_samples
        )
        deadline = self._monotonic() + self._config.waypoint_timeout_s
        stable = 0
        latest: JointVector | None = None
        reads = 0
        while self._monotonic() <= deadline:
            try:
                latest = self._read_joints()
            except (ValueError, RuntimeError, OSError):
                stable = 0
            else:
                reads += 1
                if self._detail_log is not None:
                    error = latest.error_map(target)
                    self._detail_log.readback(
                        self._goal_id_for_log,
                        index=self._waypoint_index_for_log,
                        actual=latest.as_mapping(),
                        error_deg=error,
                        max_error=max(error.values(), default=0.0),
                        tolerance_deg=tolerance,
                    )
                if latest.max_abs_error(target) <= tolerance:
                    stable += 1
                    if stable >= required_stable:
                        return latest
                else:
                    stable = 0
            self._sleep(self._config.poll_interval_s)
        return None

    def _deadline_expired(self, goal: MotionGoal) -> bool:
        return goal.deadline_ns is not None and self._timestamp_ns() > goal.deadline_ns

    def _result(
        self,
        goal: MotionGoal,
        success: bool,
        error_code: MotionErrorCode,
        message: str,
        planned: int = 0,
        executed: int = 0,
        final: JointVector | None = None,
    ) -> MotionResult:
        if self._detail_log is not None:
            self._detail_log.result(
                goal.goal_id,
                success,
                error_code.value,
                executed,
                planned,
                duration_s=0.0,
                final_joints={} if final is None else final.as_mapping(),
            )
        return MotionResult(
            goal_id=goal.goal_id,
            timestamp_ns=self._timestamp_ns(),
            success=success,
            error_code=error_code,
            message=message,
            calibration_id=self._kinematics.geometry.calibration_id,
            planned_waypoints=planned,
            executed_waypoints=executed,
            final_joint_deg=None if final is None else final.as_mapping(),
        )
