"""配置驱动的单臂五轴平面运动学。

外部关节顺序固定为 SO-101 follower 的 J1..J5。运动学只处理
``robot_base_mm``，不包含相机或桌面坐标变换，也不控制电磁铁。
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from numbers import Real

JOINT_FEATURES = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
)


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是有限数值") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数值")
    return number


def _wrap_degrees(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0
    return wrapped


def _angle_in_limit(angle_deg: float, low: float, high: float) -> float:
    candidates = [angle_deg + 360.0 * turn for turn in range(-2, 3)]
    valid = [candidate for candidate in candidates if low <= candidate <= high]
    if not valid:
        raise ValueError(f"关节角 {angle_deg:.3f}° 超出软限位 [{low:.3f}, {high:.3f}]°")
    return min(valid, key=lambda candidate: abs(candidate - angle_deg))


@dataclass(frozen=True)
class JointVector:
    """按固定五轴顺序保存的角度向量，单位为 degree。"""

    values_deg: tuple[float, float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.values_deg) != len(JOINT_FEATURES):
            raise ValueError("单臂关节向量必须恰好包含五轴")
        normalized = tuple(
            _finite_float(value, feature)
            for feature, value in zip(JOINT_FEATURES, self.values_deg, strict=True)
        )
        object.__setattr__(
            self,
            "values_deg",
            (
                normalized[0],
                normalized[1],
                normalized[2],
                normalized[3],
                normalized[4],
            ),
        )

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, object]) -> JointVector:
        actual = set(mapping)
        expected = set(JOINT_FEATURES)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            raise ValueError(
                f"五轴字段必须精确匹配固定契约； missing={missing}, extra={extra}"
            )
        values = tuple(
            _finite_float(mapping[feature], feature) for feature in JOINT_FEATURES
        )
        return cls((values[0], values[1], values[2], values[3], values[4]))

    def as_mapping(self) -> dict[str, float]:
        return dict(zip(JOINT_FEATURES, self.values_deg, strict=True))

    def error_map(self, other: JointVector) -> dict[str, float]:
        """逐关节有界角误差；五个位置舵机均不得跨软限位做环绕。"""
        return {
            feature: abs(left - right)
            for feature, (left, right) in zip(
                JOINT_FEATURES,
                zip(self.values_deg, other.values_deg),
                strict=True,
            )
        }

    def max_abs_error(self, other: JointVector) -> float:
        """五个有界位置关节的最大绝对误差（degree）。"""
        return max(
            abs(left - right)
            for left, right in zip(self.values_deg, other.values_deg, strict=True)
        )


@dataclass(frozen=True)
class ToolPose:
    """机器人基座系 TCP 位姿；位置 mm，姿态 degree。"""

    frame: str
    position_mm: tuple[float, float, float]
    pitch_deg: float
    yaw_deg: float

    def __post_init__(self) -> None:
        if self.frame != "robot_base_mm":
            raise ValueError("ToolPose.frame 必须是 robot_base_mm")
        if len(self.position_mm) != 3:
            raise ValueError("position_mm 必须恰好包含三个坐标")
        position = tuple(
            _finite_float(value, f"position_mm[{index}]")
            for index, value in enumerate(self.position_mm)
        )
        object.__setattr__(self, "position_mm", position)
        object.__setattr__(
            self, "pitch_deg", _finite_float(self.pitch_deg, "pitch_deg")
        )
        object.__setattr__(self, "yaw_deg", _finite_float(self.yaw_deg, "yaw_deg"))


@dataclass(frozen=True)
class ArmGeometry:
    """由版本化标定文件构造的五轴几何与关节映射。"""

    calibration_id: str
    shoulder_height_mm: float
    shoulder_radial_offset_mm: float
    shoulder_lateral_offset_mm: float
    upper_arm_mm: float
    forearm_mm: float
    tool_forward_mm: float
    tool_normal_mm: float
    joint_directions: tuple[int, int, int, int, int]
    joint_zero_offsets_deg: tuple[float, float, float, float, float]
    joint_limits_deg: tuple[
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
        tuple[float, float],
    ]
    wrist_roll_yaw_sign: int
    tool_yaw_zero_deg: float

    def __post_init__(self) -> None:
        if not self.calibration_id:
            raise ValueError("calibration_id 不能为空")
        for name in (
            "shoulder_height_mm",
            "shoulder_radial_offset_mm",
            "shoulder_lateral_offset_mm",
            "upper_arm_mm",
            "forearm_mm",
            "tool_forward_mm",
            "tool_normal_mm",
        ):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))
        if self.upper_arm_mm <= 0.0 or self.forearm_mm <= 0.0:
            raise ValueError("upper_arm_mm 和 forearm_mm 必须为正数")
        if self.tool_forward_mm < 0.0:
            raise ValueError("tool_forward_mm 不能为负数")
        if len(self.joint_directions) != 5 or any(
            direction not in (-1, 1) for direction in self.joint_directions
        ):
            raise ValueError("joint_directions 必须包含五个 +1/-1")
        if self.wrist_roll_yaw_sign not in (-1, 1):
            raise ValueError("wrist_roll_yaw_sign 必须是 +1 或 -1")
        object.__setattr__(
            self,
            "tool_yaw_zero_deg",
            _finite_float(self.tool_yaw_zero_deg, "tool_yaw_zero_deg"),
        )
        if len(self.joint_zero_offsets_deg) != 5:
            raise ValueError("joint_zero_offsets_deg 必须包含五轴")
        zero_offsets = tuple(
            _finite_float(value, f"joint_zero_offsets_deg[{index}]")
            for index, value in enumerate(self.joint_zero_offsets_deg)
        )
        object.__setattr__(self, "joint_zero_offsets_deg", zero_offsets)
        if len(self.joint_limits_deg) != 5:
            raise ValueError("joint_limits_deg 必须包含五轴")
        limits: list[tuple[float, float]] = []
        for index, pair in enumerate(self.joint_limits_deg):
            if len(pair) != 2:
                raise ValueError(f"joint_limits_deg[{index}] 必须是 [low, high]")
            low = _finite_float(pair[0], f"joint_limits_deg[{index}].low")
            high = _finite_float(pair[1], f"joint_limits_deg[{index}].high")
            if low >= high:
                raise ValueError(f"joint_limits_deg[{index}] 下限必须小于上限")
            limits.append((low, high))
        object.__setattr__(self, "joint_limits_deg", tuple(limits))

    def as_dict(self) -> dict[str, object]:
        return {
            "calibration_id": self.calibration_id,
            "shoulder_height_mm": self.shoulder_height_mm,
            "shoulder_radial_offset_mm": self.shoulder_radial_offset_mm,
            "shoulder_lateral_offset_mm": self.shoulder_lateral_offset_mm,
            "upper_arm_mm": self.upper_arm_mm,
            "forearm_mm": self.forearm_mm,
            "tool_forward_mm": self.tool_forward_mm,
            "tool_normal_mm": self.tool_normal_mm,
            "joint_directions": self.joint_directions,
            "joint_zero_offsets_deg": self.joint_zero_offsets_deg,
            "joint_limits_deg": self.joint_limits_deg,
            "wrist_roll_yaw_sign": self.wrist_roll_yaw_sign,
            "tool_yaw_zero_deg": self.tool_yaw_zero_deg,
        }

    def validate_joints(self, joints: JointVector) -> None:
        for feature, angle, (low, high) in zip(
            JOINT_FEATURES,
            joints.values_deg,
            self.joint_limits_deg,
            strict=True,
        ):
            if not low <= angle <= high:
                raise ValueError(
                    f"{feature}={angle:.3f}° 超出软限位 [{low:.3f}, {high:.3f}]°"
                )


class PlanarArmKinematics:
    """单偏航轴 + 三俯仰轴 + 腕滚转的解析 FK/IK。"""

    def __init__(self, geometry: ArmGeometry) -> None:
        self.geometry = geometry

    def _physical_angles_rad(self, joints: JointVector) -> tuple[float, ...]:
        self.geometry.validate_joints(joints)
        return tuple(
            math.radians(direction * logical + offset)
            for logical, direction, offset in zip(
                joints.values_deg,
                self.geometry.joint_directions,
                self.geometry.joint_zero_offsets_deg,
                strict=True,
            )
        )

    def forward(self, joints: JointVector) -> ToolPose:
        pan, shoulder, elbow, wrist, roll = self._physical_angles_rad(joints)
        forearm_pitch = shoulder + elbow
        tool_pitch = forearm_pitch + wrist

        radial = (
            self.geometry.shoulder_radial_offset_mm
            + self.geometry.upper_arm_mm * math.cos(shoulder)
            + self.geometry.forearm_mm * math.cos(forearm_pitch)
            + self.geometry.tool_forward_mm * math.cos(tool_pitch)
            - self.geometry.tool_normal_mm * math.sin(tool_pitch)
        )
        height = (
            self.geometry.shoulder_height_mm
            + self.geometry.upper_arm_mm * math.sin(shoulder)
            + self.geometry.forearm_mm * math.sin(forearm_pitch)
            + self.geometry.tool_forward_mm * math.sin(tool_pitch)
            + self.geometry.tool_normal_mm * math.cos(tool_pitch)
        )
        lateral = self.geometry.shoulder_lateral_offset_mm
        x = math.cos(pan) * radial - math.sin(pan) * lateral
        y = math.sin(pan) * radial + math.cos(pan) * lateral
        return ToolPose(
            frame="robot_base_mm",
            position_mm=(x, y, height),
            pitch_deg=_wrap_degrees(math.degrees(tool_pitch)),
            yaw_deg=_wrap_degrees(
                self.geometry.tool_yaw_zero_deg
                + math.degrees(pan)
                + self.geometry.wrist_roll_yaw_sign * math.degrees(roll)
            ),
        )

    def inverse(self, pose: ToolPose) -> JointVector:
        if pose.frame != "robot_base_mm":
            raise ValueError("IK 只接受 robot_base_mm")
        x, y, z = pose.position_mm
        lateral = self.geometry.shoulder_lateral_offset_mm
        radius_squared = x * x + y * y
        if radius_squared + 1e-9 < lateral * lateral:
            raise ValueError("目标不可达：水平半径小于肩部侧向偏置")

        radial = math.sqrt(max(0.0, radius_squared - lateral * lateral))
        if radial <= 1e-9 and abs(lateral) <= 1e-9:
            raise ValueError("目标不可达：TCP 位于偏航轴奇点")
        pan = math.atan2(y, x) - math.atan2(lateral, radial)
        tool_pitch = math.radians(pose.pitch_deg)
        wrist_radius = (
            radial
            - self.geometry.shoulder_radial_offset_mm
            - self.geometry.tool_forward_mm * math.cos(tool_pitch)
            + self.geometry.tool_normal_mm * math.sin(tool_pitch)
        )
        wrist_height = (
            z
            - self.geometry.shoulder_height_mm
            - self.geometry.tool_forward_mm * math.sin(tool_pitch)
            - self.geometry.tool_normal_mm * math.cos(tool_pitch)
        )
        distance_squared = wrist_radius**2 + wrist_height**2
        upper = self.geometry.upper_arm_mm
        forearm = self.geometry.forearm_mm
        cosine_elbow = (distance_squared - upper**2 - forearm**2) / (
            2.0 * upper * forearm
        )
        if cosine_elbow < -1.0 - 1e-9 or cosine_elbow > 1.0 + 1e-9:
            distance = math.sqrt(distance_squared)
            raise ValueError(
                "目标不可达："
                f"肩腕距离 {distance:.3f} mm 不在 "
                f"[{abs(upper - forearm):.3f}, {upper + forearm:.3f}] mm"
            )
        cosine_elbow = max(-1.0, min(1.0, cosine_elbow))
        elbow = -math.acos(cosine_elbow)
        shoulder = math.atan2(wrist_height, wrist_radius) - math.atan2(
            forearm * math.sin(elbow),
            upper + forearm * math.cos(elbow),
        )
        wrist = tool_pitch - shoulder - elbow
        roll = (
            math.radians(pose.yaw_deg - self.geometry.tool_yaw_zero_deg) - pan
        ) / self.geometry.wrist_roll_yaw_sign
        physical_deg = tuple(
            math.degrees(angle) for angle in (pan, shoulder, elbow, wrist, roll)
        )

        logical: list[float] = []
        for physical, direction, offset, (low, high) in zip(
            physical_deg,
            self.geometry.joint_directions,
            self.geometry.joint_zero_offsets_deg,
            self.geometry.joint_limits_deg,
            strict=True,
        ):
            candidate = (physical - offset) / direction
            logical.append(_angle_in_limit(candidate, low, high))
        result = JointVector(
            (logical[0], logical[1], logical[2], logical[3], logical[4])
        )
        self.geometry.validate_joints(result)
        return result
