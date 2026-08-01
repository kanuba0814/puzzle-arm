"""单臂五轴运动学标定文件加载与质量门禁。"""

from __future__ import annotations

import json
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from numbers import Real
from pathlib import Path

from .kinematics import ArmGeometry, JOINT_FEATURES

_HOLDOUT_XY_RMS_LIMIT_MM = 5.0
_HOLDOUT_XY_MAX_LIMIT_MM = 8.0
_HOLDOUT_Z_MAX_ABS_LIMIT_MM = 3.0
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} 必须是对象")
    if not all(isinstance(key, str) for key in value):
        raise ValueError(f"{name} 的键必须是字符串")
    return value


def _sequence(value: object, name: str) -> Sequence[object]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{name} 必须是数组")
    return value


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} 必须是有限数值，不能是 UNKNOWN")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是有限数值，不能是 UNKNOWN") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数值")
    return number


def _integer(value: object, name: str) -> int:
    number = _number(value, name)
    if not number.is_integer():
        raise ValueError(f"{name} 必须是整数")
    try:
        return int(number)
    except (OverflowError, ValueError) as exc:
        raise ValueError(f"{name} 必须是整数") from exc


def _text(value: object, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} 必须是非空字符串")
    return value


@dataclass(frozen=True)
class CalibrationQuality:
    status: str
    fit_samples: int
    holdout_samples: int
    holdout_xy_rms_mm: float
    holdout_xy_max_mm: float
    holdout_z_max_abs_mm: float


@dataclass(frozen=True)
class ArmCalibration:
    """已解析但不一定获准用于真实运动的单臂标定。"""

    schema_version: int
    calibration_id: str
    robot_type: str
    source_arm: str
    frame: str
    geometry: ArmGeometry
    quality: CalibrationQuality
    evidence_sha256: tuple[str, ...]

    @classmethod
    def load(cls, path: str | Path) -> ArmCalibration:
        calibration_path = Path(path)
        try:
            payload = json.loads(calibration_path.read_text(encoding="utf-8"))
        except OSError as exc:
            raise ValueError(f"无法读取标定文件 {calibration_path}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise ValueError(f"标定文件不是有效 JSON: {exc}") from exc
        return cls.from_mapping(_mapping(payload, "calibration"))

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> ArmCalibration:
        schema_version = _integer(payload.get("schema_version"), "schema_version")
        if schema_version != 1:
            raise ValueError(f"不支持的标定 schema_version={schema_version}")
        calibration_id = _text(payload.get("calibration_id"), "calibration_id")
        robot_type = _text(payload.get("robot_type"), "robot_type")
        if robot_type != "so101_follower_5d":
            raise ValueError("robot_type 必须是 so101_follower_5d")
        source_arm = _text(payload.get("source_arm"), "source_arm")
        if source_arm != "so101_follower_modified_single_arm":
            raise ValueError("source_arm 与实际单臂来源不匹配")
        frame = _text(payload.get("frame"), "frame")
        if frame != "robot_base_mm":
            raise ValueError("标定 frame 必须是 robot_base_mm")

        feature_values = _sequence(payload.get("joint_features"), "joint_features")
        joint_features = tuple(feature_values)
        if joint_features != JOINT_FEATURES:
            raise ValueError(
                "标定五轴字段必须精确匹配固定顺序；"
                f" expected={list(JOINT_FEATURES)}, actual={list(joint_features)}"
            )

        geometry_data = _mapping(payload.get("geometry_mm"), "geometry_mm")
        joint_mapping = _mapping(payload.get("joint_mapping"), "joint_mapping")
        tool_orientation = _mapping(
            payload.get("tool_orientation"), "tool_orientation"
        )
        directions_data = _sequence(
            joint_mapping.get("directions"), "joint_mapping.directions"
        )
        offsets_data = _sequence(
            joint_mapping.get("zero_offsets_deg"),
            "joint_mapping.zero_offsets_deg",
        )
        limits_data = _sequence(
            joint_mapping.get("soft_limits_deg"),
            "joint_mapping.soft_limits_deg",
        )
        if len(directions_data) != 5 or len(offsets_data) != 5 or len(limits_data) != 5:
            raise ValueError("joint_mapping 必须精确包含五轴")
        directions = tuple(
            _integer(value, f"joint_mapping.directions[{index}]")
            for index, value in enumerate(directions_data)
        )
        offsets = tuple(
            _number(value, f"joint_mapping.zero_offsets_deg[{index}]")
            for index, value in enumerate(offsets_data)
        )
        limits: list[tuple[float, float]] = []
        for index, value in enumerate(limits_data):
            pair = _sequence(value, f"joint_mapping.soft_limits_deg[{index}]")
            if len(pair) != 2:
                raise ValueError(
                    f"joint_mapping.soft_limits_deg[{index}] 必须包含上下限"
                )
            limits.append(
                (
                    _number(pair[0], f"soft_limits_deg[{index}].low"),
                    _number(pair[1], f"soft_limits_deg[{index}].high"),
                )
            )

        geometry = ArmGeometry(
            calibration_id=calibration_id,
            shoulder_height_mm=_number(
                geometry_data.get("shoulder_height"),
                "geometry_mm.shoulder_height",
            ),
            shoulder_radial_offset_mm=_number(
                geometry_data.get("shoulder_radial_offset"),
                "geometry_mm.shoulder_radial_offset",
            ),
            shoulder_lateral_offset_mm=_number(
                geometry_data.get("shoulder_lateral_offset"),
                "geometry_mm.shoulder_lateral_offset",
            ),
            upper_arm_mm=_number(
                geometry_data.get("upper_arm"), "geometry_mm.upper_arm"
            ),
            forearm_mm=_number(geometry_data.get("forearm"), "geometry_mm.forearm"),
            tool_forward_mm=_number(
                geometry_data.get("tool_forward"), "geometry_mm.tool_forward"
            ),
            tool_normal_mm=_number(
                geometry_data.get("tool_normal"), "geometry_mm.tool_normal"
            ),
            joint_directions=(
                directions[0],
                directions[1],
                directions[2],
                directions[3],
                directions[4],
            ),
            joint_zero_offsets_deg=(
                offsets[0],
                offsets[1],
                offsets[2],
                offsets[3],
                offsets[4],
            ),
            joint_limits_deg=(limits[0], limits[1], limits[2], limits[3], limits[4]),
            wrist_roll_yaw_sign=_integer(
                tool_orientation.get("wrist_roll_yaw_sign"),
                "tool_orientation.wrist_roll_yaw_sign",
            ),
            tool_yaw_zero_deg=_number(
                tool_orientation.get("yaw_zero_deg"),
                "tool_orientation.yaw_zero_deg",
            ),
        )

        quality_data = _mapping(payload.get("quality"), "quality")
        quality = CalibrationQuality(
            status=_text(quality_data.get("status"), "quality.status"),
            fit_samples=_integer(
                quality_data.get("fit_samples"), "quality.fit_samples"
            ),
            holdout_samples=_integer(
                quality_data.get("holdout_samples"), "quality.holdout_samples"
            ),
            holdout_xy_rms_mm=_number(
                quality_data.get("holdout_xy_rms_mm"),
                "quality.holdout_xy_rms_mm",
            ),
            holdout_xy_max_mm=_number(
                quality_data.get("holdout_xy_max_mm"),
                "quality.holdout_xy_max_mm",
            ),
            holdout_z_max_abs_mm=_number(
                quality_data.get("holdout_z_max_abs_mm"),
                "quality.holdout_z_max_abs_mm",
            ),
        )
        evidence_data = _sequence(payload.get("evidence_sha256"), "evidence_sha256")
        evidence = tuple(
            _text(value, f"evidence_sha256[{index}]")
            for index, value in enumerate(evidence_data)
        )
        if not evidence or any(
            _SHA256_PATTERN.fullmatch(digest) is None for digest in evidence
        ):
            raise ValueError("evidence_sha256 必须包含有效的小写 SHA256")
        return cls(
            schema_version=schema_version,
            calibration_id=calibration_id,
            robot_type=robot_type,
            source_arm=source_arm,
            frame=frame,
            geometry=geometry,
            quality=quality,
            evidence_sha256=evidence,
        )

    def require_accepted_geometry(self) -> ArmGeometry:
        quality = self.quality
        if quality.status != "accepted":
            raise RuntimeError(f"标定状态不是 accepted: {quality.status}; 禁止真实运动")
        if quality.fit_samples < 3 or quality.holdout_samples < 4:
            raise RuntimeError("标定样本不足：至少 3 个拟合点和 4 个独立验收点")
        if quality.holdout_xy_rms_mm > _HOLDOUT_XY_RMS_LIMIT_MM:
            raise RuntimeError(
                "标定 XY RMS 超限："
                f"{quality.holdout_xy_rms_mm:.3f} > "
                f"{_HOLDOUT_XY_RMS_LIMIT_MM:.3f} mm"
            )
        if quality.holdout_xy_max_mm > _HOLDOUT_XY_MAX_LIMIT_MM:
            raise RuntimeError(
                "标定 XY 最大误差超限："
                f"{quality.holdout_xy_max_mm:.3f} > "
                f"{_HOLDOUT_XY_MAX_LIMIT_MM:.3f} mm"
            )
        if quality.holdout_z_max_abs_mm > _HOLDOUT_Z_MAX_ABS_LIMIT_MM:
            raise RuntimeError(
                "标定 Z 最大绝对误差超限："
                f"{quality.holdout_z_max_abs_mm:.3f} > "
                f"{_HOLDOUT_Z_MAX_ABS_LIMIT_MM:.3f} mm"
            )
        return self.geometry
