# pyright: reportMissingImports=false

"""Thin real-hardware adapters for the frozen five-axis inspection baseline.

This module intentionally contains integration wiring only.  Cartesian motion,
IK, PARK interpolation, and the SO101 bumpless connection are delegated to the
already-frozen public components.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control.calibration import ArmCalibration
from control.kinematics import (
    JOINT_FEATURES,
    JointVector,
    PlanarArmKinematics,
    ToolPose,
)
from control.motion_controller import MotionConfig, MotionController
from control.motion_detail_log import MotionDetailLogger

from .park_sequence import JointSpaceParkMover
from .sweep_planner import PieceRef

HOME_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -22.202025,
    "elbow_flex": 38.197802,
    "wrist_flex": 84.659341,
    "wrist_roll": -82.505495,
}
BUMPLESS_P_COEFFICIENT = 24
BUMPLESS_ACCELERATION = 30
SAFE_HEIGHT_MM = 60.0
PREGRASP_HEIGHT_MM = 20.0
RADIAL_AWAY_COMPENSATION_MM = 3.0
PRECISION_PITCH_DEG = -87.5


@dataclass(frozen=True)
class TableToRobotTransform:
    yaw_deg: float
    translation_mm: tuple[float, float, float]
    sheet_origin_table_mm: tuple[float, float]

    @classmethod
    def load(cls, path: Path) -> TableToRobotTransform:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            translation = payload["translation_mm"]
            origin = payload["inverse_sheet_from_robot"]["translation_mm"]
            yaw_deg = float(payload["yaw_deg"])
            if len(translation) != 3 or len(origin) < 2:
                raise ValueError("变换字段长度无效")
            values = tuple(float(value) for value in translation)
            if not all(math.isfinite(value) for value in values + (yaw_deg,)):
                raise ValueError("变换必须为有限数值")
            return cls(
                yaw_deg,
                (values[0], values[1], values[2]),
                (float(origin[0]), float(origin[1])),
            )
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            raise RuntimeError(
                f"无法加载正式 table/contact-sheet 变换 {path}: {error}"
            ) from error

    def table_to_robot(
        self, point_table_mm: tuple[float, float], height_mm: float
    ) -> tuple[float, float, float]:
        table_x, table_y = point_table_mm
        origin_x, origin_y = self.sheet_origin_table_mm
        delta_x = table_x - origin_x
        delta_y = table_y - origin_y
        radius = math.hypot(delta_x, delta_y)
        if radius <= 1e-9:
            raise ValueError("目标位于 table 原点，无法施加径向 3 mm 终点补偿")
        table_x += RADIAL_AWAY_COMPENSATION_MM * delta_x / radius
        table_y += RADIAL_AWAY_COMPENSATION_MM * delta_y / radius
        yaw = math.radians(self.yaw_deg)
        translation_x, translation_y, translation_z = self.translation_mm
        return (
            math.cos(yaw) * table_x - math.sin(yaw) * table_y + translation_x,
            math.sin(yaw) * table_x + math.cos(yaw) * table_y + translation_y,
            translation_z + height_mm,
        )

    def robot_to_table(
        self, point_robot_mm: tuple[float, float, float]
    ) -> tuple[float, float]:
        robot_x, robot_y, _ = point_robot_mm
        translation_x, translation_y, _ = self.translation_mm
        x = robot_x - translation_x
        y = robot_y - translation_y
        yaw = math.radians(self.yaw_deg)
        return (
            math.cos(yaw) * x + math.sin(yaw) * y,
            -math.sin(yaw) * x + math.cos(yaw) * y,
        )


class InspectionHardwareAdapter:
    """Owns a connected robot and exposes only public frozen control seams."""

    def __init__(
        self,
        *,
        calibration_path: Path,
        transform_path: Path,
        park_pose_path: Path,
        port: Path,
        lerobot_calibration_dir: Path,
        motion_config: MotionConfig,
        motion_detail_log_dir: Path | None = None,
    ) -> None:
        calibration = ArmCalibration.load(calibration_path)
        self.kinematics = PlanarArmKinematics(calibration.require_accepted_geometry())
        self.transform = TableToRobotTransform.load(transform_path)
        self._park_deg = self._load_park(park_pose_path)
        self._motion_config = motion_config
        self._port = port
        self._lerobot_calibration_dir = lerobot_calibration_dir
        self.robot: Any | None = None
        self.motion: MotionController | None = None
        self.park_mover: JointSpaceParkMover | None = None
        self.motion_detail_log: MotionDetailLogger | None = (
            MotionDetailLogger(motion_detail_log_dir)
            if motion_detail_log_dir is not None
            else None
        )

    @staticmethod
    def _load_park(path: Path) -> dict[str, float]:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            joint_deg = payload["joint_deg"]
            values = {name: float(joint_deg[name]) for name in HOME_DEG}
            JointVector.from_mapping(
                {f"{name}.pos": value for name, value in values.items()}
            )
            return values
        except (
            OSError,
            KeyError,
            TypeError,
            ValueError,
            json.JSONDecodeError,
        ) as error:
            raise RuntimeError(f"无法加载正式 PARK 标定 {path}: {error}") from error

    def connect(self) -> None:
        if self.robot is not None:
            raise RuntimeError("机械臂已经连接")
        try:
            from lerobot_robot_so101_5d import SO101Follower5D, SO101Follower5DConfig

            robot = SO101Follower5D(
                SO101Follower5DConfig(
                    port=str(self._port),
                    id="epuzzle-single-arm",
                    calibration_dir=self._lerobot_calibration_dir,
                    disable_torque_on_disconnect=True,
                    use_degrees=True,
                )
            )
            robot.connect_bumpless(
                p_coefficient=BUMPLESS_P_COEFFICIENT,
                acceleration=BUMPLESS_ACCELERATION,
            )
            self.robot = robot
            self.motion = MotionController(
                robot,
                self.kinematics,
                self._motion_config,
                detail_log=self.motion_detail_log,
            )
            self.park_mover = JointSpaceParkMover(
                robot,
                park_deg=self._park_deg,
                home_deg=HOME_DEG,
                park_tolerance_deg=4.0,
                follow_tolerance_deg=8.0,
                step_deg=1.0,
                settle_s=0.06,
            )
        except BaseException:
            self.disconnect()
            raise

    def pose_for(self, piece: PieceRef, height_mm: float) -> ToolPose:
        robot_position = self.transform.table_to_robot(piece.center_table_mm, height_mm)
        lateral = self.kinematics.geometry.shoulder_lateral_offset_mm
        radius_squared = robot_position[0] ** 2 + robot_position[1] ** 2 - lateral**2
        if radius_squared <= 0.0:
            raise ValueError("目标位于偏航不可达区域")
        radial = math.sqrt(radius_squared)
        physical_pan = math.degrees(
            math.atan2(robot_position[1], robot_position[0])
            - math.atan2(lateral, radial)
        )
        return ToolPose(
            "robot_base_mm", robot_position, PRECISION_PITCH_DEG, physical_pan
        )

    def home_pose(self) -> ToolPose:
        values_deg = tuple(
            HOME_DEG[feature.removesuffix(".pos")] for feature in JOINT_FEATURES
        )
        return self.kinematics.forward(
            JointVector(
                (
                    values_deg[0],
                    values_deg[1],
                    values_deg[2],
                    values_deg[3],
                    values_deg[4],
                )
            )
        )

    def home_projection_table_mm(self) -> tuple[float, float]:
        return self.transform.robot_to_table(self.home_pose().position_mm)

    def is_at_park(self) -> bool:
        return self.park_mover is not None and self.park_mover.is_at_park()

    def attempt_park(self) -> tuple[bool, str]:
        if self.park_mover is None:
            return False, "机械臂未连接，无法回 PARK"
        if self.park_mover.is_at_park():
            return True, "已在正式 PARK"
        result = self.park_mover.home_to_park()
        return result.success, result.message

    def disconnect(self) -> None:
        if self.robot is not None:
            try:
                self.robot.disconnect()
            finally:
                self.robot = None
                self.motion = None
                self.park_mover = None
        if self.motion_detail_log is not None:
            self.motion_detail_log.close()
