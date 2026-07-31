# pyright: reportMissingImports=false

"""Deterministic table-mm sweep ordering and whole-segment IK preflight."""

from __future__ import annotations

import itertools
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Protocol

from control.kinematics import JointVector, ToolPose
from control.motion_controller import MotionConfig


@dataclass(frozen=True)
class PieceRef:
    piece_id: str
    center_table_mm: tuple[float, float]
    pick_point_table_mm: tuple[float, float] | None = None

    def __post_init__(self) -> None:
        if not self.piece_id or len(self.center_table_mm) != 2:
            raise ValueError("PieceRef 必须包含 id 和二维 table_mm 中心")


def _distance(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def plan_sweep_order(
    pieces: Sequence[PieceRef], home_table_mm: tuple[float, float]
) -> tuple[PieceRef, ...]:
    """Return shortest HOME→pieces→HOME route, deterministically for n≤4."""
    if len({piece.piece_id for piece in pieces}) != len(pieces):
        raise ValueError("piece_id 必须在一次 sweep 内唯一")
    if len(pieces) <= 1:
        return tuple(pieces)
    if len(pieces) <= 4:
        candidates = itertools.permutations(pieces)
        return min(
            candidates,
            key=lambda route: (
                sum(
                    _distance(previous, current.center_table_mm)
                    for previous, current in zip(
                        (home_table_mm,)
                        + tuple(piece.center_table_mm for piece in route[:-1]),
                        route,
                        strict=True,
                    )
                )
                + _distance(route[-1].center_table_mm, home_table_mm),
                tuple(piece.piece_id for piece in route),
            ),
        )
    remaining = sorted(pieces, key=lambda piece: piece.piece_id)
    route: list[PieceRef] = []
    current = home_table_mm
    while remaining:
        next_piece = min(
            remaining,
            key=lambda piece: (
                _distance(current, piece.center_table_mm),
                piece.piece_id,
            ),
        )
        route.append(next_piece)
        remaining.remove(next_piece)
        current = next_piece.center_table_mm
    return tuple(route)


class SweepKinematics(Protocol):
    def inverse(self, pose: ToolPose) -> JointVector: ...


def preflight_sweep_ik(kinematics: SweepKinematics, poses: Iterable[ToolPose]) -> None:
    """Compatibility check for a sequence of discrete poses."""
    for pose in poses:
        kinematics.inverse(pose)


def preflight_continuous_sweep(
    kinematics: SweepKinematics,
    motion_config: MotionConfig,
    poses: Sequence[ToolPose],
) -> None:
    """Validate every interpolated Cartesian point and its joint-space step.

    This is deliberately independent from ``MotionController`` internals so the
    entire route is proven before PARK/HOME or a servo command is attempted.
    """
    if len(poses) < 2:
        raise ValueError("连续 sweep 至少需要起点和终点")
    previous_joint: JointVector | None = None
    for segment_index, (start, target) in enumerate(
        zip(poses, poses[1:], strict=False)
    ):
        dx = target.position_mm[0] - start.position_mm[0]
        dy = target.position_mm[1] - start.position_mm[1]
        dz = target.position_mm[2] - start.position_mm[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        pitch_delta = _shortest_angle_delta(start.pitch_deg, target.pitch_deg)
        yaw_delta = _shortest_angle_delta(start.yaw_deg, target.yaw_deg)
        steps = max(
            1,
            math.ceil(distance / motion_config.max_cartesian_step_mm),
            math.ceil(abs(pitch_delta) / motion_config.max_orientation_step_deg),
            math.ceil(abs(yaw_delta) / motion_config.max_orientation_step_deg),
        )
        if previous_joint is None:
            previous_joint = kinematics.inverse(start)
        for point_index in range(1, steps + 1):
            fraction = point_index / steps
            pose = ToolPose(
                "robot_base_mm",
                (
                    start.position_mm[0] + dx * fraction,
                    start.position_mm[1] + dy * fraction,
                    start.position_mm[2] + dz * fraction,
                ),
                start.pitch_deg + pitch_delta * fraction,
                start.yaw_deg + yaw_delta * fraction,
            )
            try:
                current_joint = kinematics.inverse(pose)
            except ValueError as error:
                raise ValueError(
                    f"连续段 {segment_index} 路径点 {point_index}/{steps} IK 不可达: {error}"
                ) from error
            joint_step = current_joint.max_abs_error(previous_joint)
            if joint_step > motion_config.max_joint_step_deg:
                raise ValueError(
                    f"连续段 {segment_index} 路径点 {point_index}/{steps} 关节步长 "
                    f"{joint_step:.3f}° 超过 {motion_config.max_joint_step_deg:.3f}°"
                )
            previous_joint = current_joint


def _shortest_angle_delta(start_deg: float, target_deg: float) -> float:
    return (target_deg - start_deg + 180.0) % 360.0 - 180.0
