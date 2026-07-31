"""圆角物理轮廓到虚拟直角恢复（本项目独立实现）。

算法思想（公开几何思想，非外部代码移植）：真实卡片圆角在轮廓简化后表现为
一条短弦，其两侧是足够长且近似垂直的直边。把两侧直边外推求交，即得用于
矩形拼接的"虚拟直角"。物理轮廓保持不变，只派生虚拟几何。

安全约束：
- 官方最短切割边为 20 mm，因此短弦上限必须远小于该值，避免吞掉真实切割顶点；
- 仅在显式材料 profile 中启用；白色金属片 profile 默认关闭；
- 每次替换都记录弦深、拟合角和残差，便于审计。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RoundedCornerCandidate:
    """单个圆角替换的可审计记录。"""

    arc_start_index: int
    arc_span: int
    virtual_vertex_mm: tuple[float, float]
    chord_length_mm: float
    chord_depth_mm: float
    fit_angle_deg: float
    residual_mm: float


@dataclass(frozen=True)
class RoundedCornerResult:
    polygon_mm: tuple[tuple[float, float], ...]
    candidates: tuple[RoundedCornerCandidate, ...]

    @property
    def replaced_count(self) -> int:
        return len(self.candidates)


def _scalar(value: object) -> float:
    """把 numpy 标量转为有限 Python float；非有限值显式失败。"""
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"圆角几何出现非数值: {value!r}") from error
    if not math.isfinite(result):
        raise ValueError(f"圆角几何出现非有限值: {result!r}")
    return result


def _line_intersection(
    first_a: np.ndarray,
    first_b: np.ndarray,
    second_a: np.ndarray,
    second_b: np.ndarray,
) -> np.ndarray | None:
    """两条无限直线交点；近平行返回 None。"""
    first_direction = first_b - first_a
    second_direction = second_b - second_a
    cross = (
        first_direction[0] * second_direction[1]
        - first_direction[1] * second_direction[0]
    )
    if abs(cross) < 1e-9:
        return None
    delta = second_a - first_a
    step = (delta[0] * second_direction[1] - delta[1] * second_direction[0]) / cross
    return first_a + step * first_direction


def _point_line_distance(
    point: np.ndarray, line_a: np.ndarray, line_b: np.ndarray
) -> float:
    """点到无限直线的垂距。"""
    direction = line_b - line_a
    length = np.linalg.norm(direction)
    if length < 1e-9:
        return _scalar(np.linalg.norm(point - line_a))
    offset = point - line_a
    return abs(_scalar(direction[0] * offset[1] - direction[1] * offset[0])) / _scalar(
        length
    )


def recover_virtual_corners(
    polygon_mm: np.ndarray,
    *,
    max_chord_mm: float = 8.0,
    min_straight_ratio: float = 1.5,
    min_corner_angle_deg: float = 60.0,
    max_corner_angle_deg: float = 120.0,
    max_residual_mm: float = 2.0,
    max_arc_span: int = 3,
    max_sides: int = 5,
) -> RoundedCornerResult:
    """把圆弧短弦替换为虚拟直角顶点。

    Args:
        polygon_mm: 毫米域有序多边形顶点，shape (N, 2)。
        max_chord_mm: 单个圆弧总弦长上限；必须远小于官方 20 mm 最短边。
        min_straight_ratio: 两侧直边长度至少为圆弧总弦长的该倍数。
        min_corner_angle_deg: 两侧直边夹角下限。
        max_corner_angle_deg: 两侧直边夹角上限。
        max_residual_mm: 虚拟顶点到原直边的最大允许偏离。
        max_arc_span: 单个圆弧允许被简化成的最大连续短边数量。
        max_sides: 官方每片边数上限。

    Returns:
        `RoundedCornerResult`；无可信圆角时返回原多边形和空候选。
    """
    points = [
        np.asarray((_scalar(point[0]), _scalar(point[1])), dtype=float)
        for point in np.asarray(polygon_mm, dtype=float)
    ]
    if len(points) < 4 or max_chord_mm <= 0.0:
        return RoundedCornerResult(
            tuple((_scalar(p[0]), _scalar(p[1])) for p in points), ()
        )

    candidates: list[RoundedCornerCandidate] = []
    changed = True
    while changed and len(points) >= 4:
        changed = False
        count = len(points)
        for start in range(count):
            match = _try_replace_arc(
                points,
                start,
                max_chord_mm=max_chord_mm,
                min_straight_ratio=min_straight_ratio,
                min_corner_angle_deg=min_corner_angle_deg,
                max_corner_angle_deg=max_corner_angle_deg,
                max_residual_mm=max_residual_mm,
                max_arc_span=max_arc_span,
            )
            if match is None:
                continue
            replacement, candidate = match
            points = replacement
            candidates.append(candidate)
            changed = True
            break

    polygon = tuple((_scalar(point[0]), _scalar(point[1])) for point in points)
    if len(polygon) > max_sides:
        # 恢复结果仍违反官方边数上限时不可信，退回原始几何。
        original = tuple(
            (_scalar(point[0]), _scalar(point[1]))
            for point in np.asarray(polygon_mm, dtype=float)
        )
        return RoundedCornerResult(original, ())
    return RoundedCornerResult(polygon, tuple(candidates))


def _try_replace_arc(
    points: list[np.ndarray],
    start: int,
    *,
    max_chord_mm: float,
    min_straight_ratio: float,
    min_corner_angle_deg: float,
    max_corner_angle_deg: float,
    max_residual_mm: float,
    max_arc_span: int,
) -> tuple[list[np.ndarray], RoundedCornerCandidate] | None:
    """尝试把 `start` 处开始的短弧替换为一个虚拟顶点。"""
    count = len(points)
    # 旋转序列，使 rotated[0]->rotated[1] 成为圆弧前的长直边。
    rotated = points[start:] + points[:start]
    straight_before_a, straight_before_b = rotated[0], rotated[1]
    before_vector = straight_before_b - straight_before_a
    before_length = _scalar(np.linalg.norm(before_vector))

    for span in range(1, min(max_arc_span, count - 3) + 1):
        arc_points = [rotated[index] for index in range(1, span + 2)]
        arc_lengths = [
            _scalar(np.linalg.norm(arc_points[index + 1] - arc_points[index]))
            for index in range(len(arc_points) - 1)
        ]
        chord_total = sum(arc_lengths)
        if chord_total <= 0.0 or chord_total > max_chord_mm:
            continue

        straight_after_a = rotated[span + 1]
        straight_after_b = rotated[span + 2]
        after_vector = straight_after_b - straight_after_a
        after_length = _scalar(np.linalg.norm(after_vector))
        threshold = min_straight_ratio * chord_total
        if before_length < threshold or after_length < threshold:
            continue

        cosine = _scalar(np.dot(before_vector, after_vector)) / (
            before_length * after_length
        )
        interior_angle = 180.0 - math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        if not min_corner_angle_deg <= interior_angle <= max_corner_angle_deg:
            continue

        corner = _line_intersection(
            straight_before_a, straight_before_b, straight_after_a, straight_after_b
        )
        if corner is None:
            continue

        # 虚拟顶点必须靠近被替换的圆弧，而不是远处的伪交点。
        distance_to_arc = min(
            _scalar(np.linalg.norm(corner - arc_point)) for arc_point in arc_points
        )
        if distance_to_arc > max(max_chord_mm, 2.5 * chord_total):
            continue

        residual = max(
            _point_line_distance(corner, straight_before_a, straight_before_b),
            _point_line_distance(corner, straight_after_a, straight_after_b),
        )
        if residual > max_residual_mm:
            continue

        chord_depth = min(
            _point_line_distance(arc_point, straight_before_a, straight_before_b)
            for arc_point in arc_points
        )
        replacement = [rotated[0], corner, *rotated[span + 2 :]]
        if len(replacement) < 3:
            continue
        return replacement, RoundedCornerCandidate(
            arc_start_index=start,
            arc_span=span,
            virtual_vertex_mm=(_scalar(corner[0]), _scalar(corner[1])),
            chord_length_mm=chord_total,
            chord_depth_mm=chord_depth,
            fit_angle_deg=interior_angle,
            residual_mm=residual,
        )
    return None
