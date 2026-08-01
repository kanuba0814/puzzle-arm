"""求解后碎片离散排布。

本模块只平移求解器已经确定朝向的多边形，不参与拼图匹配。对每对碎片
构造一个宽度为 ``clearance_mm`` 的分离带，再求离原始紧密拼图最近的确定性
平移解。这样既保留原版拼图解，也能在输出动作计划前验证真实多边形间距。
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


class DiscreteLayoutError(ValueError):
    """无法生成满足间距或目标区域约束的离散布局。"""


@dataclass(frozen=True)
class PairClearance:
    first_index: int
    second_index: int
    clearance_mm: float


@dataclass(frozen=True)
class DiscreteLayout:
    """每片相对紧密拼图的平移，以及最终几何验收结果。"""

    translations_mm: tuple[tuple[float, float], ...]
    pair_clearances: tuple[PairClearance, ...]
    minimum_clearance_mm: float | None
    bounds_mm: tuple[float, float, float, float]

    def as_diagnostics(
        self, piece_ids: Sequence[str], requested_clearance_mm: float
    ) -> dict[str, object]:
        if len(piece_ids) != len(self.translations_mm):
            raise ValueError("piece_ids 数量必须与离散布局一致")
        return {
            "layout_version": "discrete-clearance-v1",
            "requested_clearance_mm": float(requested_clearance_mm),
            "minimum_pair_clearance_mm": self.minimum_clearance_mm,
            "pair_clearances_mm": {
                f"{piece_ids[item.first_index]}|{piece_ids[item.second_index]}": (
                    item.clearance_mm
                )
                for item in self.pair_clearances
            },
            "bounds_table_mm": list(self.bounds_mm),
        }


def _as_polygon(points: Iterable[tuple[float, float]] | np.ndarray) -> np.ndarray:
    polygon = np.asarray(tuple(points), dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3:
        raise DiscreteLayoutError("每个多边形必须至少包含三个二维顶点")
    if not np.isfinite(polygon).all():
        raise DiscreteLayoutError("多边形顶点必须是有限数")
    return polygon


def _polygon_centroid(polygon: np.ndarray) -> np.ndarray:
    following = np.roll(polygon, -1, axis=0)
    cross = polygon[:, 0] * following[:, 1] - following[:, 0] * polygon[:, 1]
    twice_area = float(cross.sum())
    if abs(twice_area) <= 1e-9:
        return polygon.mean(axis=0)
    return ((polygon + following) * cross[:, None]).sum(axis=0) / (3.0 * twice_area)


def _oriented_unit_axis(axis: np.ndarray, center_delta: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-9:
        return None
    unit = axis / norm
    orientation = float(np.dot(unit, center_delta))
    if orientation < -1e-12 or (
        abs(orientation) <= 1e-12
        and (unit[0] < -1e-12 or (abs(unit[0]) <= 1e-12 and unit[1] < 0.0))
    ):
        unit = -unit
    return unit


def _best_separating_axis(
    first: np.ndarray, second: np.ndarray
) -> tuple[np.ndarray, float]:
    """选择最接近原拼图接缝的确定性分离轴及当前投影间距。"""
    center_delta = _polygon_centroid(second) - _polygon_centroid(first)
    candidates: list[np.ndarray] = [center_delta]
    for polygon in (first, second):
        for start, end in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
            edge = end - start
            candidates.append(np.array([-edge[1], edge[0]], dtype=np.float64))

    unique: dict[tuple[float, float], np.ndarray] = {}
    for candidate in candidates:
        axis = _oriented_unit_axis(candidate, center_delta)
        if axis is None:
            continue
        key = (round(float(axis[0]), 10), round(float(axis[1]), 10))
        unique.setdefault(key, axis)
    if not unique:
        raise DiscreteLayoutError("无法确定碎片间分离方向")

    ranked: list[tuple[float, float, float, np.ndarray]] = []
    center_norm = float(np.linalg.norm(center_delta))
    for key, axis in unique.items():
        projected_gap = float(np.min(second @ axis) - np.max(first @ axis))
        alignment = (
            0.0
            if center_norm <= 1e-9
            else float(np.dot(axis, center_delta) / center_norm)
        )
        ranked.append((projected_gap, alignment, key[0] * 1e-6 + key[1] * 1e-9, axis))
    projected_gap, _, _, axis = max(ranked, key=lambda item: item[:3])
    return axis, projected_gap


def _minimum_norm_offsets(
    rows: np.ndarray, limits: np.ndarray, piece_count: int
) -> np.ndarray:
    """枚举最多六个有效约束，求 ``A x >= b`` 的最小范数确定性解。"""
    dimensions = piece_count * 2
    anchors = np.zeros((2, dimensions), dtype=np.float64)
    anchors[0, 0::2] = 1.0
    anchors[1, 1::2] = 1.0
    anchor_values = np.zeros(2, dtype=np.float64)

    best: tuple[float, tuple[float, ...], np.ndarray] | None = None
    constraint_count = len(rows)
    for active_count in range(constraint_count + 1):
        for active in itertools.combinations(range(constraint_count), active_count):
            if active:
                active_rows = rows[np.asarray(active, dtype=int)]
                equations = np.vstack((anchors, active_rows))
                values = np.concatenate((anchor_values, limits[np.asarray(active, dtype=int)]))
            else:
                equations = anchors
                values = anchor_values

            gram = equations @ equations.T
            dual = np.linalg.lstsq(gram, values, rcond=None)[0]
            offsets = equations.T @ dual
            if np.max(np.abs(equations @ offsets - values)) > 1e-7:
                continue
            if np.any(rows @ offsets < limits - 1e-7):
                continue

            objective = float(np.dot(offsets, offsets))
            canonical = tuple(round(float(value), 12) for value in offsets)
            candidate = (round(objective, 12), canonical, offsets)
            if best is None or candidate[:2] < best[:2]:
                best = candidate

    if best is None:
        raise DiscreteLayoutError("碎片间距约束无可行解")
    return best[2].reshape(piece_count, 2)


def _cross(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _point_on_segment(
    point: np.ndarray, start: np.ndarray, end: np.ndarray, tolerance: float = 1e-9
) -> bool:
    return abs(_cross(end - start, point - start)) <= tolerance and (
        min(start[0], end[0]) - tolerance
        <= point[0]
        <= max(start[0], end[0]) + tolerance
        and min(start[1], end[1]) - tolerance
        <= point[1]
        <= max(start[1], end[1]) + tolerance
    )


def _segments_intersect(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> bool:
    first_side_a = _cross(first_end - first_start, second_start - first_start)
    first_side_b = _cross(first_end - first_start, second_end - first_start)
    second_side_a = _cross(second_end - second_start, first_start - second_start)
    second_side_b = _cross(second_end - second_start, first_end - second_start)
    if first_side_a * first_side_b < -1e-12 and second_side_a * second_side_b < -1e-12:
        return True
    return any(
        (
            abs(side) <= 1e-9
            and _point_on_segment(point, segment_start, segment_end)
        )
        for side, point, segment_start, segment_end in (
            (first_side_a, second_start, first_start, first_end),
            (first_side_b, second_end, first_start, first_end),
            (second_side_a, first_start, second_start, second_end),
            (second_side_b, first_end, second_start, second_end),
        )
    )


def _point_segment_distance(
    point: np.ndarray, start: np.ndarray, end: np.ndarray
) -> float:
    segment = end - start
    denominator = float(np.dot(segment, segment))
    if denominator <= 1e-18:
        return float(np.linalg.norm(point - start))
    fraction = float(np.dot(point - start, segment) / denominator)
    nearest = start + np.clip(fraction, 0.0, 1.0) * segment
    return float(np.linalg.norm(point - nearest))


def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _point_on_segment(point, previous, current):
            return True
        crosses = (current[1] > point[1]) != (previous[1] > point[1])
        if crosses:
            intersection_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < intersection_x:
                inside = not inside
        previous = current
    return inside


def polygon_distance_mm(first: np.ndarray, second: np.ndarray) -> float:
    """返回两个简单多边形的最小欧氏距离；相交、相切或包含时为 0。"""
    first = _as_polygon(first)
    second = _as_polygon(second)
    minimum = math.inf
    for first_start, first_end in zip(first, np.roll(first, -1, axis=0), strict=True):
        for second_start, second_end in zip(
            second, np.roll(second, -1, axis=0), strict=True
        ):
            if _segments_intersect(first_start, first_end, second_start, second_end):
                return 0.0
            minimum = min(
                minimum,
                _point_segment_distance(first_start, second_start, second_end),
                _point_segment_distance(first_end, second_start, second_end),
                _point_segment_distance(second_start, first_start, first_end),
                _point_segment_distance(second_end, first_start, first_end),
            )
    if _point_in_polygon(first[0], second) or _point_in_polygon(second[0], first):
        return 0.0
    return float(minimum)


def plan_discrete_layout(
    polygons: Sequence[Iterable[tuple[float, float]] | np.ndarray],
    *,
    clearance_mm: float,
    target_bounds_mm: tuple[float, float, float, float],
    verification_tolerance_mm: float = 1e-5,
) -> DiscreteLayout:
    """保持各片朝向，为每对多边形生成至少 ``clearance_mm`` 的间距。

    返回平移量已包含最终居中平移，可直接同时应用到对应多边形和握点。
    任何一对间距不足或离散布局超出目标区域时都会失败，不产生动作计划。
    """
    if not math.isfinite(clearance_mm) or clearance_mm < 5.0:
        raise DiscreteLayoutError("clearance_mm 必须是至少 5 mm 的有限数")
    if (
        not math.isfinite(verification_tolerance_mm)
        or verification_tolerance_mm < 0.0
    ):
        raise DiscreteLayoutError("verification_tolerance_mm 必须是非负有限数")
    if not 1 <= len(polygons) <= 4:
        raise DiscreteLayoutError("离散布局只接受 1--4 个多边形")

    min_x, min_y, max_x, max_y = map(float, target_bounds_mm)
    if not all(math.isfinite(value) for value in (min_x, min_y, max_x, max_y)):
        raise DiscreteLayoutError("目标区域边界必须是有限数")
    if min_x >= max_x or min_y >= max_y:
        raise DiscreteLayoutError("目标区域必须是有效矩形")

    prepared = tuple(_as_polygon(polygon) for polygon in polygons)
    piece_count = len(prepared)
    pairs = tuple(itertools.combinations(range(piece_count), 2))

    if not pairs:
        local_offsets = np.zeros((piece_count, 2), dtype=np.float64)
    else:
        constraint_rows: list[np.ndarray] = []
        constraint_limits: list[float] = []
        for first_index, second_index in pairs:
            axis, projected_gap = _best_separating_axis(
                prepared[first_index], prepared[second_index]
            )
            row = np.zeros(piece_count * 2, dtype=np.float64)
            row[first_index * 2 : first_index * 2 + 2] = -axis
            row[second_index * 2 : second_index * 2 + 2] = axis
            constraint_rows.append(row)
            constraint_limits.append(clearance_mm - projected_gap)
        local_offsets = _minimum_norm_offsets(
            np.vstack(constraint_rows),
            np.asarray(constraint_limits, dtype=np.float64),
            piece_count,
        )

    locally_placed = tuple(
        polygon + local_offsets[index] for index, polygon in enumerate(prepared)
    )
    all_points = np.vstack(locally_placed)
    lower = all_points.min(axis=0)
    upper = all_points.max(axis=0)
    extent = upper - lower
    target_extent = np.array([max_x - min_x, max_y - min_y], dtype=np.float64)
    if np.any(extent > target_extent + verification_tolerance_mm):
        raise DiscreteLayoutError(
            "离散布局超出目标区域: "
            f"layout={extent[0]:.3f}x{extent[1]:.3f}mm, "
            f"target={target_extent[0]:.3f}x{target_extent[1]:.3f}mm"
        )

    target_center = np.array([(min_x + max_x) / 2.0, (min_y + max_y) / 2.0])
    layout_center = (lower + upper) / 2.0
    center_shift = target_center - layout_center
    translations = local_offsets + center_shift
    placed = tuple(
        polygon + translations[index] for index, polygon in enumerate(prepared)
    )
    final_points = np.vstack(placed)
    final_lower = final_points.min(axis=0)
    final_upper = final_points.max(axis=0)

    clearances = tuple(
        PairClearance(
            first_index,
            second_index,
            polygon_distance_mm(placed[first_index], placed[second_index]),
        )
        for first_index, second_index in pairs
    )
    minimum_clearance = (
        min(item.clearance_mm for item in clearances) if clearances else None
    )
    if minimum_clearance is not None and (
        minimum_clearance < clearance_mm - verification_tolerance_mm
    ):
        raise DiscreteLayoutError(
            "离散布局间距验收失败: "
            f"minimum={minimum_clearance:.6f}mm, required={clearance_mm:.6f}mm"
        )
    if (
        final_lower[0] < min_x - verification_tolerance_mm
        or final_lower[1] < min_y - verification_tolerance_mm
        or final_upper[0] > max_x + verification_tolerance_mm
        or final_upper[1] > max_y + verification_tolerance_mm
    ):
        raise DiscreteLayoutError("居中后的离散布局超出目标区域")

    return DiscreteLayout(
        translations_mm=tuple(
            (float(translation[0]), float(translation[1]))
            for translation in translations
        ),
        pair_clearances=clearances,
        minimum_clearance_mm=minimum_clearance,
        bounds_mm=(
            float(final_lower[0]),
            float(final_lower[1]),
            float(final_upper[0]),
            float(final_upper[1]),
        ),
    )
