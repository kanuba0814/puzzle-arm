"""最多四片的确定性毫米域拼图求解器。

求解器只消费观察几何，不读取夹具真值、图像或文件，也不执行动作。
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from enum import Enum
from typing import Protocol

import numpy as np

from .discrete_layout import (
    DiscreteLayout,
    DiscreteLayoutError,
    plan_discrete_layout,
)


class _PieceLike(Protocol):
    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float
    timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]


class PuzzleSolveStatus(str, Enum):
    SOLVED = "SOLVED"
    AMBIGUOUS = "AMBIGUOUS"
    NO_SOLUTION = "NO_SOLUTION"


# 旋转基准约定（2026-08-01）：不测量碎片绝对朝向，相机拍下的初始位置
# 一律视为 0°。PlacementPlan.source.yaw_deg 恒为 0，target.yaw_deg 即
# 求解器相对旋转角（拼图只需要碎片间的相对方向）。
SOURCE_YAW_BASELINE_DEG = 0.0


@dataclass(frozen=True)
class Pose2:
    frame: str
    center_mm: tuple[float, float]
    yaw_deg: float


@dataclass(frozen=True)
class TargetRegion:
    frame: str
    min_x_mm: float
    min_y_mm: float
    max_x_mm: float
    max_y_mm: float

    def __post_init__(self) -> None:
        if (
            self.frame != "table_mm"
            or self.min_x_mm >= self.max_x_mm
            or self.min_y_mm >= self.max_y_mm
        ):
            raise ValueError("TargetRegion 必须是有效 table_mm 矩形")


@dataclass(frozen=True)
class PuzzleOptions:
    time_budget_ms: int = 10000
    max_expansions: int = 10000
    target_width_range_mm: tuple[float, float] = (90.0, 120.0)
    target_height_range_mm: tuple[float, float] = (50.0, 90.0)
    ambiguity_score_gap: float | None = None
    partial_offset_step_mm: float = 1.0
    abs_tolerance_mm: float = 2.0
    rel_tolerance: float = 0.03
    vertex_tolerance_mm: float = 2.0
    # 原版几何完成拼图匹配后，只平移目标位形成片间最小净距。
    # 默认 5 mm 是执行计划约束，不参与接缝匹配和几何评分。
    placement_gap_mm: float = 5.0
    # 相邻碎片接缝处的可容忍重叠深度：视觉多边形顶点有 ≈±1 mm 测量偏差，
    # 零容忍重叠剪枝会把正确装配整体判死（合成复现）；官方验收容差为
    # 顶点距离 ≤ 20 mm，0.75 mm 远在安全范围内。
    overlap_depth_tolerance_mm: float = 0.75

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.placement_gap_mm)
            or self.placement_gap_mm < 5.0
        ):
            raise ValueError("placement_gap_mm 必须是至少 5 mm 的有限数")


@dataclass(frozen=True)
class TextureEvidence:
    rectified_image: np.ndarray
    piece_masks: dict[str, np.ndarray]


@dataclass(frozen=True)
class WorkspaceState:
    pieces: tuple[_PieceLike, ...]
    locked_placements: tuple[object, ...] = ()


@dataclass(frozen=True)
class PlacementPlan:
    piece_id: str
    order: int
    source: Pose2
    target: Pose2
    geometry_score: float
    texture_score: float | None
    texture_status: str
    plan_confidence: float
    source_timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]
    solver_version: str = "puzzle-geometry-v2"

    def as_mapping(self) -> dict[str, object]:
        return {
            "piece_id": self.piece_id,
            "order": self.order,
            "source": {
                "frame": self.source.frame,
                "center_mm": list(self.source.center_mm),
                "yaw_deg": self.source.yaw_deg,
            },
            "target": {
                "frame": self.target.frame,
                "center_mm": list(self.target.center_mm),
                "yaw_deg": self.target.yaw_deg,
            },
            "geometry_score": self.geometry_score,
            "texture_score": self.texture_score,
            "texture_status": self.texture_status,
            "texture_status_reason": "texture disabled",
            "plan_confidence": self.plan_confidence,
            "source_timestamp_ns": self.source_timestamp_ns,
            "equivalent_yaws_deg": list(self.equivalent_yaws_deg),
            "solver_version": self.solver_version,
        }


@dataclass(frozen=True)
class PuzzleSolveResult:
    status: PuzzleSolveStatus
    error_code: str | None
    inferred_target: tuple[float, float] | None
    placements: tuple[PlacementPlan, ...]
    candidates_after_dedup: int
    best_score: float | None
    second_best_score: float | None
    score_gap: float | None
    score_breakdown: dict[str, float]
    texture_executed: bool
    solver_version: str
    profile_version: str
    calibration_refs: dict[str, str]
    timing_ms: dict[str, float]
    expansions: int
    pruned: int
    diagnostics: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class EdgeDescriptor:
    piece_id: str
    index: int
    start: tuple[float, float]
    end: tuple[float, float]
    length_mm: float
    direction_rad: float


@dataclass(frozen=True)
class EdgeCandidate:
    """两条反向贴合边及长边上的接触起点。

    对部分边，first 永远是短边、second 是长边；offset_mm 相对 second.start。
    """

    first: EdgeDescriptor
    second: EdgeDescriptor
    kind: str
    offset_mm: float


def _polygon(piece: _PieceLike) -> np.ndarray:
    array = np.asarray(piece.polygon_table_mm, dtype=np.float64)
    if array.ndim != 2 or array.shape[1] != 2 or not 3 <= len(array) <= 5:
        raise ValueError("每个 piece 必须提供 3--5 个 polygon_table_mm 顶点")
    if (
        not np.isfinite(array).all()
        or _polygon_area_signed(array) <= 1e-6
        or not _simple_polygon(array)
    ):
        raise ValueError("piece polygon 必须是有限、简单且面积为正的多边形")
    return array


def _polygon_area_signed(points: np.ndarray) -> float:
    return (
        abs(
            float(
                np.dot(points[:, 0], np.roll(points[:, 1], -1))
                - np.dot(points[:, 1], np.roll(points[:, 0], -1))
            )
        )
        / 2
    )


def _cross(first: np.ndarray, second: np.ndarray) -> float:
    return float(first[0] * second[1] - first[1] * second[0])


def _segments_intersect(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> bool:
    """严格交叉；公共端点和共线接缝不算自交。"""
    ab, ac, ad = b - a, c - a, d - a
    cd, ca, cb = d - c, a - c, b - c
    first = _cross(ab, ac) * _cross(ab, ad)
    second = _cross(cd, ca) * _cross(cd, cb)
    return first < -1e-8 and second < -1e-8


def _simple_polygon(points: np.ndarray) -> bool:
    count = len(points)
    for first in range(count):
        for second in range(first + 1, count):
            if second in (first, (first + 1) % count) or first == (second + 1) % count:
                continue
            if _segments_intersect(
                points[first],
                points[(first + 1) % count],
                points[second],
                points[(second + 1) % count],
            ):
                return False
    return True


def describe_edges(piece: _PieceLike) -> tuple[EdgeDescriptor, ...]:
    polygon = _polygon(piece)
    descriptors: list[EdgeDescriptor] = []
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % len(polygon)]
        vector = end - start
        length = float(np.linalg.norm(vector))
        if length > 1e-8:
            descriptors.append(
                EdgeDescriptor(
                    str(piece.piece_id),
                    index,
                    tuple(start),
                    tuple(end),
                    length,
                    math.atan2(vector[1], vector[0]),
                )
            )
    return tuple(descriptors)


def edge_candidates(
    first_piece: _PieceLike, second_piece: _PieceLike, options: PuzzleOptions
) -> tuple[EdgeCandidate, ...]:
    """枚举完整、端点和内部部分边；候选从不以全局 top-N 截断。"""
    result: list[EdgeCandidate] = []
    for first in describe_edges(first_piece):
        for second in describe_edges(second_piece):
            tolerance = options.abs_tolerance_mm + options.rel_tolerance * min(
                first.length_mm, second.length_mm
            )
            if abs(first.length_mm - second.length_mm) <= tolerance:
                result.append(EdgeCandidate(first, second, "complete", 0.0))
                continue
            short, long = (
                (first, second)
                if first.length_mm < second.length_mm
                else (second, first)
            )
            difference = long.length_mm - short.length_mm
            # A partial contact must leave legal external-edge residuals at internal offsets.
            if difference < 1e-8:
                continue
            result.extend(
                (
                    EdgeCandidate(short, long, "endpoint", 0.0),
                    EdgeCandidate(short, long, "endpoint", difference),
                )
            )
            minimum_residual = max(0.0, 20.0 - options.vertex_tolerance_mm)
            upper = difference - minimum_residual
            if upper >= minimum_residual:
                step = max(0.1, options.partial_offset_step_mm)
                offsets = [minimum_residual, upper]
                value = math.ceil(minimum_residual / step) * step
                while value < upper:
                    offsets.append(round(value, 6))
                    value += step
                result.extend(
                    EdgeCandidate(short, long, "internal", value)
                    for value in sorted(set(offsets))
                )
    return tuple(
        sorted(
            result,
            key=lambda item: (
                item.first.piece_id,
                item.second.piece_id,
                item.first.index,
                item.second.index,
                item.kind,
                item.offset_mm,
            ),
        )
    )


def se2_align(
    first: EdgeDescriptor, second: EdgeDescriptor, offset_mm: float = 0.0
) -> tuple[np.ndarray, np.ndarray]:
    """将 second 反向贴合 first；仅返回 det(R)=+1 的刚体变换。"""
    angle = first.direction_rad - (second.direction_rad + math.pi)
    rotation = np.array(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]],
        dtype=np.float64,
    )
    target = np.asarray(first.start) + offset_mm * np.array(
        [math.cos(first.direction_rad), math.sin(first.direction_rad)]
    )
    return rotation, target - rotation @ np.asarray(second.end)


def _is_rectangle(polygon: np.ndarray, tolerance: float) -> bool:
    if len(polygon) != 4:
        return False
    vectors = np.roll(polygon, -1, axis=0) - polygon
    lengths = np.linalg.norm(vectors, axis=1)
    return bool(
        np.all(lengths > 1e-6)
        and np.all(
            np.abs(np.sum(vectors * np.roll(vectors, -1, axis=0), axis=1))
            <= tolerance * np.maximum(lengths * np.roll(lengths, -1), 1.0)
        )
    )


def _empty_result(
    error_code: str, elapsed: float, expansions: int, pruned: int, texture: bool = False
) -> PuzzleSolveResult:
    return PuzzleSolveResult(
        PuzzleSolveStatus.NO_SOLUTION,
        error_code,
        None,
        (),
        0,
        None,
        None,
        None,
        {
            "edge_match": 0.0,
            "overlap": 0.0,
            "rectangle_gap": 0.0,
            "outside_target": 0.0,
        },
        texture,
        "puzzle-geometry-v2",
        "geometry-only",
        {},
        {"solve": elapsed},
        expansions,
        pruned,
    )


def _single_piece_solution(
    piece: _PieceLike, target: TargetRegion, options: PuzzleOptions
) -> PuzzleSolveResult | None:
    polygon = _polygon(piece)
    if not _is_rectangle(polygon, options.abs_tolerance_mm):
        return None
    extents = polygon.max(axis=0) - polygon.min(axis=0)
    dimensions = sorted((float(extents[0]), float(extents[1])))
    if not (
        options.target_width_range_mm[0] - options.abs_tolerance_mm
        <= dimensions[1]
        <= options.target_width_range_mm[1] + options.abs_tolerance_mm
        and options.target_height_range_mm[0] - options.abs_tolerance_mm
        <= dimensions[0]
        <= options.target_height_range_mm[1] + options.abs_tolerance_mm
    ):
        return None
    try:
        layout = plan_discrete_layout(
            (polygon,),
            clearance_mm=options.placement_gap_mm,
            target_bounds_mm=(
                target.min_x_mm,
                target.min_y_mm,
                target.max_x_mm,
                target.max_y_mm,
            ),
        )
    except DiscreteLayoutError:
        return _empty_result("SOLVER_DISCRETE_LAYOUT_FAILED", 0.0, 0, 0)
    target_reference = np.asarray(piece.center_table_mm) + np.asarray(
        layout.translations_mm[0]
    )
    plan = PlacementPlan(
        str(piece.piece_id),
        0,
        Pose2("table_mm", piece.center_table_mm, SOURCE_YAW_BASELINE_DEG),
        Pose2(
            "table_mm",
            (float(target_reference[0]), float(target_reference[1])),
            SOURCE_YAW_BASELINE_DEG,
        ),  # 单片不旋转
        0.0,
        None,
        "not_run",
        1.0,
        int(piece.timestamp_ns),
        tuple(piece.equivalent_yaws_deg),
    )
    return PuzzleSolveResult(
        PuzzleSolveStatus.SOLVED,
        None,
        tuple(extents),
        (plan,),
        1,
        0.0,
        None,
        None,
        {
            "edge_match": 0.0,
            "overlap": 0.0,
            "rectangle_gap": 0.0,
            "outside_target": 0.0,
            "placement_gap_mm": options.placement_gap_mm,
        },
        False,
        "puzzle-geometry-v2+discrete-v1",
        "geometry-only",
        {},
        {"solve": 0.0},
        0,
        0,
        {
            "extra": layout.as_diagnostics(
                (str(piece.piece_id),), options.placement_gap_mm
            )
        },
    )


def _transform(
    points: np.ndarray, rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    return (rotation @ points.T).T + translation


def _point_in_polygon(point: tuple[float, float], polygon: np.ndarray) -> bool:
    inside = False
    x_value, y_value = point
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > y_value) != (previous[1] > y_value):
            crossing = (previous[0] - current[0]) * (y_value - current[1]) / (
                previous[1] - current[1]
            ) + current[0]
            if x_value < crossing:
                inside = not inside
        previous = current
    return inside


def _points_in_polygon(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """向量化射线法；与 `_point_in_polygon` 同一交叉规则与运算顺序。

    纯 Python 逐点栅格重叠检查是搜索期主要开销（树莓派 5 上单次扩展毫秒级），
    多朝向种子搜索必须依赖本函数才能留在 10 s 预算内。
    """
    if len(points) == 0:
        return np.zeros(0, dtype=bool)
    x_values = points[:, 0][:, None]
    y_values = points[:, 1][:, None]
    current = polygon[None, :, :]
    previous = np.roll(polygon, 1, axis=0)[None, :, :]
    current_y = current[..., 1]
    previous_y = previous[..., 1]
    crosses = (current_y > y_values) != (previous_y > y_values)
    with np.errstate(divide="ignore", invalid="ignore"):
        crossing = (previous[..., 0] - current[..., 0]) * (
            y_values - current_y
        ) / (previous_y - current_y) + current[..., 0]
    hits = crosses & (x_values < crossing)
    return hits.sum(axis=1) % 2 == 1


def _raster_quality(
    polygons: tuple[np.ndarray, ...], lower: np.ndarray, upper: np.ndarray
) -> tuple[float, float]:
    """1 mm 固定栅格检查矩形覆盖和真实片间重叠，避免凸包近似。"""
    step = 1.0
    x_values = np.arange(lower[0] + step / 2, upper[0], step)
    y_values = np.arange(lower[1] + step / 2, upper[1], step)
    total = max(1, len(x_values) * len(y_values))
    if len(x_values) == 0 or len(y_values) == 0:
        return 1.0, 0.0
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    grid = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    counts = np.zeros(len(grid), dtype=np.int64)
    for polygon in polygons:
        counts += _points_in_polygon(grid, polygon)
    covered = int(np.count_nonzero(counts > 0))
    overlap = int(np.count_nonzero(counts > 1))
    return 1.0 - covered / total, overlap / total


def _min_distance_to_edges(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """每个采样点到多边形边界（线段集）的最小距离；向量化。"""
    starts = polygon
    ends = np.roll(polygon, -1, axis=0)
    segment = ends - starts
    to_point = points[:, None, :] - starts[None, :, :]
    squared = np.maximum((segment * segment).sum(axis=1), 1e-12)
    ratio = np.clip((to_point * segment[None]).sum(axis=2) / squared, 0.0, 1.0)
    closest = starts[None] + ratio[..., None] * segment[None]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(axis=1)


def _positive_overlap(
    first: np.ndarray, second: np.ndarray, depth_tolerance_mm: float = 0.75
) -> bool:
    """只在两片 AABB 的交集采样，供搜索期快速拒绝。

    真重叠要求存在同时深入两片边界超过 depth_tolerance_mm 的采样点；
    接缝处由视觉顶点噪声造成的亚毫米薏片不算重叠。
    """
    lower = np.maximum(first.min(axis=0), second.min(axis=0))
    upper = np.minimum(first.max(axis=0), second.max(axis=0))
    if np.any(upper - lower <= 1e-6):
        return False
    # 1 mm centre sampling: valid contact is a line and不会落入采样点。
    x_values = np.arange(lower[0] + 0.5, upper[0], 1.0)
    y_values = np.arange(lower[1] + 0.5, upper[1], 1.0)
    if len(x_values) == 0 or len(y_values) == 0:
        return False
    grid_x, grid_y = np.meshgrid(x_values, y_values)
    grid = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    inside_first = _points_in_polygon(grid, first)
    if not inside_first.any():
        return False
    shared = grid[inside_first]
    inside_second = _points_in_polygon(shared, second)
    if not inside_second.any():
        return False
    shared = shared[inside_second]
    deep_first = _min_distance_to_edges(shared, first) > depth_tolerance_mm
    if not deep_first.any():
        return False
    deep = shared[deep_first]
    return bool((_min_distance_to_edges(deep, second) > depth_tolerance_mm).any())


def _assembly_score(
    polygons: Iterable[np.ndarray], target: TargetRegion, options: PuzzleOptions
) -> tuple[float, tuple[float, float], dict[str, float]] | None:
    assembled = tuple(polygons)
    all_points = np.concatenate(assembled)
    lower, upper = all_points.min(axis=0), all_points.max(axis=0)
    width, height = (float(value) for value in upper - lower)
    if not (
        options.target_width_range_mm[0] - options.abs_tolerance_mm
        <= width
        <= options.target_width_range_mm[1] + options.abs_tolerance_mm
        and options.target_height_range_mm[0] - options.abs_tolerance_mm
        <= height
        <= options.target_height_range_mm[1] + options.abs_tolerance_mm
    ):
        return None
    # The final assembly is translated to the target centre. Translation never repairs an oversized target.
    if (
        width > target.max_x_mm - target.min_x_mm + options.abs_tolerance_mm
        or height > target.max_y_mm - target.min_y_mm + options.abs_tolerance_mm
    ):
        return None
    area_sum = sum(_polygon_area_signed(polygon) for polygon in assembled)
    rectangle_area = width * height
    area_error = abs(area_sum - rectangle_area) / max(rectangle_area, 1.0)
    allowed_error = max(0.01, options.abs_tolerance_mm / max(min(width, height), 1.0))
    # Most incorrect edge pairings have an oversized bounding rectangle.  Reject
    # them analytically before invoking the deliberately conservative raster test.
    if area_error > allowed_error:
        return None
    if all(_is_rectangle(polygon, options.abs_tolerance_mm) for polygon in assembled):
        # During search every inserted rectangle has already passed pairwise
        # positive-overlap rejection.  Equal total/bounding-box area therefore
        # proves rectangular coverage without an expensive repeated raster pass.
        gap, overlap = 0.0, 0.0
    else:
        gap, overlap = _raster_quality(assembled, lower, upper)
        if gap > allowed_error or overlap > allowed_error:
            return None
    score = gap + overlap + area_error
    return (
        score,
        (width, height),
        {
            "edge_match": 0.0,
            "overlap": overlap,
            "rectangle_gap": gap,
            "area_conservation": area_error,
            "outside_target": 0.0,
            "complexity": 0.0,
        },
    )


def _yaw_from_rotation(rotation: np.ndarray, yaw_deg: float) -> float:
    """把 2×2 旋转矩阵的旋转角叠加到基准角并归一化到 [-180, 180)。

    0° 基准约定下调用方传 SOURCE_YAW_BASELINE_DEG，返回即相对旋转角。
    """
    return (
        yaw_deg + math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])) + 180.0
    ) % 360.0 - 180.0


def _edge_offsets(
    candidate: EdgeCandidate, anchor: EdgeDescriptor, new: EdgeDescriptor
) -> tuple[float, float, float]:
    if candidate.kind == "complete":
        return 0.0, 0.0, anchor.length_mm
    if (
        candidate.first.piece_id == anchor.piece_id
        and candidate.first.index == anchor.index
    ):
        return 0.0, candidate.offset_mm, candidate.first.length_mm
    return candidate.offset_mm, 0.0, candidate.first.length_mm


def _align_contact(
    anchor: EdgeDescriptor, new: EdgeDescriptor, anchor_offset: float, new_offset: float
) -> tuple[np.ndarray, np.ndarray]:
    angle = anchor.direction_rad - (new.direction_rad + math.pi)
    rotation = np.array(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]],
        dtype=np.float64,
    )
    unit_anchor = np.array(
        [math.cos(anchor.direction_rad), math.sin(anchor.direction_rad)]
    )
    unit_new = np.array([math.cos(new.direction_rad), math.sin(new.direction_rad)])
    target = np.asarray(anchor.start) + anchor_offset * unit_anchor
    # The two boundary walks are opposite.  ``new_offset`` is measured from
    # new.start in its own walk direction, hence contact begins at end-offset.
    new_contact = np.asarray(new.start) + (new.length_mm - new_offset) * unit_new
    return rotation, target - rotation @ new_contact


def _interval_ok(
    occupancy: dict[tuple[str, int], tuple[tuple[float, float], ...]],
    edge: EdgeDescriptor,
    start: float,
    length: float,
) -> bool:
    end = start + length
    if start < -1e-6 or end > edge.length_mm + 1e-6:
        return False
    return all(
        min(end, used_end) - max(start, used_start) <= 1e-6
        for used_start, used_end in occupancy.get((edge.piece_id, edge.index), ())
    )


def _assembly_key(
    pieces: tuple[_PieceLike, ...], transforms: dict[str, tuple[np.ndarray, np.ndarray]]
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """去除平移、全局 90° 旋转和同形片 identity 的几何等价类键。"""
    transformed = [
        _transform(_polygon(piece), *transforms[piece.piece_id]) for piece in pieces
    ]
    variants: list[tuple[tuple[tuple[float, float], ...], ...]] = []
    for quarter_turn in range(4):
        angle = quarter_turn * math.pi / 2
        rotation = np.array(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )
        rotated = [polygon @ rotation.T for polygon in transformed]
        lower = np.concatenate(rotated).min(axis=0)
        encoded = []
        for polygon in rotated:
            normalized = polygon - lower
            possibilities = [
                tuple(
                    (round(float(point[0]), 5), round(float(point[1]), 5))
                    for point in np.roll(normalized, -index, axis=0)
                )
                for index in range(len(normalized))
            ]
            encoded.append(min(possibilities))
        variants.append(tuple(sorted(encoded)))
    return min(variants)


def _plans_for_assembly(
    pieces: tuple[_PieceLike, ...],
    transforms: dict[str, tuple[np.ndarray, np.ndarray]],
    target: TargetRegion,
    score: float,
    placement_gap_mm: float,
) -> tuple[tuple[PlacementPlan, ...], DiscreteLayout]:
    ordered_pieces = tuple(sorted(pieces, key=lambda item: item.piece_id))
    transformed = tuple(
        _transform(_polygon(piece), *transforms[piece.piece_id])
        for piece in ordered_pieces
    )
    layout = plan_discrete_layout(
        transformed,
        clearance_mm=placement_gap_mm,
        target_bounds_mm=(
            target.min_x_mm,
            target.min_y_mm,
            target.max_x_mm,
            target.max_y_mm,
        ),
    )
    plans: list[PlacementPlan] = []
    # Piece ID is tie-breaker；离散模块只增加平移，不改变求解器给出的旋转。
    for order, piece in enumerate(ordered_pieces):
        rotation, translation = transforms[piece.piece_id]
        target_center = (
            rotation @ np.asarray(piece.center_table_mm)
            + translation
            + np.asarray(layout.translations_mm[order])
        )
        plans.append(
            PlacementPlan(
                piece.piece_id,
                order,
                Pose2("table_mm", piece.center_table_mm, SOURCE_YAW_BASELINE_DEG),
                Pose2(
                    "table_mm",
                    (float(target_center[0]), float(target_center[1])),
                    _yaw_from_rotation(rotation, SOURCE_YAW_BASELINE_DEG),
                ),
                score,
                None,
                "not_run",
                max(0.0, 1.0 - score),
                int(piece.timestamp_ns),
                tuple(piece.equivalent_yaws_deg),
            )
        )
    return tuple(plans), layout


def solve_puzzle(
    workspace: WorkspaceState,
    target_region: TargetRegion,
    options: PuzzleOptions,
    texture_evidence: TextureEvidence | None = None,
    *,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
) -> PuzzleSolveResult:
    """搜索 2--4 片连通装配；超预算及非唯一结果都不产生可执行计划。"""
    started = monotonic_ns()
    pieces = tuple(sorted(workspace.pieces, key=lambda piece: piece.piece_id))
    if not 1 <= len(pieces) <= 4:
        raise ValueError("solve_puzzle 只接受 1--4 个碎片")
    if options.max_expansions <= 0 or options.time_budget_ms <= 0:
        raise ValueError("求解预算必须为正")
    for piece in pieces:
        _polygon(piece)
    if len(pieces) == 1:
        answer = _single_piece_solution(pieces[0], target_region, options)
        if answer is not None:
            return answer

    pair_candidates = {
        (first.piece_id, second.piece_id): edge_candidates(first, second, options)
        for index, first in enumerate(pieces)
        for second in pieces[index + 1 :]
    }
    raw_solutions: list[
        tuple[
            float,
            tuple[float, float],
            dict[str, float],
            dict[str, tuple[np.ndarray, np.ndarray]],
        ]
    ] = []
    expansions = 0
    pruned = 0
    timed_out = False

    def expired() -> bool:
        return (
            expansions >= options.max_expansions
            or (monotonic_ns() - started) / 1_000_000 > options.time_budget_ms
        )

    def search(
        placed: tuple[_PieceLike, ...],
        transforms: dict[str, tuple[np.ndarray, np.ndarray]],
        occupancy: dict[tuple[str, int], tuple[tuple[float, float], ...]],
    ) -> None:
        nonlocal expansions, pruned, timed_out
        if timed_out:
            return
        if expired():
            timed_out = True
            return
        if len(placed) == len(pieces):
            scored = _assembly_score(
                (
                    _transform(_polygon(piece), *transforms[piece.piece_id])
                    for piece in pieces
                ),
                target_region,
                options,
            )
            if scored is None:
                pruned += 1
            else:
                raw_solutions.append((*scored, dict(transforms)))
            return
        for new_piece in pieces:
            if new_piece.piece_id in transforms:
                continue
            # Different geometric edge descriptors can imply the same SE(2)
            # pose.  Deduplicate only this local expansion, never candidate
            # generation, so valid pair candidates remain available elsewhere.
            tried_poses: set[tuple[float, ...]] = set()
            for anchor_piece in placed:
                if anchor_piece.piece_id < new_piece.piece_id:
                    key: tuple[str, str] = (anchor_piece.piece_id, new_piece.piece_id)
                else:
                    key = (new_piece.piece_id, anchor_piece.piece_id)
                candidates = pair_candidates[key]
                # Exact shared edges dominate arbitrary partial contacts for this
                # pair.  Keeping the latter would create a combinatorial cloud of
                # physically inferior placements around a known complete seam.
                if any(candidate.kind == "complete" for candidate in candidates):
                    candidates = tuple(
                        candidate
                        for candidate in candidates
                        if candidate.kind == "complete"
                    )
                for candidate in candidates:
                    if candidate.first.piece_id == anchor_piece.piece_id:
                        anchor_edge, new_edge = candidate.first, candidate.second
                    else:
                        anchor_edge, new_edge = candidate.second, candidate.first
                    anchor_rotation, anchor_translation = transforms[
                        anchor_piece.piece_id
                    ]
                    global_start = (
                        anchor_rotation @ np.asarray(anchor_edge.start)
                        + anchor_translation
                    )
                    global_end = (
                        anchor_rotation @ np.asarray(anchor_edge.end)
                        + anchor_translation
                    )
                    global_anchor = EdgeDescriptor(
                        anchor_edge.piece_id,
                        anchor_edge.index,
                        tuple(global_start),
                        tuple(global_end),
                        anchor_edge.length_mm,
                        anchor_edge.direction_rad
                        + math.atan2(anchor_rotation[1, 0], anchor_rotation[0, 0]),
                    )
                    anchor_offset, new_offset, contact_length = _edge_offsets(
                        candidate, anchor_edge, new_edge
                    )
                    if not _interval_ok(
                        occupancy, anchor_edge, anchor_offset, contact_length
                    ) or not _interval_ok(
                        occupancy, new_edge, new_offset, contact_length
                    ):
                        pruned += 1
                        continue
                    rotation, translation = _align_contact(
                        global_anchor, new_edge, anchor_offset, new_offset
                    )
                    pose_key = tuple(
                        round(float(value), 6)
                        for value in (*rotation.ravel(), *translation)
                    )
                    if pose_key in tried_poses:
                        continue
                    tried_poses.add(pose_key)
                    expansions += 1
                    candidate_polygon = _transform(
                        _polygon(new_piece), rotation, translation
                    )
                    existing = tuple(
                        _transform(_polygon(piece), *transforms[piece.piece_id])
                        for piece in placed
                    )
                    # Fast overlap prune: a contact boundary has zero area; any sampled positive overlap is invalid.
                    if any(
                        _positive_overlap(
                            existing_polygon,
                            candidate_polygon,
                            options.overlap_depth_tolerance_mm,
                        )
                        for existing_polygon in existing
                    ):
                        pruned += 1
                        continue
                    partial_points = np.concatenate((*existing, candidate_polygon))
                    partial_extent = partial_points.max(axis=0) - partial_points.min(
                        axis=0
                    )
                    if (
                        partial_extent[0]
                        > options.target_width_range_mm[1] + options.abs_tolerance_mm
                        or partial_extent[1]
                        > options.target_height_range_mm[1] + options.abs_tolerance_mm
                    ):
                        pruned += 1
                        continue
                    next_transforms = dict(transforms)
                    next_transforms[new_piece.piece_id] = (rotation, translation)
                    next_occupancy = dict(occupancy)
                    next_occupancy[(anchor_edge.piece_id, anchor_edge.index)] = (
                        occupancy.get((anchor_edge.piece_id, anchor_edge.index), ())
                        + ((anchor_offset, anchor_offset + contact_length),)
                    )
                    next_occupancy[(new_edge.piece_id, new_edge.index)] = occupancy.get(
                        (new_edge.piece_id, new_edge.index), ()
                    ) + ((new_offset, new_offset + contact_length),)
                    search((*placed, new_piece), next_transforms, next_occupancy)
                    if timed_out:
                        return

    # 官方约束：每片至少 1 条边位于目标矩形外边上，而评分/剪枝只接受轴对齐
    # 装配。散放碎片朝向任意，单一 identity 种子会对旋转摆放的碎片系统性
    # NO_SOLUTION（合成复现：同一切割 0° 可解、整体旋转后无解）。因此把根片
    # 每条边分别对齐 x/y 轴作为确定性种子集；其中至少一个使真实装配轴对齐，
    # 180° 变体由 _assembly_key 的 90° 规范化去重。
    root = pieces[0]
    seed_angles: list[float] = []
    for edge in describe_edges(root):
        for axis_rad in (0.0, math.pi / 2):
            angle = (axis_rad - edge.direction_rad) % (2.0 * math.pi)
            if all(
                abs(angle - existing) > 1e-9 for existing in seed_angles
            ):
                seed_angles.append(angle)
    for seed_angle in seed_angles:
        seed_rotation = np.array(
            [
                [math.cos(seed_angle), -math.sin(seed_angle)],
                [math.sin(seed_angle), math.cos(seed_angle)],
            ],
            dtype=np.float64,
        )
        search((root,), {root.piece_id: (seed_rotation, np.zeros(2))}, {})
        if timed_out:
            break
    elapsed = (monotonic_ns() - started) / 1_000_000
    if timed_out:
        return _empty_result(
            "SOLVER_TIMEOUT", elapsed, expansions, pruned, texture_evidence is not None
        )
    deduped: dict[
        tuple[tuple[tuple[float, float], ...], ...],
        tuple[
            float,
            tuple[float, float],
            dict[str, float],
            dict[str, tuple[np.ndarray, np.ndarray]],
        ],
    ] = {}
    for solution in raw_solutions:
        key = _assembly_key(pieces, solution[3])
        if key not in deduped or solution[0] < deduped[key][0]:
            deduped[key] = solution
    solutions = sorted(
        deduped.values(),
        key=lambda item: (round(item[0], 9), _assembly_key(pieces, item[3])),
    )
    if not solutions:
        return _empty_result(
            "SOLVER_NO_GEOMETRIC_ASSEMBLY",
            elapsed,
            expansions,
            pruned,
            texture_evidence is not None,
        )
    best_score, dimensions, breakdown, transforms = solutions[0]
    second_score = solutions[1][0] if len(solutions) > 1 else None
    # gap 用与排序一致的 9 位舍入分数计算：浮点尘埃（~1e-16）曾使并列最优解
    # 产生负 gap，把 ambiguity_score_gap=0.0（“从不判歧义”）误判为 AMBIGUOUS。
    gap = (
        None
        if second_score is None
        else round(second_score, 9) - round(best_score, 9)
    )
    is_ambiguous = False
    if second_score is not None:
        is_ambiguous = options.ambiguity_score_gap is None or (
            gap is not None and gap < options.ambiguity_score_gap
        )
    if is_ambiguous:
        return PuzzleSolveResult(
            PuzzleSolveStatus.AMBIGUOUS,
            "SOLVER_AMBIGUOUS",
            dimensions,
            (),
            len(solutions),
            best_score,
            second_score,
            gap,
            breakdown,
            texture_evidence is not None,
            "puzzle-geometry-v2",
            "geometry-only",
            {},
            {"solve": elapsed},
            expansions,
            pruned,
        )
    try:
        placements, layout = _plans_for_assembly(
            pieces, transforms, target_region, best_score, options.placement_gap_mm
        )
    except DiscreteLayoutError:
        return _empty_result(
            "SOLVER_DISCRETE_LAYOUT_FAILED",
            elapsed,
            expansions,
            pruned,
            texture_evidence is not None,
        )
    layout_breakdown = dict(breakdown)
    layout_breakdown["placement_gap_mm"] = options.placement_gap_mm
    if layout.minimum_clearance_mm is not None:
        layout_breakdown["minimum_pair_clearance_mm"] = (
            layout.minimum_clearance_mm
        )
    return PuzzleSolveResult(
        PuzzleSolveStatus.SOLVED,
        None,
        dimensions,
        placements,
        len(solutions),
        best_score,
        second_score,
        gap,
        layout_breakdown,
        texture_evidence is not None,
        "puzzle-geometry-v2+discrete-v1",
        "geometry-only",
        {},
        {"solve": elapsed},
        expansions,
        pruned,
        {
            "extra": layout.as_diagnostics(
                tuple(piece.piece_id for piece in pieces),
                options.placement_gap_mm,
            )
        },
    )
