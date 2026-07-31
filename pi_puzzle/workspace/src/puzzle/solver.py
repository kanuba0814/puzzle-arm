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
    center = (
        (target.min_x_mm + target.max_x_mm) / 2,
        (target.min_y_mm + target.max_y_mm) / 2,
    )
    plan = PlacementPlan(
        str(piece.piece_id),
        0,
        Pose2("table_mm", piece.center_table_mm, float(piece.yaw_deg)),
        Pose2("table_mm", center, float(piece.yaw_deg)),
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
        },
        False,
        "puzzle-geometry-v2",
        "geometry-only",
        {},
        {"solve": 0.0},
        0,
        0,
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


def _raster_quality(
    polygons: tuple[np.ndarray, ...], lower: np.ndarray, upper: np.ndarray
) -> tuple[float, float]:
    """1 mm 固定栅格检查矩形覆盖和真实片间重叠，避免凸包近似。"""
    step = 1.0
    x_values = np.arange(lower[0] + step / 2, upper[0], step)
    y_values = np.arange(lower[1] + step / 2, upper[1], step)
    total = max(1, len(x_values) * len(y_values))
    covered = 0
    overlap = 0
    for y_value in y_values:
        for x_value in x_values:
            count = sum(
                _point_in_polygon((float(x_value), float(y_value)), polygon)
                for polygon in polygons
            )
            covered += count > 0
            overlap += count > 1
    return 1.0 - covered / total, overlap / total


def _positive_overlap(first: np.ndarray, second: np.ndarray) -> bool:
    """只在两片 AABB 的交集采样，供搜索期快速拒绝。"""
    lower = np.maximum(first.min(axis=0), second.min(axis=0))
    upper = np.minimum(first.max(axis=0), second.max(axis=0))
    if np.any(upper - lower <= 1e-6):
        return False
    # 1 mm centre sampling: valid contact is a line and不会落入采样点。
    for y_value in np.arange(lower[1] + 0.5, upper[1], 1.0):
        for x_value in np.arange(lower[0] + 0.5, upper[0], 1.0):
            if _point_in_polygon(
                (float(x_value), float(y_value)), first
            ) and _point_in_polygon((float(x_value), float(y_value)), second):
                return True
    return False


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
) -> tuple[PlacementPlan, ...]:
    transformed = [
        _transform(_polygon(piece), *transforms[piece.piece_id]) for piece in pieces
    ]
    all_points = np.concatenate(transformed)
    shift = (
        np.array(
            [
                (target.min_x_mm + target.max_x_mm) / 2,
                (target.min_y_mm + target.max_y_mm) / 2,
            ]
        )
        - (all_points.min(axis=0) + all_points.max(axis=0)) / 2
    )
    plans: list[PlacementPlan] = []
    # Piece ID is the tie-breaker; geometry score makes this deterministic and easy to audit.
    for order, piece in enumerate(sorted(pieces, key=lambda item: item.piece_id)):
        rotation, translation = transforms[piece.piece_id]
        target_center = (
            rotation @ np.asarray(piece.center_table_mm) + translation + shift
        )
        plans.append(
            PlacementPlan(
                piece.piece_id,
                order,
                Pose2("table_mm", piece.center_table_mm, float(piece.yaw_deg)),
                Pose2(
                    "table_mm",
                    (float(target_center[0]), float(target_center[1])),
                    _yaw_from_rotation(rotation, float(piece.yaw_deg)),
                ),
                score,
                None,
                "not_run",
                max(0.0, 1.0 - score),
                int(piece.timestamp_ns),
                tuple(piece.equivalent_yaws_deg),
            )
        )
    return tuple(plans)


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
                        _positive_overlap(existing_polygon, candidate_polygon)
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

    search((pieces[0],), {pieces[0].piece_id: (np.eye(2), np.zeros(2))}, {})
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
    gap = None if second_score is None else second_score - best_score
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
    return PuzzleSolveResult(
        PuzzleSolveStatus.SOLVED,
        None,
        dimensions,
        _plans_for_assembly(pieces, transforms, target_region, best_score),
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
