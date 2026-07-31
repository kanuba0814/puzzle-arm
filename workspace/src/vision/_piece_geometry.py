# pyright: reportMissingImports=false

"""从物理外轮廓派生求解虚拟几何的私有工具。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2
import numpy as np

from ._rounded_corner import recover_virtual_corners

if TYPE_CHECKING:
    from ._segmentation import SegmentationCandidate
    from .generic_piece_observer import RectifiedFrame, VisionProfile


@dataclass(frozen=True)
class _Geometry:
    contour_px: tuple[tuple[float, float], ...]
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    pick_point_table_mm: tuple[float, float]
    yaw_deg: float
    equivalent_yaws_deg: tuple[float, ...]
    residual_mm: float
    pick_clearance_mm: float
    rounded_candidates: int
    geometry_source: str


def _signed_area(points: np.ndarray) -> float:
    return (
        float(
            np.dot(points[:, 0], np.roll(points[:, 1], -1))
            - np.dot(points[:, 1], np.roll(points[:, 0], -1))
        )
        / 2
    )


def _simple_polygon(points: np.ndarray) -> bool:
    # cv2 contour approximations cannot normally self-intersect; protect contract explicitly.
    if len(points) < 3 or abs(_signed_area(points)) < 1e-6:
        return False
    for index in range(len(points)):
        a, b = points[index], points[(index + 1) % len(points)]
        for other in range(index + 2, len(points)):
            if (other + 1) % len(points) in (index, (index + 1) % len(points)):
                continue
            c, d = points[other], points[(other + 1) % len(points)]
            if _segments_intersect(a, b, c, d):
                return False
    return True


def _segments_intersect(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> bool:
    def orient(first: np.ndarray, second: np.ndarray, third: np.ndarray) -> float:
        first_vector = second - first
        second_vector = third - first
        return float(
            first_vector[0] * second_vector[1] - first_vector[1] * second_vector[0]
        )

    return (
        orient(a, b, c) * orient(a, d, c) < 0 and orient(c, d, a) * orient(c, d, b) < 0
    )


def _table_points(points_px: np.ndarray, frame: RectifiedFrame) -> np.ndarray:
    result = points_px.astype(np.float64).copy()
    result[:, 0] /= frame.px_per_mm
    result[:, 1] = (frame.image.shape[0] - 1 - result[:, 1]) / frame.px_per_mm
    return result


def _candidate_polygon(
    contour: np.ndarray, frame: RectifiedFrame, profile: VisionProfile
) -> tuple[np.ndarray, float] | None:
    contour_points = contour.reshape(-1, 2).astype(np.float64)
    # 直线切割产生的碎片必然是凸多边形，因此先取凸包：实机掩码会因
    # 相邻碎片阴影或折痕出现假凹口（实测：一个四边形因此拟合出 7 顶点
    # 而被整片拒绝）。凹形碎片（如拼图榫口）需单独 profile 开关后再保留原序。
    boundary = cv2.convexHull(contour)
    peri = cv2.arcLength(boundary, True)
    choices: list[tuple[float, np.ndarray]] = []
    # 使用相对 epsilon（周长的百分比），适应不同尺寸的碎片。
    # 0.5% 对 100mm 周长 = 0.5mm，对 200mm = 1.0mm。
    # 如果残差仍高，回退到更粗的 2.5% 简化。
    for epsilon_rel in (0.005, 0.01, 0.015, 0.025):
        polygon = cv2.approxPolyDP(boundary, epsilon_rel * peri, True).reshape(-1, 2)
        if not 3 <= len(polygon) <= profile.max_piece_sides:
            continue
        table = _table_points(polygon, frame)
        edge_lengths = np.linalg.norm(table - np.roll(table, -1, axis=0), axis=1)
        min_allowed_edge_mm = max(6.0, profile.min_edge_length_mm - 12.0)
        if np.any(edge_lengths < min_allowed_edge_mm) or not _simple_polygon(table):
            continue
        # 残差必须测轮廓点到多边形【边】的距离。早期实现测的是到【顶点】
        # 的距离，使任何长边的中点都被计为半边长误差（例如 60 mm 边产生
        # ~15 mm 均值），拟合良好时仍输出巨大残差，会误导候选排序和诊断。
        polygon_i32 = polygon.astype(np.int32).reshape(-1, 1, 2)
        residual = (
            float(
                np.mean(
                    [
                        abs(
                            cv2.pointPolygonTest(
                                polygon_i32, (float(point[0]), float(point[1])), True
                            )
                        )
                        for point in contour_points
                    ]
                )
            )
            / frame.px_per_mm
        )
        choices.append((residual, polygon))
    if not choices:
        return None
    choices.sort(key=lambda choice: (round(choice[0], 8), len(choice[1])))
    best_residual = choices[0][0]
    # 顶点数偏好：掩码栅格化会让多余顶点把残差压低零点几毫米（实测四边形
    # 被拟合成 5 边，多出一条假切割边误导求解器）。官方最短切割边 20 mm，
    # 删掉一个真实顶点至少造成毫米级残差，因此在 0.2 mm 容差内选边数最少者。
    for residual, polygon in sorted(
        choices, key=lambda choice: (len(choice[1]), round(choice[0], 8))
    ):
        if residual <= best_residual + 0.2:
            return polygon, residual
    return choices[0][1], choices[0][0]


def _equivalent_yaws(polygon: np.ndarray, yaw_deg: float) -> tuple[float, ...]:
    lengths = np.linalg.norm(polygon - np.roll(polygon, -1, axis=0), axis=1)
    if len(polygon) == 4 and np.allclose(lengths, lengths[0], rtol=0.05, atol=1.0):
        return tuple(
            (yaw_deg + delta + 180.0) % 360.0 - 180.0
            for delta in (0.0, 90.0, 180.0, 270.0)
        )
    if len(polygon) == 4 and np.allclose(lengths[0], lengths[2], rtol=0.05, atol=1.0):
        return (yaw_deg, (yaw_deg + 180.0) % 360.0 - 180.0)
    return ()


def build_piece_geometry(
    frame: RectifiedFrame, profile: VisionProfile, candidate: SegmentationCandidate
) -> _Geometry | None:
    selected = _candidate_polygon(candidate.contour, frame, profile)
    if selected is None:
        return None
    polygon_px, residual_mm = selected
    table_polygon = _table_points(polygon_px, frame)
    moments = cv2.moments(candidate.contour)
    if moments["m00"] == 0:
        return None
    center_px = np.array(
        [[moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]]]
    )
    center_table = _table_points(center_px, frame)[0]
    rectangle = cv2.minAreaRect(polygon_px.astype(np.float32))
    yaw = rectangle[2] + (90.0 if rectangle[1][0] < rectangle[1][1] else 0.0)
    yaw = (float(yaw) + 180.0) % 360.0 - 180.0
    distance = cv2.distanceTransform(candidate.mask, cv2.DIST_L2, 5)
    # Distance transform peak is the largest inscribed clearance, which is a safer
    # magnet target than the centroid on concave or thin fragments.
    _, max_clearance_px, _, peak_px = cv2.minMaxLoc(distance)
    pick_clearance_mm = float(max_clearance_px) / frame.px_per_mm
    pick_table = _table_points(
        np.array([[float(peak_px[0]), float(peak_px[1])]]), frame
    )[0]
    contour_points = tuple(
        (float(point[0]), float(point[1])) for point in candidate.contour.reshape(-1, 2)
    )
    # Rounded-corner virtualization is profile-gated; the white metal profile keeps
    # measured geometry untouched so real cut vertices can never be absorbed.
    geometry_source = "fitted_polygon"
    rounded_candidates = 0
    solver_polygon = table_polygon
    if profile.enable_rounded_corner_recovery:
        recovered = recover_virtual_corners(
            table_polygon,
            max_chord_mm=min(8.0, profile.min_edge_length_mm * 0.4),
            max_sides=profile.max_piece_sides,
        )
        if recovered.replaced_count:
            candidate_polygon = np.asarray(recovered.polygon_mm, dtype=float)
            if len(candidate_polygon) >= 3 and _simple_polygon(candidate_polygon):
                solver_polygon = candidate_polygon
                rounded_candidates = recovered.replaced_count
                geometry_source = "virtual_corner_polygon"
    return _Geometry(
        contour_px=contour_points,
        polygon_table_mm=tuple(
            (float(point[0]), float(point[1])) for point in solver_polygon
        ),
        center_table_mm=(float(center_table[0]), float(center_table[1])),
        pick_point_table_mm=(float(pick_table[0]), float(pick_table[1])),
        yaw_deg=yaw,
        equivalent_yaws_deg=_equivalent_yaws(solver_polygon, yaw),
        residual_mm=residual_mm,
        pick_clearance_mm=pick_clearance_mm,
        rounded_candidates=rounded_candidates,
        geometry_source=geometry_source,
    )
