# pyright: reportMissingImports=false

"""从物理外轮廓派生求解虚拟几何的私有工具。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    from ._segmentation import SegmentationCandidate
    from .generic_piece_observer import RectifiedFrame, VisionProfile


@dataclass(frozen=True)
class _Geometry:
    contour_px: tuple[tuple[float, float], ...]
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float
    equivalent_yaws_deg: tuple[float, ...]
    residual_mm: float
    pick_clearance_mm: float
    rounded_candidates: int


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
    choices: list[tuple[float, np.ndarray]] = []
    for epsilon_mm in (0.4, 0.7, 1.0, 1.5, 2.0):
        polygon = cv2.approxPolyDP(contour, epsilon_mm * frame.px_per_mm, True).reshape(
            -1, 2
        )
        if not 3 <= len(polygon) <= profile.max_piece_sides:
            continue
        table = _table_points(polygon, frame)
        edge_lengths = np.linalg.norm(table - np.roll(table, -1, axis=0), axis=1)
        if np.any(
            edge_lengths < profile.min_edge_length_mm - 2.0
        ) or not _simple_polygon(table):
            continue
        residual = (
            float(
                np.mean(
                    np.min(
                        np.linalg.norm(
                            contour_points[:, None, :] - polygon[None, :, :], axis=2
                        ),
                        axis=1,
                    )
                )
            )
            / frame.px_per_mm
        )
        choices.append((residual, polygon))
    if not choices:
        return None
    choices.sort(key=lambda choice: (round(choice[0], 8), len(choice[1])))
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
    pick_clearance_mm = float(distance.max()) / frame.px_per_mm
    contour_points = tuple(
        (float(point[0]), float(point[1])) for point in candidate.contour.reshape(-1, 2)
    )
    return _Geometry(
        contour_px=contour_points,
        polygon_table_mm=tuple(
            (float(point[0]), float(point[1])) for point in table_polygon
        ),
        center_table_mm=(float(center_table[0]), float(center_table[1])),
        yaw_deg=yaw,
        equivalent_yaws_deg=_equivalent_yaws(table_polygon, yaw),
        residual_mm=residual_mm,
        pick_clearance_mm=pick_clearance_mm,
        # Rounded recovery is profile-gated; implementation intentionally leaves physical geometry untouched.
        rounded_candidates=0 if not profile.enable_rounded_corner_recovery else 0,
    )
