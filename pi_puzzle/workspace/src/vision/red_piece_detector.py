# pyright: reportMissingImports=false

"""1280×720 UVC 画面中的红色碎片检测。只输出观察，不控制机械臂。"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

EXPECTED_FRAME_SIZE = (1280, 720)


def _finite_float(value: object, field: str) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须是有限数值") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} 必须是有限数值")
    return number


@dataclass(frozen=True)
class _Candidate:
    center_pixel: tuple[float, float]
    center_table: np.ndarray
    table_box: np.ndarray
    pixel_box: np.ndarray
    yaw_deg: float
    area: float


@dataclass(frozen=True)
class RedPieceObservation:
    piece_id: str
    timestamp_ns: int
    center_table_mm: tuple[float, float]
    pick_point_table_mm: tuple[float, float]
    polygon_table_mm: tuple[tuple[float, float], ...]
    yaw_deg: float
    confidence: float
    center_pixel: tuple[float, float]
    pixel_area: float

    def as_mapping(self) -> dict[str, object]:
        return {
            "piece_id": self.piece_id,
            "timestamp_ns": self.timestamp_ns,
            "frame": "table_mm",
            "polygon_table_mm": [list(point) for point in self.polygon_table_mm],
            "pick_point_table_mm": list(self.pick_point_table_mm),
            "center_table_mm": list(self.center_table_mm),
            "yaw_deg": self.yaw_deg,
            "confidence": self.confidence,
            "diagnostics": {
                "center_pixel": list(self.center_pixel),
                "pixel_area": self.pixel_area,
            },
        }


def load_homography(path: str | Path) -> np.ndarray:
    homography = np.load(Path(path))
    if homography.shape != (3, 3) or not np.isfinite(homography).all():
        raise ValueError("table homography 必须是有限的 3×3 矩阵")
    return homography.astype(np.float64)


def detect_red_pieces(
    frame: np.ndarray,
    homography: np.ndarray,
    *,
    minimum_pixel_area: float = 300.0,
) -> tuple[list[RedPieceObservation], np.ndarray, np.ndarray]:
    """返回按 table y 降序、x 升序排列的碎片、掩码和标注图。"""
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame 必须是 BGR 三通道图像")
    height, width = frame.shape[:2]
    if (width, height) != EXPECTED_FRAME_SIZE:
        raise ValueError(
            f"frame 必须是 {EXPECTED_FRAME_SIZE[0]}×{EXPECTED_FRAME_SIZE[1]}，"
            f"实际为 {width}×{height}"
        )
    if homography.shape != (3, 3):
        raise ValueError("homography 必须是 3×3")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([0, 80, 50], dtype=np.uint8),
        np.array([15, 255, 255], dtype=np.uint8),
    )
    mask |= cv2.inRange(
        hsv,
        np.array([165, 80, 50], dtype=np.uint8),
        np.array([179, 255, 255], dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[_Candidate] = []
    annotated = frame.copy()
    timestamp_ns = time.time_ns()
    for contour in contours:
        area = _finite_float(cv2.contourArea(contour), "contour area")
        if area < minimum_pixel_area:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        center_pixel = (
            _finite_float(moments["m10"] / moments["m00"], "center pixel x"),
            _finite_float(moments["m01"] / moments["m00"], "center pixel y"),
        )
        center_table = cv2.perspectiveTransform(
            np.array([[center_pixel]], dtype=np.float32), homography
        )[0, 0]
        if not (-5.0 <= center_table[0] <= 215.0):
            continue
        if not (-5.0 <= center_table[1] <= 302.0):
            continue

        pixel_box = cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32)
        table_box = cv2.perspectiveTransform(
            pixel_box.reshape(-1, 1, 2), homography
        ).reshape(-1, 2)
        table_rect = cv2.minAreaRect(table_box.astype(np.float32))
        table_width, table_height = table_rect[1]
        yaw_deg = _finite_float(table_rect[2], "piece yaw")
        if table_width < table_height:
            yaw_deg += 90.0
        yaw_deg = (yaw_deg + 90.0) % 180.0 - 90.0
        candidates.append(
            _Candidate(
                center_pixel=center_pixel,
                center_table=center_table,
                table_box=table_box,
                pixel_box=pixel_box,
                yaw_deg=yaw_deg,
                area=area,
            )
        )

    candidates.sort(
        key=lambda item: (
            -_finite_float(item.center_table[1], "sort table y"),
            _finite_float(item.center_table[0], "sort table x"),
        )
    )
    observations: list[RedPieceObservation] = []
    for index, item in enumerate(candidates):
        center_table = item.center_table
        center_pixel = item.center_pixel
        table_box = item.table_box
        pixel_box = item.pixel_box
        area = item.area
        confidence = min(1.0, max(0.0, area / 2500.0))
        observation = RedPieceObservation(
            piece_id=f"red-{index}",
            timestamp_ns=timestamp_ns,
            center_table_mm=(
                _finite_float(center_table[0], "center table x"),
                _finite_float(center_table[1], "center table y"),
            ),
            pick_point_table_mm=(
                _finite_float(center_table[0], "pick table x"),
                _finite_float(center_table[1], "pick table y"),
            ),
            polygon_table_mm=tuple(
                (
                    _finite_float(point[0], "polygon table x"),
                    _finite_float(point[1], "polygon table y"),
                )
                for point in table_box
            ),
            yaw_deg=item.yaw_deg,
            confidence=confidence,
            center_pixel=(
                _finite_float(center_pixel[0], "center pixel x"),
                _finite_float(center_pixel[1], "center pixel y"),
            ),
            pixel_area=area,
        )
        observations.append(observation)
        box_int = np.int32(np.round(pixel_box))
        cv2.drawContours(annotated, [box_int], -1, (0, 255, 0), 2)
        center_int = (round(center_pixel[0]), round(center_pixel[1]))
        cv2.circle(annotated, center_int, 6, (255, 255, 0), -1)
        cv2.putText(
            annotated,
            f"{index}:({center_table[0]:.1f},{center_table[1]:.1f})",
            (center_int[0] + 8, center_int[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 0),
            2,
        )
    return observations, mask, annotated
