# pyright: reportMissingImports=false

"""通用白色/印刷碎片的私有尺度感知分割。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    from .generic_piece_observer import RectifiedFrame, VisionProfile


@dataclass(frozen=True)
class SegmentationCandidate:
    contour: np.ndarray
    mask: np.ndarray
    stability: float
    matches: int
    touches_frame_border: bool


def _odd(value: float) -> int:
    return max(3, int(round(value)) | 1)


def _component_matches(
    masks: list[np.ndarray], reference: np.ndarray, minimum_area: float
) -> tuple[int, list[float]]:
    reference_area = cv2.contourArea(reference)
    matches = 0
    areas: list[float] = []
    center = cv2.moments(reference)
    if center["m00"] == 0:
        return 0, areas
    point = (int(center["m10"] / center["m00"]), int(center["m01"] / center["m00"]))
    for mask in masks:
        labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        label = (
            labels[point[1], point[0]]
            if 0 <= point[1] < labels.shape[0] and 0 <= point[0] < labels.shape[1]
            else 0
        )
        if label and stats[label, cv2.CC_STAT_AREA] >= minimum_area:
            area = float(stats[label, cv2.CC_STAT_AREA])
            if reference_area and abs(area - reference_area) / reference_area <= 0.20:
                matches += 1
                areas.append(area)
    return matches, areas


def segment_stable_pieces(
    frame: RectifiedFrame, profile: VisionProfile
) -> tuple[list[SegmentationCandidate], str | None]:
    """多阈值存在性投票。输入是 rectified 图，ROI 为 A4 上半源区。"""
    image = frame.image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image.copy()
    height, width = gray.shape[:2]
    # 图像 y 向下，table 原点在左下；源区对应图像下半。
    roi_start = height // 2
    roi = gray[roi_start:, :]
    background_kernel = _odd(frame.px_per_mm * 15.0)
    background = cv2.morphologyEx(
        roi,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (background_kernel, background_kernel)
        ),
    )
    difference = cv2.subtract(roi, background)
    # White/card surfaces become positive residuals. Three nearby thresholds avoid fragile one-shot segmentation.
    thresholds = (12, 20, 28)
    morphology_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (_odd(frame.px_per_mm), _odd(frame.px_per_mm))
    )
    masks: list[np.ndarray] = []
    for threshold in thresholds:
        residual_mask = cv2.threshold(difference, threshold, 255, cv2.THRESH_BINARY)[1]
        # Absolute bright evidence complements background residual where a large white card
        # survives opening unchanged. Threshold remains deliberately conservative for dark worktops.
        bright_mask = cv2.threshold(roi, 140 + threshold // 4, 255, cv2.THRESH_BINARY)[
            1
        ]
        mask = cv2.bitwise_or(residual_mask, bright_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, morphology_kernel)
        masks.append(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morphology_kernel))

    min_area = (profile.min_edge_length_mm * frame.px_per_mm) ** 2 * 0.35
    contours, _ = cv2.findContours(masks[1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    candidates: list[SegmentationCandidate] = []
    roi_height, roi_width = roi.shape
    max_piece_area = (120.0 * 90.0 * frame.px_per_mm**2) * 1.5
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        # Thin horizontal A4 separator cannot be a legal fragment.
        x, y, component_width, component_height = cv2.boundingRect(contour)
        if (
            component_height <= frame.px_per_mm * 5
            and component_width > frame.px_per_mm * 80
        ):
            continue
        if area > max_piece_area:
            return [], "VISION_MERGED_BLOBS"
        matches, _ = _component_matches(masks, contour, min_area)
        if matches < 2:
            continue
        touches = (
            x <= 0
            or y <= 0
            or x + component_width >= roi_width
            or y + component_height >= roi_height
        )
        full_contour = contour.copy()
        full_contour[:, 0, 1] += roi_start
        full_mask = np.zeros_like(gray)
        cv2.drawContours(
            full_mask[roi_start:, :], [contour], -1, 255, thickness=cv2.FILLED
        )
        candidates.append(
            SegmentationCandidate(
                contour=full_contour,
                mask=full_mask,
                stability=matches / len(thresholds),
                matches=matches,
                touches_frame_border=touches,
            )
        )
    if len(candidates) > profile.max_pieces:
        return [], "VISION_MERGED_BLOBS"
    # Dark/metal backs are expected to yield no stable bright components; reject instead of guessing orientation.
    if not candidates and float(np.mean(roi)) < 90.0:
        return [], "VISION_SURFACE_MISMATCH"
    return candidates, None
