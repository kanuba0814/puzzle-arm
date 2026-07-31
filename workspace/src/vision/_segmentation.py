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


# 彩色碎片的 HSV 门限。竞赛材料是白色；彩色仅用于开发期在黑色 A4 上做几何验证。
# 每个条目是 (hue_low, hue_high) 列表，红色跨 0° 需要两段。
_CHROMATIC_HUE_BANDS: dict[str, tuple[tuple[int, int], ...]] = {
    "red": ((0, 12), (168, 179)),
    "green": ((35, 90),),
    "blue": ((95, 130),),
}


def _chromatic_masks(
    roi_bgr: np.ndarray, material: str, morphology_kernel: np.ndarray
) -> list[np.ndarray]:
    """对彩色碎片生成多饱和度阈值掩码，保持与亮度路径相同的投票语义。

    竞赛正式材料为白纸；本路径只服务开发期彩色几何验证，因此门限保持宽松，
    真正的接受判据仍由多边形契约、面积和稳定性投票决定。
    """
    bands = _CHROMATIC_HUE_BANDS[material]
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    masks: list[np.ndarray] = []
    # 彩色纸的色相掩码边缘比白色亮度掩码噪声大：先用较大椭圆核闭运算填掉锯齿，
    # 再开运算去毛刺，否则轮廓简化会把掩码锯齿当成 3~10 mm 的假切割边。
    smoothing_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (_odd(morphology_kernel.shape[0] * 3), _odd(morphology_kernel.shape[0] * 3)),
    )
    # 相邻饱和度门限替代单一阈值，使阴影和高光边缘不会一次性决定分割结果。
    for saturation_minimum in (70, 90, 110):
        combined = np.zeros(hsv.shape[:2], np.uint8)
        for hue_low, hue_high in bands:
            band = cv2.inRange(
                hsv,
                np.array([hue_low, saturation_minimum, 50], np.uint8),
                np.array([hue_high, 255, 255], np.uint8),
            )
            combined = cv2.bitwise_or(combined, band)
        # 先闭后开：闭合填补掩码内部空洞和边缘凹陷，开运算移除孤立噪点。
        closed = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, smoothing_kernel)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, smoothing_kernel)
        # 中值滤波进一步平滑阶梯状边界，保留直边和真实角点。
        masks.append(cv2.medianBlur(opened, _odd(morphology_kernel.shape[0] * 2)))
    return masks


def _luminance_masks(
    roi_gray: np.ndarray, px_per_mm: float, morphology_kernel: np.ndarray
) -> list[np.ndarray]:
    """白色/印刷牌基的背景残差 + 曝光自适应绝对亮度掩码。

    绝对亮度门限不再硬编码：黑色 A4 在实机画面中有明显纵向渐变和光泽，且
    广角相机对整体偏暗场景会拉低自动曝光（实测合成 0.6× 曝光时白色碎片
    仅 ~135 灰度，固定 140 阈值整帧丢失）。参考公开实现的思路，用每帧
    ROI 中值估计黑纸亮度、高分位估计碎片亮度，按对比度取相对阈值。
    """
    background_kernel = _odd(px_per_mm * 15.0)
    background = cv2.morphologyEx(
        roi_gray,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (background_kernel, background_kernel)
        ),
    )
    difference = cv2.subtract(roi_gray, background)
    # 相邻碎片间的窄黑缝经镜头模糊后可能越过亮度阈值，把两片连成同一
    # 连通域（实测：大三角形与四边形粘连，凸包后多出一条 12.5 mm 短边）。
    # 因此断桥用的 OPEN 必须明显大于回填用的 CLOSE；两者同尺时桥会被接回。
    break_bridge_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (_odd(px_per_mm * 1.5), _odd(px_per_mm * 1.5))
    )
    # 黑纸占 ROI 绝大多数像素，中值即纸面亮度；99.5 分位落在碎片内部。
    paper_level = float(np.median(roi_gray))
    bright_level = float(np.percentile(roi_gray, 99.5))
    contrast = bright_level - paper_level
    # 白纸对黑纸的实际对比远大于黑纸自身渐变（渐变 ≲ 40 灰度）。对比不足
    # 说明 ROI 内没有可信白色碎片，此时禁用绝对亮度证据，只留残差路径，
    # 避免把黑纸渐变亮端当成前景。
    minimum_contrast = 45.0
    masks: list[np.ndarray] = []
    # White/card surfaces become positive residuals. Three nearby thresholds avoid fragile one-shot segmentation.
    for residual_threshold, contrast_fraction in ((12, 0.35), (20, 0.50), (28, 0.65)):
        residual_mask = cv2.threshold(
            difference, residual_threshold, 255, cv2.THRESH_BINARY
        )[1]
        # Absolute bright evidence complements background residual where a large white card
        # survives opening unchanged; the threshold tracks per-frame paper/piece levels.
        if contrast >= minimum_contrast:
            absolute_threshold = min(250.0, paper_level + contrast_fraction * contrast)
            bright_mask = cv2.threshold(
                roi_gray, absolute_threshold, 255, cv2.THRESH_BINARY
            )[1]
        else:
            bright_mask = np.zeros_like(roi_gray)
        mask = cv2.bitwise_or(residual_mask, bright_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, break_bridge_kernel)
        masks.append(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, morphology_kernel))
    return masks


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
    # 校正图 row 0 对应最大 table y；A4 上半源区因此位于图像上半。
    # 横向/纵向各内缩一段：实机校正图包含黑纸之外的 A4 白边和桌面，
    # 白边与靠边碎片会连成同一轮廓（实测：2732 mm² 大三角形与顶/左白边
    # 粘连，包围盒填充率仅 19%），使该碎片无法被单独拟合。
    roi_inset_px = int(round(profile.roi_inset_mm * frame.px_per_mm))
    roi_start = roi_inset_px
    # 下边界要避开分界线（题面允许实线宽 ≤ 5 mm）：正好取 height//2 会把
    # 线本身当成前景。但该内缩不能沿用 20 mm 白边余量：靠分界线摆放的合法
    # 碎片会被静默裁剪出一条假边，因此使用独立的 roi_divider_inset_mm。
    divider_inset_px = int(round(profile.roi_divider_inset_mm * frame.px_per_mm))
    roi_stop = max(roi_start + 1, height // 2 - divider_inset_px)
    roi_x0 = roi_inset_px
    roi_x1 = max(roi_x0 + 1, width - roi_inset_px)
    roi = gray[roi_start:roi_stop, roi_x0:roi_x1]
    morphology_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (_odd(frame.px_per_mm), _odd(frame.px_per_mm))
    )

    material = profile.material.strip().lower()
    chromatic = material in _CHROMATIC_HUE_BANDS
    if chromatic:
        if image.ndim != 3:
            # 彩色 profile 需要 BGR 输入；灰度帧无法判定色相。
            return [], "VISION_SURFACE_MISMATCH"
        masks = _chromatic_masks(
            image[roi_start:roi_stop, roi_x0:roi_x1], material, morphology_kernel
        )
    else:
        masks = _luminance_masks(roi, frame.px_per_mm, morphology_kernel)

    min_area = (profile.min_edge_length_mm * frame.px_per_mm) ** 2 * 0.35
    contours, _ = cv2.findContours(masks[1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    candidates: list[SegmentationCandidate] = []
    roi_height, roi_width = roi.shape if len(roi.shape) == 2 else roi.shape[:2]
    max_piece_area = (120.0 * 90.0 * frame.px_per_mm**2) * 1.5
    min_matches = max(1, len(masks) // 2)  # 至少一半掩码同意
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        # 纸张边条无论横竖都不可能是合法碎片：实测右侧白边为 5 mm × 117 mm
        # （16:350 px），仅过滤横条会放过它并占掉一个 max_pieces 名额。
        x, y, component_width, component_height = cv2.boundingRect(contour)
        thin_px = frame.px_per_mm * 5
        long_px = frame.px_per_mm * 80
        if (
            min(component_width, component_height) <= thin_px
            and max(component_width, component_height) > long_px
        ):
            continue
        if area > max_piece_area:
            return [], "VISION_MERGED_BLOBS"
        matches, _ = _component_matches(masks, contour, min_area)
        if matches < min_matches:
            continue
        touches = (
            x <= 0
            or y <= 0
            or x + component_width >= roi_width
            or y + component_height >= roi_height
        )
        # 不能一律拒绝触边轮廓：靠 A4 边摆放的合法碎片内缩后恰好落在 ROI
        # 边界上（实测：左下大三角形因此整片丢失）。只拒绝包围盒几乎跨整个
        # ROI 的“纸框”轮廓；真正的白边粘连已由 roi_inset_mm 内缩切除。
        spans_sheet_frame = (
            component_width >= roi_width * 0.9 and component_height >= roi_height * 0.9
        )
        if spans_sheet_frame:
            continue
        full_contour = contour.copy()
        full_contour[:, 0, 0] += roi_x0
        full_contour[:, 0, 1] += roi_start
        full_mask = np.zeros_like(gray)
        cv2.drawContours(
            full_mask[roi_start:roi_stop, roi_x0:roi_x1],
            [contour],
            -1,
            255,
            thickness=cv2.FILLED,
        )
        candidates.append(
            SegmentationCandidate(
                contour=full_contour,
                mask=full_mask,
                stability=matches / len(masks),
                matches=matches,
                touches_frame_border=touches,
            )
        )
    if len(candidates) > profile.max_pieces:
        # 竞赛明确最多 4 片；超出时保留面积最大的 max_pieces 个，丢弃小碎屑/阴影伪影。
        candidates.sort(key=lambda c: cv2.contourArea(c.contour), reverse=True)
        candidates = candidates[: profile.max_pieces]
    if not candidates:
        # 彩色路径缺少目标色相，或亮度路径面对暗背/金属背面：都拒绝而不是猜测。
        if chromatic:
            return [], "VISION_SURFACE_MISMATCH"
        # Dark/metal backs are expected to yield no stable bright components; reject instead of guessing orientation.
        if float(np.mean(roi)) < 90.0:
            return [], "VISION_SURFACE_MISMATCH"
    return candidates, None
