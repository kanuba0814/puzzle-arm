# pyright: reportMissingImports=false

"""A4 工作台标定到校正帧的桥接适配器。

该模块把现有的 `table_homography.npy` 单应矩阵（原始 1280×720 像素 → table_mm）
转换为 `observe_pieces` 所需的 `RectifiedFrame`（已校正俯视图 + px_per_mm）。

坐标约定：
- table_mm 原点在 A4 纸左下角，x 向右（0-210 mm），y 向上（0-297 mm）。
- 校正图像原点在左下，行顺序为从下到上（与 OpenCV 默认相反），使像素坐标与 table 坐标同向。
"""

from __future__ import annotations

import hashlib
import time
from pathlib import Path

import cv2
import numpy as np

from ._numeric import finite_float, positive_float, rounded_int
from .generic_piece_observer import RectifiedFrame


def load_a4_homography(path: Path | str) -> np.ndarray:
    """加载现有 3×3 单应矩阵（原始像素 → table_mm）。"""
    homography = np.load(path)
    if homography.shape != (3, 3):
        raise ValueError(f"单应矩阵形状必须为 (3,3)，实际为 {homography.shape}")
    if not np.isfinite(homography).all():
        raise ValueError("单应矩阵必须全部为有限值")
    matrix = homography.astype(np.float64)
    if abs(finite_float(np.linalg.det(matrix), "homography determinant")) < 1e-12:
        raise ValueError("单应矩阵奇异，无法求逆用于校正")
    return matrix


def _positive_dimension(value: float, field: str) -> float:
    """校验正有限尺寸；非法输入显式失败而不是产生退化图像。"""
    return positive_float(value, field)


def rectify_a4_frame(
    raw_frame: np.ndarray,
    homography: np.ndarray,
    *,
    px_per_mm: float = 3.0,
    a4_width_mm: float = 210.0,
    a4_height_mm: float = 297.0,
    source_image_id: str | None = None,
    homography_id: str = "table_homography_gui_20260731",
    camera_intrinsics_id: str | None = None,
) -> RectifiedFrame:
    """把原始相机帧通过单应矩阵校正为俯视 table_mm 图像。

    Args:
        raw_frame: 原始相机图像，shape (H, W, 3) BGR。
        homography: 3×3 单应矩阵，映射 [px, py, 1] → [table_x_mm, table_y_mm, 1]。
        px_per_mm: 校正图像的像素/毫米比例。
        a4_width_mm: A4 宽度（x 方向）。
        a4_height_mm: A4 高度（y 方向）。
        source_image_id: 原图标识。
        homography_id: 单应矩阵标识。
        camera_intrinsics_id: 相机内参标识（可选）。

    Returns:
        RectifiedFrame，其中图像原点在左下，y 向上，像素与 table_mm 对齐。
    """
    if raw_frame.ndim != 3 or raw_frame.shape[2] != 3:
        raise ValueError(f"原始帧必须为 BGR 三通道图像，实际形状 {raw_frame.shape}")
    if homography.shape != (3, 3) or not np.isfinite(homography).all():
        raise ValueError("homography 必须是有限的 3×3 矩阵")

    scale = _positive_dimension(px_per_mm, "px_per_mm")
    width_mm = _positive_dimension(a4_width_mm, "a4_width_mm")
    height_mm = _positive_dimension(a4_height_mm, "a4_height_mm")

    # 校正图像尺寸：A4 范围 × px_per_mm
    rect_width_px = rounded_int(width_mm * scale, "rectified width px")
    rect_height_px = rounded_int(height_mm * scale, "rectified height px")
    if rect_width_px < 2 or rect_height_px < 2:
        raise ValueError("校正图像尺寸过小，请提高 px_per_mm 或检查 A4 尺寸")

    # 构造从校正像素到 table_mm 的映射：
    # table_x = rect_u / px_per_mm
    # table_y = (rect_height_px - 1 - rect_v) / px_per_mm
    # 其中 rect_v=0 对应图像顶部，rect_v=rect_height_px-1 对应底部（table_y=0）。
    #
    # 反向：从 table_mm 到校正像素
    # rect_u = table_x * px_per_mm
    # rect_v = rect_height_px - 1 - table_y * px_per_mm
    #
    # 组合 homography^{-1}：
    # [raw_px, raw_py, 1] = H^{-1} @ [table_x, table_y, 1]
    #                     = H^{-1} @ [rect_u/ppm, (H-1-rect_v)/ppm, 1]
    #
    # 定义 M = H^{-1} @ [[1/ppm, 0, 0], [0, 1/ppm, (H-1)/ppm], [0, 0, 1]]
    #   即从校正像素 [rect_u, rect_v, 1] 直接映射到原始像素。

    homography_inv = np.linalg.inv(homography.astype(np.float64))
    scale_and_flip = np.array(
        [
            [1.0 / scale, 0.0, 0.0],
            [0.0, -1.0 / scale, (rect_height_px - 1) / scale],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    warp_matrix = homography_inv @ scale_and_flip

    rectified = cv2.warpPerspective(
        raw_frame,
        warp_matrix,
        (rect_width_px, rect_height_px),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )

    # 计算原图 SHA256
    raw_bytes = raw_frame.tobytes()
    source_sha256 = hashlib.sha256(raw_bytes).hexdigest()

    # 计算单应矩阵 SHA256
    homography_bytes = homography.tobytes()
    homography_sha256 = hashlib.sha256(homography_bytes).hexdigest()

    timestamp_ns = time.monotonic_ns()

    return RectifiedFrame(
        image=rectified,
        px_per_mm=scale,
        table_mm_size=(width_mm, height_mm),
        source_image_sha256=source_sha256,
        source_image_id=source_image_id or f"raw_{timestamp_ns}",
        timestamp_ns=timestamp_ns,
        homography_id=homography_id,
        homography_sha256=homography_sha256,
        camera_intrinsics_id=camera_intrinsics_id,
    )
