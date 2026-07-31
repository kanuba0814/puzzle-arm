"""视觉观察模块。

公共接口分两层：
- 通用碎片观察：`observe_pieces` + `RectifiedFrame` + `VisionProfile`；
- A4 标定桥接：`load_a4_homography` + `rectify_a4_frame`，把现有单应矩阵
  和原始相机帧转换为 `RectifiedFrame`。

`detect_red_pieces` 是已冻结的 legacy 确定性路径，保持兼容直到迁移审查完成。
"""

from .generic_piece_observer import (
    MeasuredContour,
    PieceObservation,
    RectifiedFrame,
    VisionDiagnostics,
    VisionProfile,
    VisionResult,
    observe_pieces,
)
from .red_piece_detector import (
    EXPECTED_FRAME_SIZE,
    RedPieceObservation,
    detect_red_pieces,
    load_homography,
)
from .table_rectifier import load_a4_homography, rectify_a4_frame

__all__ = [
    "EXPECTED_FRAME_SIZE",
    "MeasuredContour",
    "PieceObservation",
    "RectifiedFrame",
    "RedPieceObservation",
    "VisionDiagnostics",
    "VisionProfile",
    "VisionResult",
    "detect_red_pieces",
    "load_a4_homography",
    "load_homography",
    "observe_pieces",
    "rectify_a4_frame",
]
