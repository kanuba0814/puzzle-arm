"""视觉观察模块。"""

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

__all__ = [
    "MeasuredContour",
    "PieceObservation",
    "RectifiedFrame",
    "VisionDiagnostics",
    "VisionProfile",
    "VisionResult",
    "observe_pieces",
    "EXPECTED_FRAME_SIZE",
    "RedPieceObservation",
    "detect_red_pieces",
    "load_homography",
]
