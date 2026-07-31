"""把旧红色检测观察迁移到通用严格契约，而不改变检测器行为。"""

from __future__ import annotations

import math
import time

from .generic_piece_observer import (
    MeasuredContour,
    PieceObservation,
    VisionDiagnostics,
)
from .red_piece_detector import RedPieceObservation


def red_observation_to_piece_observation(
    observation: RedPieceObservation, *, source_image_id: str = "legacy-red"
) -> PieceObservation:
    """以 diagnostics 中稳定的中心/面积重建最小矩形，不要求旧 dataclass 新字段。"""
    # RedPieceObservation 的公开构造参数长期只包含 center_pixel、面积和 table 多边形。
    # adapter 不向该旧类型索要 pixel_box，避免为迁移而破坏调用方构造契约。
    half_side = math.sqrt(max(0.0, observation.pixel_area)) / 2.0
    center_x, center_y = observation.center_pixel
    pixel_contour = (
        (center_x - half_side, center_y - half_side),
        (center_x + half_side, center_y - half_side),
        (center_x + half_side, center_y + half_side),
        (center_x - half_side, center_y + half_side),
    )
    return PieceObservation(
        piece_id=observation.piece_id,
        timestamp_ns=time.monotonic_ns(),
        source_image_id=source_image_id,
        measured_contour=MeasuredContour("image_px", pixel_contour),
        polygon_table_mm=observation.polygon_table_mm,
        pick_point_table_mm=observation.pick_point_table_mm,
        pick_clearance_mm=None,
        center_table_mm=observation.center_table_mm,
        yaw_deg=(observation.yaw_deg + 180.0) % 360.0 - 180.0,
        equivalent_yaws_deg=(),
        confidence=observation.confidence,
        diagnostics=VisionDiagnostics(
            segmentation_stability=observation.confidence,
            polygon_fit_residual_mm=0.0,
            pick_clearance_ok=None,
            rounded_corner_candidates=0,
            geometry_source="legacy_min_area_rect",
            touches_frame_border=False,
            extra={
                "pixel_area": observation.pixel_area,
                "center_pixel": list(observation.center_pixel),
                "physical_contour": "area-derived-square",
            },
        ),
    )
