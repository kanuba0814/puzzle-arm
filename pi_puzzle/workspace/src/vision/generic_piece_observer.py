"""通用碎片观察公共接口。

此模块只产生可审计的视觉观察；分割细节保留在私有模块中，绝不驱动机械臂。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import numpy as np

from ._piece_geometry import build_piece_geometry
from ._segmentation import segment_stable_pieces


@dataclass(frozen=True)
class RectifiedFrame:
    """已校正为俯视图的帧；table_mm 原点为图像左下。"""

    image: np.ndarray
    px_per_mm: float
    table_mm_size: tuple[float, float]
    source_image_sha256: str
    source_image_id: str
    timestamp_ns: int
    homography_id: str
    homography_sha256: str
    camera_intrinsics_id: str | None


@dataclass(frozen=True)
class VisionProfile:
    """材料 profile；所有空间参数仍在毫米域。"""

    profile_id: str
    material: str
    enable_rounded_corner_recovery: bool
    min_edge_length_mm: float = 20.0
    max_piece_sides: int = 5
    max_pieces: int = 4


@dataclass(frozen=True)
class MeasuredContour:
    frame: str
    points: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class VisionDiagnostics:
    segmentation_stability: float
    polygon_fit_residual_mm: float
    pick_clearance_ok: bool | None
    rounded_corner_candidates: int
    geometry_source: str
    touches_frame_border: bool
    error_code: str | None = None
    extra: dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class PieceObservation:
    piece_id: str
    timestamp_ns: int
    source_image_id: str
    measured_contour: MeasuredContour
    polygon_table_mm: tuple[tuple[float, float], ...]
    pick_point_table_mm: tuple[float, float] | None
    pick_clearance_mm: float | None
    center_table_mm: tuple[float, float]
    yaw_deg: float
    equivalent_yaws_deg: tuple[float, ...]
    confidence: float
    diagnostics: VisionDiagnostics

    def as_mapping(self) -> dict[str, object]:
        """返回严格 PieceObservation schema 可序列化映射。"""
        return {
            "piece_id": self.piece_id,
            "timestamp_ns": self.timestamp_ns,
            "source_image_id": self.source_image_id,
            "frame": "table_mm",
            "measured_contour": {
                "frame": self.measured_contour.frame,
                "points": [list(point) for point in self.measured_contour.points],
            },
            "polygon_table_mm": [list(point) for point in self.polygon_table_mm],
            "pick_point_table_mm": (
                list(self.pick_point_table_mm) if self.pick_point_table_mm else None
            ),
            "pick_clearance_mm": self.pick_clearance_mm,
            "center_table_mm": list(self.center_table_mm),
            "yaw_deg": self.yaw_deg,
            "equivalent_yaws_deg": list(self.equivalent_yaws_deg),
            "confidence": self.confidence,
            "diagnostics": {
                "segmentation_stability": self.diagnostics.segmentation_stability,
                "polygon_fit_residual_mm": self.diagnostics.polygon_fit_residual_mm,
                "pick_clearance_ok": self.diagnostics.pick_clearance_ok,
                "rounded_corner_candidates": self.diagnostics.rounded_corner_candidates,
                "geometry_source": self.diagnostics.geometry_source,
                "touches_frame_border": self.diagnostics.touches_frame_border,
                "error_code": self.diagnostics.error_code,
                "extra": self.diagnostics.extra,
            },
        }


@dataclass(frozen=True)
class VisionResult:
    observations: tuple[PieceObservation, ...]
    status: str
    error_code: str | None
    profile: VisionProfile
    profile_version: str
    px_per_mm: float
    source_image_sha256: str
    timestamp_ns: int
    processing_time_ms: float


def _validate_frame(frame: RectifiedFrame, profile: VisionProfile) -> None:
    if frame.image.ndim not in (2, 3) or frame.image.size == 0:
        raise ValueError("RectifiedFrame.image 必须是非空灰度或 BGR 图像")
    if frame.image.ndim == 3 and frame.image.shape[2] != 3:
        raise ValueError("RectifiedFrame.image 必须是灰度或 BGR 三通道图像")
    if not math.isfinite(frame.px_per_mm) or frame.px_per_mm <= 0:
        raise ValueError("px_per_mm 必须是正有限数")
    if any(not math.isfinite(value) or value <= 0 for value in frame.table_mm_size):
        raise ValueError("table_mm_size 必须为正有限尺寸")
    if profile.max_pieces < 1 or profile.max_piece_sides not in range(3, 6):
        raise ValueError("profile 的片数或边数限制非法")
    if not math.isfinite(profile.min_edge_length_mm) or profile.min_edge_length_mm <= 0:
        raise ValueError("min_edge_length_mm 必须是正有限数")


def _confidence(stability: float, residual_mm: float, touches_border: bool) -> float:
    """未校准的保守组合 seam；阈值在真实标注集冻结前不得用于自动执行。"""
    fit_component = 1.0 / (1.0 + max(0.0, residual_mm))
    border_component = 0.55 if touches_border else 1.0
    return max(0.0, min(1.0, stability * fit_component * border_component))


def observe_pieces(frame: RectifiedFrame, profile: VisionProfile) -> VisionResult:
    """从已校正帧提取 1--4 个碎片，坏图以拒绝结果而非异常返回。"""
    _validate_frame(frame, profile)
    started_ns = time.monotonic_ns()
    candidates, error_code = segment_stable_pieces(frame, profile)
    if error_code is not None:
        return VisionResult(
            observations=(),
            status="REJECTED",
            error_code=error_code,
            profile=profile,
            profile_version=profile.profile_id,
            px_per_mm=frame.px_per_mm,
            source_image_sha256=frame.source_image_sha256,
            timestamp_ns=frame.timestamp_ns,
            processing_time_ms=(time.monotonic_ns() - started_ns) / 1_000_000,
        )

    observations: list[PieceObservation] = []
    for index, candidate in enumerate(candidates):
        geometry = build_piece_geometry(frame, profile, candidate)
        if geometry is None:
            # A stable blob that cannot meet the physical polygon contract is unsafe.
            return VisionResult(
                observations=(),
                status="REJECTED",
                error_code="VISION_MERGED_BLOBS",
                profile=profile,
                profile_version=profile.profile_id,
                px_per_mm=frame.px_per_mm,
                source_image_sha256=frame.source_image_sha256,
                timestamp_ns=frame.timestamp_ns,
                processing_time_ms=(time.monotonic_ns() - started_ns) / 1_000_000,
            )
        diagnostics = VisionDiagnostics(
            segmentation_stability=candidate.stability,
            polygon_fit_residual_mm=geometry.residual_mm,
            pick_clearance_ok=None,  # 磁头 footprint 尚未冻结。
            rounded_corner_candidates=geometry.rounded_candidates,
            geometry_source="fitted_polygon",
            touches_frame_border=candidate.touches_frame_border,
            extra={"threshold_matches": candidate.matches},
        )
        observations.append(
            PieceObservation(
                piece_id=f"piece-{index}",
                timestamp_ns=frame.timestamp_ns,
                source_image_id=frame.source_image_id,
                measured_contour=MeasuredContour("rectified_px", geometry.contour_px),
                polygon_table_mm=geometry.polygon_table_mm,
                pick_point_table_mm=None,
                pick_clearance_mm=geometry.pick_clearance_mm,
                center_table_mm=geometry.center_table_mm,
                yaw_deg=geometry.yaw_deg,
                equivalent_yaws_deg=geometry.equivalent_yaws_deg,
                confidence=_confidence(
                    candidate.stability,
                    geometry.residual_mm,
                    candidate.touches_frame_border,
                ),
                diagnostics=diagnostics,
            )
        )
    observations.sort(
        key=lambda observation: (
            -observation.center_table_mm[1],
            observation.center_table_mm[0],
            observation.piece_id,
        )
    )
    # Stable IDs must follow deterministic output ordering after sorting.
    observations = [
        observation
        if observation.piece_id == f"piece-{index}"
        else PieceObservation(
            piece_id=f"piece-{index}",
            timestamp_ns=observation.timestamp_ns,
            source_image_id=observation.source_image_id,
            measured_contour=observation.measured_contour,
            polygon_table_mm=observation.polygon_table_mm,
            pick_point_table_mm=observation.pick_point_table_mm,
            pick_clearance_mm=observation.pick_clearance_mm,
            center_table_mm=observation.center_table_mm,
            yaw_deg=observation.yaw_deg,
            equivalent_yaws_deg=observation.equivalent_yaws_deg,
            confidence=observation.confidence,
            diagnostics=observation.diagnostics,
        )
        for index, observation in enumerate(observations)
    ]
    return VisionResult(
        observations=tuple(observations),
        status="OK",
        error_code=None,
        profile=profile,
        profile_version=profile.profile_id,
        px_per_mm=frame.px_per_mm,
        source_image_sha256=frame.source_image_sha256,
        timestamp_ns=frame.timestamp_ns,
        processing_time_ms=(time.monotonic_ns() - started_ns) / 1_000_000,
    )
