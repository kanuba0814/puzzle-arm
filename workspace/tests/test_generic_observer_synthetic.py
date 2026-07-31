# pyright: reportMissingImports=false

"""通用观察器合成回归测试。

场景建模自实机画面（stable_camera_annotated.jpg）：A4 白纸上铺黑色 A4，
黑纸有明显纵向亮度渐变与光泽，白色碎片位于上半源区，最多 4 片，
每片至少有一条边是拼好矩形的边（官方切割族性质）。
"""

import hashlib
import unittest

import cv2
import numpy as np

from vision import RectifiedFrame, VisionProfile, observe_pieces

PX_PER_MM = 3.0

# 四片、每片至少一条边为拼好矩形的边；坐标为校正图 mm（y 向下）。
SCENE_PIECES_MM = (
    ((30, 40), (90, 40), (90, 75), (30, 75)),  # 矩形
    ((110, 30), (170, 45), (125, 80)),  # 三角形
    ((40, 92), (95, 88), (100, 126), (55, 130)),  # 四边形
    ((120, 98), (175, 93), (180, 118), (140, 133)),  # 四边形（靠近分界线）
)


def _make_scene(
    piece_level: float = 225.0,
    paper_dark: float = 35.0,
    paper_light: float = 105.0,
    exposure: float = 1.0,
    pieces_mm: tuple = SCENE_PIECES_MM,
) -> RectifiedFrame:
    width = int(210 * PX_PER_MM)
    height = int(297 * PX_PER_MM)
    image = np.full((height, width), 195.0, np.float32)
    margin = int(6 * PX_PER_MM)  # A4 白边
    gradient = np.linspace(
        paper_dark, paper_light, height - 2 * margin, dtype=np.float32
    )
    image[margin : height - margin, margin : width - margin] = gradient[:, None]
    mid = height // 2  # 分界线（宽约 3 mm）
    half_line = int(1.5 * PX_PER_MM)
    image[mid - half_line : mid + half_line, margin : width - margin] = 15.0
    for polygon_mm in pieces_mm:
        points = (np.asarray(polygon_mm, np.float64) * PX_PER_MM).astype(np.int32)
        cv2.fillPoly(image, [points], float(piece_level))
    image *= exposure
    rng = np.random.default_rng(7)
    image += rng.normal(0.0, 3.0, image.shape).astype(np.float32)
    image = cv2.GaussianBlur(np.clip(image, 0, 255), (5, 5), 0)
    bgr = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    return RectifiedFrame(
        bgr,
        PX_PER_MM,
        (210.0, 297.0),
        hashlib.sha256(bgr.tobytes()).hexdigest(),
        "synthetic",
        1,
        "synthetic",
        "0" * 64,
        None,
    )


def _white_profile() -> VisionProfile:
    return VisionProfile("white-3ppmm", "white", False)


def _ground_truth_centers_table_mm() -> list[tuple[float, float]]:
    centers = []
    for polygon_mm in SCENE_PIECES_MM:
        array = np.asarray(polygon_mm, np.float64)
        center = array.mean(axis=0)
        centers.append((float(center[0]), 297.0 - float(center[1])))
    return centers


class GenericObserverSyntheticTest(unittest.TestCase):
    def test_white_pieces_on_gradient_black_paper(self) -> None:
        result = observe_pieces(_make_scene(), _white_profile())
        self.assertEqual(result.status, "OK")
        self.assertEqual(len(result.observations), 4)
        side_counts = sorted(len(o.polygon_table_mm) for o in result.observations)
        self.assertEqual(side_counts, [3, 4, 4, 4])
        for observation in result.observations:
            self.assertLess(observation.diagnostics.polygon_fit_residual_mm, 1.0)
            self.assertEqual(observation.diagnostics.geometry_source, "fitted_polygon")
            self.assertIsNone(observation.diagnostics.pick_clearance_ok)
            self.assertIsNotNone(observation.pick_point_table_mm)
            # 官方切割族：每片至少一条边 >= 20 mm（拼好矩形的边）。
            polygon = np.asarray(observation.polygon_table_mm)
            edge_lengths = np.linalg.norm(
                polygon - np.roll(polygon, -1, axis=0), axis=1
            )
            self.assertGreaterEqual(float(edge_lengths.max()), 20.0)
        # 中心定位误差 < 2 mm（一一匹配到真值质心）。
        truth = _ground_truth_centers_table_mm()
        for observation in result.observations:
            center = np.asarray(observation.center_table_mm)
            nearest = min(
                float(np.linalg.norm(center - np.asarray(t))) for t in truth
            )
            self.assertLess(nearest, 2.0)

    def test_low_exposure_frame_still_observed(self) -> None:
        # 广角相机对整体偏暗场景自动压暗曝光；固定 140 亮度阈值曾整帧丢失。
        result = observe_pieces(_make_scene(exposure=0.6), _white_profile())
        self.assertEqual(result.status, "OK")
        self.assertEqual(len(result.observations), 4)

    def test_bright_paper_and_dim_pieces_observed(self) -> None:
        result = observe_pieces(
            _make_scene(piece_level=180.0, paper_dark=60.0, paper_light=140.0),
            _white_profile(),
        )
        self.assertEqual(result.status, "OK")
        self.assertEqual(len(result.observations), 4)

    def test_piece_near_divider_is_not_clipped(self) -> None:
        # 第 4 片底边距分界线约 15 mm；ROI 下内缩必须不把它裁剪成 5 边。
        result = observe_pieces(_make_scene(), _white_profile())
        self.assertEqual(result.status, "OK")
        lowest = min(
            result.observations, key=lambda o: min(p[1] for p in o.polygon_table_mm)
        )
        self.assertEqual(len(lowest.polygon_table_mm), 4)

    def test_separated_white_rectangles_are_observed(self) -> None:
        pieces = (
            ((30, 40), (80, 40), (80, 100), (30, 100)),
            ((110, 50), (160, 50), (160, 110), (110, 110)),
        )
        result = observe_pieces(
            _make_scene(pieces_mm=pieces), _white_profile()
        )
        self.assertEqual(result.status, "OK")
        self.assertEqual(len(result.observations), 2)
        for observation in result.observations:
            self.assertEqual(len(observation.polygon_table_mm), 4)
            self.assertEqual(observation.diagnostics.rounded_corner_candidates, 0)
            self.assertGreaterEqual(len(observation.measured_contour.points), 3)

    def test_empty_black_paper_is_rejected(self) -> None:
        result = observe_pieces(_make_scene(pieces_mm=()), _white_profile())
        self.assertEqual(result.status, "REJECTED")
        self.assertEqual(result.error_code, "VISION_SURFACE_MISMATCH")

    def test_dark_surface_is_explicitly_rejected(self) -> None:
        image = np.full((594, 420, 3), 20, dtype=np.uint8)
        frame = RectifiedFrame(
            image,
            2.0,
            (210.0, 297.0),
            hashlib.sha256(image.tobytes()).hexdigest(),
            "synthetic",
            1,
            "synthetic",
            "0" * 64,
            None,
        )
        result = observe_pieces(frame, _white_profile())
        self.assertEqual(result.status, "REJECTED")
        self.assertEqual(result.error_code, "VISION_SURFACE_MISMATCH")


if __name__ == "__main__":
    unittest.main()
