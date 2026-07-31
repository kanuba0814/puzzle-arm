"""零件共用辅助函数。"""

from typing import cast

import params as P
from build123d import Box, Cylinder, Part, Pos, RegularPolygon, Shape, extrude


def as_part(shape: Shape | None) -> Part:
    """代数/构建器结果收窄为 Part（build123d 源码注解过宽，运行期恒为 Part）。"""
    return cast(Part, shape)


def m3_nut_trap(depth_mm: float) -> Part:
    """M3 螺母预埋孔切除体（六角柱），开口于 XY 平面、沿 +Z 延伸。

    六角对边取 M3_NUT_TRAP_AF_MM（含装配间隙）。
    """
    corner_r = (P.M3_NUT_TRAP_AF_MM / 2) / 0.8660254
    return as_part(
        extrude(RegularPolygon(radius=corner_r, side_count=6), amount=depth_mm)
    )


def m3_clearance_hole(depth_mm: float) -> Part:
    """M3 通孔切除体，以 XY 平面为起点沿 +Z。"""
    return as_part(
        Pos(0, 0, depth_mm / 2)
        * Cylinder(radius=P.M3_CLEARANCE_MM / 2, height=depth_mm)
    )


def radial_rib(length_mm: float, thick_mm: float, height_mm: float) -> Part:
    """径向加强筋：沿 +X 长度、Y 厚度、Z 高度，根部在原点。"""
    return as_part(
        Pos(length_mm / 2, 0, height_mm / 2) * Box(length_mm, thick_mm, height_mm)
    )
