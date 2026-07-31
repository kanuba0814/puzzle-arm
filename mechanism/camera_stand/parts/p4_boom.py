"""P4 斜梁：变截面箱形梁，根部法兰接 P3 转子，顶端叉口（双颊板）接 P5 肘节。

坐标：直接在 stand 装配坐标系构建（桌面 Z=0，底座圆心为 XY 原点，+X 指向 A4）。
打印时需旋转至平放（Bambu Studio 自动定向，床面对角线方向）。
"""

import math

import params as P
from build123d import (
    Box,
    BuildPart,
    BuildSketch,
    CenterArc,
    Cylinder,
    Part,
    Plane,
    Pos,
    Rectangle,
    Rot,
    SlotArc,
    extrude,
    loft,
)
from common import as_part

# 梁轴线：根部截面中心 -> 顶端铰链孔中心
ROOT_C = (0.0, 0.0, 76.0)  # 根板顶 66 + 半高 10
HINGE = (230.0, 0.0, 219.0)  # 肘节铰链孔心（stand 坐标）
_RUN = HINGE[0] - ROOT_C[0]
_RISE = HINGE[2] - ROOT_C[2]
AXIS_LEN = math.hypot(_RUN, _RISE)  # 约 270.8
UX, UZ = _RUN / AXIS_LEN, _RISE / AXIS_LEN
ANGLE_DEG = math.degrees(math.atan2(_RISE, _RUN))  # 梁仰角约 31.9°
ANG_BACK_DEG = math.degrees(math.atan2(-UZ, -UX))  # 梁轴反方向（约 211.9°）

CLEVIS_GAP_MM = 12.2  # 叉口净距（肘节板 11.8 + 0.4）
CHEEK_THICK_MM = 4.0  # 厚颊板（+Y，铰链支撑侧）
CHEEK_THIN_MM = 2.0  # 薄颊板（-Y，锁紧螺丝侧）
CLEVIS_LEN_MM = 36.0  # 颊板沿轴长度（自铰链孔向后）
CLEVIS_H_MM = 22.0  # 颊板高度（垂直轴）
LOCK_RADII_MM = (18.0, 32.0)  # 两个圆弧长孔半径（均在颊板范围内）
SLOT_WIDTH_MM = 3.4
SLOT_HALF_ANGLE_DEG = 12.0  # 长孔半角（覆盖肘节 ±10° + 余量）


def _axis_pt(dist: float) -> tuple[float, float, float]:
    """轴线上距根部 dist 的点。"""
    return (ROOT_C[0] + UX * dist, 0.0, ROOT_C[2] + UZ * dist)


def _arc_slot_cutter(radius: float, y_center: float) -> Part:
    """以 HINGE 为圆心、半径 radius 的圆弧长孔切除体（沿 -Y 贯穿薄颊板）。

    草图平面 XY 对应 stand XZ（草图 Y = stand Z），挤压后旋转至 Y 向。
    """
    arc = CenterArc(
        (HINGE[0], HINGE[2]),
        radius,
        ANG_BACK_DEG - SLOT_HALF_ANGLE_DEG,
        2 * SLOT_HALF_ANGLE_DEG,
    )
    slot = extrude(SlotArc(arc=arc, height=SLOT_WIDTH_MM), amount=CHEEK_THIN_MM + 0.6)
    # 旋转：草图 +Z -> stand -Y，草图 (x, y) -> stand (x, z)
    return as_part(Pos(0, y_center, 0) * Rot(90, 0, 0) * slot)


def build() -> Part:
    axis_dir = (UX, 0.0, UZ)

    # 外轮廓 loft：根部 30x20 -> 顶端 20x15
    with BuildPart() as outer:
        with BuildSketch(Plane(origin=ROOT_C, x_dir=(0, 1, 0), z_dir=axis_dir)):
            Rectangle(P.BOOM_ROOT_W_MM, P.BOOM_ROOT_H_MM)
        with BuildSketch(
            Plane(
                origin=_axis_pt(AXIS_LEN - CLEVIS_LEN_MM),
                x_dir=(0, 1, 0),
                z_dir=axis_dir,
            )
        ):
            Rectangle(P.BOOM_TIP_W_MM, P.BOOM_TIP_H_MM)
        loft()

    # 内腔 loft（壁厚 2），两端各缩进 2
    w = P.WALL_MM
    with BuildPart() as inner:
        with BuildSketch(Plane(origin=_axis_pt(w), x_dir=(0, 1, 0), z_dir=axis_dir)):
            Rectangle(P.BOOM_ROOT_W_MM - 2 * w, P.BOOM_ROOT_H_MM - 2 * w)
        with BuildSketch(
            Plane(
                origin=_axis_pt(AXIS_LEN - CLEVIS_LEN_MM - w),
                x_dir=(0, 1, 0),
                z_dir=axis_dir,
            )
        ):
            Rectangle(P.BOOM_TIP_W_MM - 2 * w, P.BOOM_TIP_H_MM - 2 * w)
        loft()

    boom = as_part(outer.part) - as_part(inner.part)

    # 内部隔板：间距 30，厚 2，尺寸随截面内腔渐变
    inner_w0 = P.BOOM_ROOT_W_MM - 2 * w
    inner_h0 = P.BOOM_ROOT_H_MM - 2 * w
    inner_w1 = P.BOOM_TIP_W_MM - 2 * w
    inner_h1 = P.BOOM_TIP_H_MM - 2 * w
    span = AXIS_LEN - CLEVIS_LEN_MM - 2 * w
    n_ribs = math.floor(span / P.BOOM_RIB_PITCH_MM)
    for i in range(1, n_ribs + 1):
        d = w + i * P.BOOM_RIB_PITCH_MM
        if d >= AXIS_LEN - CLEVIS_LEN_MM - w:
            break
        t = (d - w) / span
        rw = inner_w0 + (inner_w1 - inner_w0) * t + 0.6  # 嵌入侧壁 0.3/边
        rh = inner_h0 + (inner_h1 - inner_h0) * t + 0.6
        boom += Pos(*_axis_pt(d)) * Rot(0, -ANGLE_DEG, 0) * Box(2.0, rw, rh)

    # 根部法兰板（水平，与 P3 转子法兰对接），z 64..66
    plate = as_part(Pos(0, 0, 65.0) * Box(44.0, 34.0, 2.0))
    for ang in (0, 90, 180, 270):
        plate -= (
            Rot(0, 0, ang)
            * Pos(P.ROTOR_BOLT_PCD_MM / 2, 0, 65.0)
            * Cylinder(radius=P.M3_CLEARANCE_MM / 2, height=2.0)
        )
    boom += plate

    # 顶端叉口双颊板：自铰链孔向后沿轴伸出 CLEVIS_LEN
    cheek_c = _axis_pt(AXIS_LEN - CLEVIS_LEN_MM / 2)
    y_thick = CLEVIS_GAP_MM / 2 + CHEEK_THICK_MM / 2
    y_thin = -(CLEVIS_GAP_MM / 2 + CHEEK_THIN_MM / 2)
    boom += (
        Pos(cheek_c[0], y_thick, cheek_c[2])
        * Rot(0, -ANGLE_DEG, 0)
        * Box(CLEVIS_LEN_MM, CHEEK_THICK_MM, CLEVIS_H_MM)
    )
    boom += (
        Pos(cheek_c[0], y_thin, cheek_c[2])
        * Rot(0, -ANGLE_DEG, 0)
        * Box(CLEVIS_LEN_MM, CHEEK_THIN_MM, CLEVIS_H_MM)
    )
    # 铰链孔（Y 向，贯穿双颊板）
    boom -= (
        Pos(*HINGE)
        * Rot(90, 0, 0)
        * Cylinder(
            radius=P.ELBOW_HINGE_PIN_D_MM / 2, height=CLEVIS_GAP_MM + 2 * CHEEK_THICK_MM
        )
    )
    # 薄颊板上的两个同心圆弧长孔（与 P5 肘节板铜柱孔位对齐，±12° 可调）
    for radius in LOCK_RADII_MM:
        boom -= _arc_slot_cutter(radius, y_thin)
    return as_part(boom)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
    bb = part.bounding_box()
    print(
        "bbox:",
        [
            round(v, 1)
            for v in (bb.min.X, bb.min.Y, bb.min.Z, bb.max.X, bb.max.Y, bb.max.Z)
        ],
    )
