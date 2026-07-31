"""P6 叉耳夹箍：C 形开口环抱紧 boardCam 叉耳块（35×12 区域），M3×6 + 预埋螺母锁紧。

局部坐标：环中心为原点，X 沿叉耳块宽（35 方向），Y 沿叉耳块深，Z 沿叉耳块高。
打印方向：平放（XY 贴床）。
"""

import params as P
from build123d import Box, Cylinder, Part, Pos, Rot
from common import as_part, m3_nut_trap

INNER_W_MM = 35.4  # 内宽 X（叉耳块 35 + 0.4）
INNER_D_MM = 12.4  # 内深 Y（叉口跨度 12 + 0.4）
WALL_MM = 3.0
BAND_H_MM = 10.0  # 环高（Z）
SPLIT_MM = 2.0  # 开口缝宽


def build() -> Part:
    outer_w = INNER_W_MM + 2 * WALL_MM
    outer_d = INNER_D_MM + 2 * WALL_MM
    band = as_part(Pos(0, 0, BAND_H_MM / 2) * Box(outer_w, outer_d, BAND_H_MM))
    band -= Pos(0, 0, BAND_H_MM / 2) * Box(INNER_W_MM, INNER_D_MM, BAND_H_MM)

    # 开口缝：+Y 侧壁中央沿 Z 贯通
    slit_y = INNER_D_MM / 2 + WALL_MM / 2
    band -= Pos(0, slit_y, BAND_H_MM / 2) * Box(SPLIT_MM, WALL_MM + 0.4, BAND_H_MM)

    # 夹紧耳：缝两侧沿 +Y 伸出，耳1（+X 侧，薄壁）通孔、耳2 预埋 M3 螺母
    ear1_x = SPLIT_MM / 2 + P.CLAMP_EAR_THIN_MM / 2
    ear2_x = SPLIT_MM / 2 + P.CLAMP_EAR_THICK_MM / 2
    ear_y = INNER_D_MM / 2 + WALL_MM + 4.0
    band += Pos(ear1_x, ear_y, BAND_H_MM / 2) * Box(
        P.CLAMP_EAR_THIN_MM, 10.0, BAND_H_MM
    )
    band += Pos(-ear2_x, ear_y, BAND_H_MM / 2) * Box(
        P.CLAMP_EAR_THICK_MM, 10.0, BAND_H_MM
    )
    # 耳1 通孔（X 向），耳2 螺母预埋孔（自缝面开口）
    band -= (
        Pos(ear1_x, ear_y, BAND_H_MM / 2)
        * Rot(0, 90, 0)
        * Cylinder(radius=P.M3_CLEARANCE_MM / 2, height=P.CLAMP_EAR_THIN_MM + 0.2)
    )
    band -= (
        Pos(-SPLIT_MM / 2, ear_y, BAND_H_MM / 2)
        * Rot(0, -90, 0)
        * m3_nut_trap(P.M3_NUT_THICK_MM + 0.2)
    )
    return as_part(band)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
