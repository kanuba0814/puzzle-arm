"""P8 线缆夹：卡扣式线夹，固定 MaixCAM USB 线到斜梁侧面（3 件）。

局部坐标：Z=0 为贴合面，Z 向上。打印方向：平放。
用 2 条扎带穿过底部槽固定在斜梁上。
"""

from build123d import Box, Cylinder, Part, Pos
from common import as_part

CLIP_W_MM = 24.0  # 沿梁方向
CLIP_D_MM = 12.0  # 垂直梁方向
CLIP_H_MM = 10.0
CABLE_D_MM = 5.0  # USB 线直径
TIE_SLOT_W_MM = 3.0  # 扎带槽宽


def build() -> Part:
    clip = as_part(Pos(0, 0, CLIP_H_MM / 2) * Box(CLIP_W_MM, CLIP_D_MM, CLIP_H_MM))
    # 线缆槽：顶部开口的半圆槽（沿 X）
    clip -= Pos(0, 0, CLIP_H_MM - CABLE_D_MM / 2) * Cylinder(
        radius=CABLE_D_MM / 2, height=CLIP_W_MM, rotation=(0, 90, 0)
    )
    # 扎带槽 ×2（沿 Y 贯通，位于底部）
    for x in (-CLIP_W_MM / 4, CLIP_W_MM / 4):
        clip -= Pos(x, 0, CLIP_H_MM / 2) * Box(
            TIE_SLOT_W_MM, CLIP_D_MM + 0.4, CLIP_H_MM
        )
    return as_part(clip)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
