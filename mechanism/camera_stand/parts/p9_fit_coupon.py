"""P9 配合试片：正式打印前先打小样验证本机孔/轴/槽补偿（约 30 分钟打印）。

P9a 母件：12.0 槽、M3 螺母预埋孔、M3 铜柱预埋孔、M3 自攻底孔、M3 通孔。
P9b 公件：11.8×11.8 插榫段。
验收：插榫入槽顺畅无可见晃动；螺母/铜柱压入到位；M3×6 自攻咬合不裂。
"""

import params as P
from build123d import Box, Cylinder, Part, Pos, Rot
from common import as_part, m3_nut_trap

BLOCK_W_MM = 70.0
BLOCK_D_MM = 30.0
BLOCK_H_MM = 8.0


def build_coupon() -> Part:
    block = as_part(Pos(0, 0, BLOCK_H_MM / 2) * Box(BLOCK_W_MM, BLOCK_D_MM, BLOCK_H_MM))
    # 12.0 槽（与 boardCam 叉口同宽），深 12、长 20，开口于 +X 端面
    block -= Pos(BLOCK_W_MM / 2 - 10, 0, BLOCK_H_MM / 2) * Box(
        20.0, P.FORK_SLOT_WIDTH_MM, BLOCK_H_MM + 0.4
    )
    # M3 螺母预埋孔（顶面开口）
    block -= Pos(-20, -8, BLOCK_H_MM - 2.6) * m3_nut_trap(2.6)
    # M3 铜柱预埋孔（顶面开口，深 8 贯通）
    block -= Pos(-20, 8, 0) * m3_nut_trap(BLOCK_H_MM)
    # M3 自攻底孔 Ø2.5 深 4
    block -= Pos(0, -8, BLOCK_H_MM - 4.0) * Cylinder(
        radius=P.SELF_TAP_PILOT_MM / 2, height=4.0
    )
    # M3 通孔 Ø3.2
    block -= Pos(0, 8, BLOCK_H_MM / 2) * Cylinder(
        radius=P.M3_CLEARANCE_MM / 2, height=BLOCK_H_MM
    )
    return block


def build_plug() -> Part:
    """11.8×11.8 插榫试件，长 25。"""
    plug = as_part(Pos(0, 0, 12.5) * Box(P.LUG_THICK_MM, P.LUG_DEPTH_MM, 25.0))
    # 顶部拉环孔，方便拔出
    plug -= (
        Pos(0, 0, 21.0)
        * Rot(90, 0, 0)
        * Cylinder(radius=2.0, height=P.LUG_DEPTH_MM + 0.4)
    )
    return plug


if __name__ == "__main__":
    a = build_coupon()
    b = build_plug()
    print("coupon valid:", a.is_valid, "plug valid:", b.is_valid)
