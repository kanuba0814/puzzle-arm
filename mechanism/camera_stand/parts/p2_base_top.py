"""P1b 底座顶板：封闭配重腔，中央塔柱提供偏航轴孔（开口夹环锁紧）。

局部坐标：Z=0 为顶板底面（装配时贴在 P1a 桶顶 Z=32），Z 向上。
装配后塔柱顶面 = 32 + 3 + 25 = 60（即偏航接触点高度）。
打印方向：顶板朝下贴床（塔柱朝上）——刻字面贴床，刻线清晰。
"""

import params as P
from build123d import Box, Cylinder, Part, Pos, Rot
from common import as_part, m3_nut_trap

TOWER_HEIGHT_MM = P.YAW_CONTACT_Z_MM - P.BASE_TUB_HEIGHT_MM - P.BASE_TOP_THICK_MM  # 25
SOCKET_D_MM = P.YAW_SPIGOT_D_MM + P.YAW_SPIGOT_CLEARANCE_MM  # 40.4
SOCKET_DEPTH_MM = P.YAW_SPIGOT_LEN_MM + P.YAW_SOCKET_EXTRA_DEPTH_MM  # 20
CLAMP_EAR_W_MM = 10.0  # 夹紧耳沿 X 宽度
CLAMP_EAR_H_MM = 12.0  # 夹紧耳沿 Z 高度
TOWER_TOP_LOCAL_Z = P.BASE_TOP_THICK_MM + TOWER_HEIGHT_MM  # 28（局部）


def build() -> Part:
    outer_r = P.BASE_DIAMETER_MM / 2
    rim_r = (P.CAVITY_DIAMETER_MM / 2 + outer_r) / 2
    plate = as_part(
        Pos(0, 0, P.BASE_TOP_THICK_MM / 2)
        * Cylinder(radius=outer_r, height=P.BASE_TOP_THICK_MM)
    )

    # 4 个 M3 通孔，与 P1a 桶壁自攻底孔对齐
    for ang in (45, 135, 225, 315):
        plate -= (
            Rot(0, 0, ang)
            * Pos(rim_r, 0, P.BASE_TOP_THICK_MM / 2)
            * Cylinder(radius=P.M3_CLEARANCE_MM / 2, height=P.BASE_TOP_THICK_MM)
        )

    # 中央塔柱（从顶板顶面起）
    plate += Pos(0, 0, TOWER_TOP_LOCAL_Z - TOWER_HEIGHT_MM / 2) * Cylinder(
        radius=P.TOWER_OUTER_D_MM / 2, height=TOWER_HEIGHT_MM
    )

    # 偏航轴孔（止口窝），从塔柱顶面向下 SOCKET_DEPTH
    plate -= Pos(0, 0, TOWER_TOP_LOCAL_Z - SOCKET_DEPTH_MM / 2) * Cylinder(
        radius=SOCKET_D_MM / 2, height=SOCKET_DEPTH_MM
    )

    # 开口夹环：塔柱顶部沿 X 切一条贯通缝（宽 TOWER_CLAMP_SLIT）
    ear_z = TOWER_TOP_LOCAL_Z - CLAMP_EAR_H_MM / 2
    plate -= Pos(0, 0, ear_z) * Box(
        P.TOWER_OUTER_D_MM, P.TOWER_CLAMP_SLIT_MM, CLAMP_EAR_H_MM
    )

    # 夹紧耳：缝两侧耳板，耳1（+Y，薄壁）通孔、耳2（-Y）预埋 M3 螺母
    ear1_y = P.TOWER_CLAMP_SLIT_MM / 2 + P.CLAMP_EAR_THIN_MM / 2
    ear2_y = P.TOWER_CLAMP_SLIT_MM / 2 + P.CLAMP_EAR_THICK_MM / 2
    ear_x = (
        P.TOWER_OUTER_D_MM / 2 - 1 + CLAMP_EAR_W_MM / 2
    )  # 耳中心 X（贴塔柱外圆向外伸）
    plate += Pos(ear_x, ear1_y, ear_z) * Box(
        CLAMP_EAR_W_MM, P.CLAMP_EAR_THIN_MM, CLAMP_EAR_H_MM
    )
    plate += Pos(ear_x, -ear2_y, ear_z) * Box(
        CLAMP_EAR_W_MM, P.CLAMP_EAR_THICK_MM, CLAMP_EAR_H_MM
    )
    # 耳1 通孔（Y 向），耳2 螺母预埋孔（自缝面开口，深 2.6）
    plate -= (
        Pos(ear_x, ear1_y, ear_z)
        * Rot(90, 0, 0)
        * Cylinder(radius=P.M3_CLEARANCE_MM / 2, height=P.CLAMP_EAR_THIN_MM + 0.2)
    )
    plate -= (
        Pos(ear_x, -P.TOWER_CLAMP_SLIT_MM / 2, ear_z)
        * Rot(90, 0, 0)
        * m3_nut_trap(P.M3_NUT_THICK_MM + 0.2)
    )

    # A4 定位刻线：+X 中心线（指向 A4）+ A4 短边位置横线，刻深 0.6
    mark_depth = 0.6
    top_z = P.BASE_TOP_THICK_MM - mark_depth / 2
    plate -= Pos(outer_r / 2, 0, top_z) * Box(outer_r - 10, 1.0, mark_depth)
    plate -= Pos(outer_r - 1, 0, top_z) * Box(2.0, 60.0, mark_depth)
    return as_part(plate)


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
