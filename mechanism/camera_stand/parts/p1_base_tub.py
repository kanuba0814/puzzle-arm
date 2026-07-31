"""P1a 底座桶：Ø166.2 镂空壳体 + 配重腔（顶部开口，灌沙后由 P1b 顶板封闭）。

局部坐标：Z=0 为桶底（即桌面），Z 向上。打印方向即使用方向（底面朝床）。
"""

import params as P
from build123d import Cylinder, Part, Pos, Rot
from common import as_part, radial_rib


def build() -> Part:
    outer_r = P.BASE_DIAMETER_MM / 2
    cavity_r = P.CAVITY_DIAMETER_MM / 2
    floor_t = P.BASE_TUB_HEIGHT_MM - P.CAVITY_DEPTH_MM  # 腔底壁厚

    # 外壳
    tub = as_part(
        Pos(0, 0, P.BASE_TUB_HEIGHT_MM / 2)
        * Cylinder(radius=outer_r, height=P.BASE_TUB_HEIGHT_MM)
    )
    # 配重腔（顶部开口，底部留 floor_t）
    tub -= Pos(0, 0, floor_t + P.CAVITY_DEPTH_MM / 2) * Cylinder(
        radius=cavity_r, height=P.CAVITY_DEPTH_MM
    )

    # 外环加强筋：连接腔壁与外壳，8 根径向筋
    rib_len = outer_r - cavity_r + 1.0  # 两端各嵌入 0.5
    for i in range(P.BASE_RIB_COUNT):
        ang = i * 360.0 / P.BASE_RIB_COUNT
        rib = (
            Rot(0, 0, ang)
            * Pos(cavity_r - 0.5, 0, floor_t)
            * radial_rib(rib_len, P.BASE_RIB_THICK_MM, P.CAVITY_DEPTH_MM)
        )
        tub += rib

    # 顶板连接：桶壁顶面 4 个 M3 自攻底孔（Ø2.5 深 4），位于环形壁中线上
    rim_r = (cavity_r + outer_r) / 2
    for ang in (45, 135, 225, 315):
        tub -= (
            Rot(0, 0, ang)
            * Pos(rim_r, 0, P.BASE_TUB_HEIGHT_MM - 2.0)
            * Cylinder(radius=P.SELF_TAP_PILOT_MM / 2, height=4.0)
        )
    return as_part(tub)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
