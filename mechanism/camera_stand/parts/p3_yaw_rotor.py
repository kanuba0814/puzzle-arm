"""P3 偏航转子：止口插入 P1b 塔柱轴孔，法兰承托斜梁。

局部坐标：Z=0 为止口底面（装配时止口底位于 stand Z=42），Z 向上。
打印方向：法兰贴床，止口朝上。
"""

import params as P
from build123d import Cylinder, Part, Pos, Rot
from common import as_part


def build() -> Part:
    # 止口（插入塔柱轴孔 Ø40.4）
    rotor = as_part(
        Pos(0, 0, P.YAW_SPIGOT_LEN_MM / 2)
        * Cylinder(radius=P.YAW_SPIGOT_D_MM / 2, height=P.YAW_SPIGOT_LEN_MM)
    )
    # 法兰
    flange_z = P.YAW_SPIGOT_LEN_MM
    rotor += Pos(0, 0, flange_z + P.ROTOR_FLANGE_THICK_MM / 2) * Cylinder(
        radius=P.ROTOR_FLANGE_D_MM / 2, height=P.ROTOR_FLANGE_THICK_MM
    )

    # 4 个 M3 自攻底孔（斜梁根板螺丝从上面拧入），PCD 44，自法兰顶面向下
    for ang in (0, 90, 180, 270):
        rotor -= (
            Rot(0, 0, ang)
            * Pos(P.ROTOR_BOLT_PCD_MM / 2, 0, flange_z + P.ROTOR_FLANGE_THICK_MM - 1.5)
            * Cylinder(
                radius=P.SELF_TAP_PILOT_MM / 2, height=P.ROTOR_FLANGE_THICK_MM - 1.0
            )
        )
    return as_part(rotor)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
