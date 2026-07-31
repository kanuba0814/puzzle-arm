"""P7 A4 定位角尺：L 形，贴底座标记摆放 A4 背景纸（2 件）。

局部坐标：内角为原点，两腿分别沿 +X、+Y。打印方向：平放。
"""

from build123d import Box, Part, Pos
from common import as_part

LEG_LEN_MM = 70.0
LEG_W_MM = 18.0
PLATE_T_MM = 3.0
LIP_H_MM = 6.0  # 内缘挡边高度
LIP_T_MM = 2.0


def build() -> Part:
    jig = as_part(
        Pos(LEG_LEN_MM / 2, LEG_W_MM / 2, PLATE_T_MM / 2)
        * Box(LEG_LEN_MM, LEG_W_MM, PLATE_T_MM)
    )
    jig += Pos(LEG_W_MM / 2, LEG_LEN_MM / 2, PLATE_T_MM / 2) * Box(
        LEG_W_MM, LEG_LEN_MM, PLATE_T_MM
    )
    # 内缘挡边（纸边贴合面）
    jig += Pos(
        LEG_LEN_MM / 2, LEG_W_MM + LIP_T_MM / 2, PLATE_T_MM + LIP_H_MM / 2
    ) * Box(LEG_LEN_MM, LIP_T_MM, LIP_H_MM)
    jig += Pos(
        LEG_W_MM + LIP_T_MM / 2, LEG_LEN_MM / 2, PLATE_T_MM + LIP_H_MM / 2
    ) * Box(LIP_T_MM, LEG_LEN_MM, LIP_H_MM)
    return as_part(jig)


if __name__ == "__main__":
    part = build()
    print("valid:", part.is_valid, "volume_cm3:", round(part.volume / 1000, 1))
