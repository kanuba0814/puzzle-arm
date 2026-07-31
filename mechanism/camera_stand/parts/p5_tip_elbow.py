"""P5 顶端肘节板：11.8mm 单板，上端轮毂接 P4 叉口，下端插榫插入 boardCam 叉耳槽。

局部坐标：铰链孔心为原点，插榫指向 -Z（装配时竖直向下），板面为 XZ 平面。
装配：绕 stand Y 轴旋转俯仰角后平移到 HINGE=(230, 0, 219)。
打印方向：平放（板面贴床）。
"""

import math

import params as P
from build123d import (
    BuildPart,
    BuildSketch,
    Cylinder,
    Locations,
    Part,
    Pos,
    Rectangle,
    extrude,
)
from common import as_part, m3_nut_trap

# 与 P4 一致的梁轴参数（用于铜柱孔位对位）
_UX = 230.0 / math.hypot(230.0, 143.0)
_UZ = 143.0 / math.hypot(230.0, 143.0)
LOCK_RADII_MM = (18.0, 32.0)  # 同 P4

PLATE_T_MM = 11.8  # 板厚（Y 向）= 插榫厚度（叉口 12 - 0.2）
PLUG_DEPTH_MM = 11.8  # 插榫 Y 向深度（叉耳槽跨度 12 - 0.2，试片验证）
PLUG_LEN_MM = 40.0  # 插榫插入段长度（槽深约 40）
NECK_LEN_MM = 40.0  # 轮毂到叉耳顶的过渡段


def build() -> Part:
    with BuildPart() as bp:
        with BuildSketch():
            # 插榫 + 颈部：宽 11.8（X 居中），z 自 -80 到 0（与轮毂区重叠熔合）
            with Locations((0, -40)):
                Rectangle(PLATE_T_MM, PLUG_LEN_MM + NECK_LEN_MM)
            # 轮毂：覆盖铰链孔（原点）与两个铜柱孔位（x 到 -32*0.85）
            with Locations((-13, -9)):
                Rectangle(42.0, 34.0)
        extrude(amount=PLATE_T_MM / 2, both=True)
    elbow = as_part(bp.part)

    # 铰链孔 Ø8（原点，沿板法向 Z）
    elbow -= Pos(0, 0, 0) * Cylinder(
        radius=P.ELBOW_HINGE_PIN_D_MM / 2, height=PLATE_T_MM + 0.4
    )

    # 2 个 M3 铜柱预埋孔（-Z 面开口，深 8），位置与 P4 圆弧长孔对齐
    for radius in LOCK_RADII_MM:
        tx, tz = -radius * _UX, -radius * _UZ
        elbow -= Pos(tx, tz, -PLATE_T_MM / 2) * m3_nut_trap(P.STANDOFF_LEN_MM)
    return as_part(elbow)


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
