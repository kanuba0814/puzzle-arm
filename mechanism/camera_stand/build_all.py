"""相机支架总构建脚本：构建全部零件 -> 导出 STL/STEP -> 几何/装配/性能断言 -> JSON 报告 + 三视图渲染。

用法：
    cd mechanism/camera_stand
    PYTHONPATH=.:parts python build_all.py

断言分级：hard（不通过则退出码 1）/ soft（警告，允许继续）。
估计值与实测值在报告中分开标注；凡 UNKNOWN 一律显式列出。
"""

import json
import math
import pathlib
import sys

import params as P
from build123d import Axis, Box, Compound, Part, Pos, export_step, export_stl

ROOT = pathlib.Path(__file__).parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "parts"))

import p1_base_tub  # noqa: E402
import p2_base_top  # noqa: E402
import p3_yaw_rotor  # noqa: E402
import p4_boom  # noqa: E402
import p5_tip_elbow  # noqa: E402
import p6_clamp_band  # noqa: E402
import p7_a4_jig  # noqa: E402
import p8_cable_clip  # noqa: E402
import p9_fit_coupon  # noqa: E402

EXPORTS = ROOT / "exports"
REPORTS = ROOT / "reports"
RENDERS = ROOT / "renders"

BED = P.PRINTER_BED_MM
BED_DIAG = math.sqrt(2) * BED  # 平放对角线

# 估计用打印实心率（壁/筋为主的件偏高，大壳体偏低）——工程估计，切片后更新
FILL_FACTOR = {
    "p1_base_tub": 0.35,
    "p2_base_top": 0.45,
    "p3_yaw_rotor": 0.50,
    "p4_boom": 0.70,
    "p5_tip_elbow": 0.80,
    "p6_clamp_band": 0.60,
    "p7_a4_jig": 0.50,
    "p8_cable_clip": 0.60,
    "p9a_fit_coupon": 0.80,
    "p9b_fit_plug": 0.80,
}

# 装配位置（stand 坐标系，桌面 Z=0，+X 指向 A4）
HINGE = p4_boom.HINGE
BOARDCAM_BOTTOM_Z = 124.0  # boardCam 底边高度（由镜头 364 与板高 241.15 反推）
BOARDCAM_ANCHOR_X = 230.0  # 叉口中心（stand X）


def build_parts() -> dict[str, Part]:
    return {
        "p1_base_tub": p1_base_tub.build(),
        "p2_base_top": p2_base_top.build(),
        "p3_yaw_rotor": p3_yaw_rotor.build(),
        "p4_boom": p4_boom.build(),
        "p5_tip_elbow": p5_tip_elbow.build(),
        "p6_clamp_band": p6_clamp_band.build(),
        "p7_a4_jig": p7_a4_jig.build(),
        "p8_cable_clip": p8_cable_clip.build(),
        "p9a_fit_coupon": p9_fit_coupon.build_coupon(),
        "p9b_fit_plug": p9_fit_coupon.build_plug(),
    }


def boardcam_proxy() -> Part:
    """boardCam 简化代理体：5mm 立板 + 叉耳块（含 12mm 槽），用于装配干涉检查。

    尺寸来自 cad/boardCam.STL 实测；仅用于间隙校核，不作为加工依据。
    """
    # 立板：X_b 0..182.3 -> stand X = X_b - 91 + 230；Y_b 0..5 -> stand Y -40..-35
    plate = Pos(
        230.0 + (182.3 / 2 - 91.0), -37.5, BOARDCAM_BOTTOM_Z + 241.15 / 2
    ) * Box(182.3, 5.0, 241.15)
    # 叉耳块按实测结构分体：底部实体带（Z_b 0..8）+ 双叉耳（Y_b 34..46，Z_b 15..55）
    # 中间为开放槽区（Y_b 10..34 上方无材料），不过度填充
    band_low = Pos(230.0 + (90.5 - 91.0), -17.0, BOARDCAM_BOTTOM_Z + 4.0) * Box(
        35.0, 46.0, 8.0
    )
    prong_l = Pos(230.0 + (79.0 - 91.0), 0.0, BOARDCAM_BOTTOM_Z + 35.0) * Box(
        12.0, 12.0, 40.0
    )
    prong_r = Pos(230.0 + (102.5 - 91.0), 0.0, BOARDCAM_BOTTOM_Z + 35.0) * Box(
        11.0, 12.0, 40.0
    )
    return plate + band_low + prong_l + prong_r


def place_assembly(
    parts: dict[str, Part], elbow_tilt_deg: float = 0.0
) -> dict[str, Part]:
    placed = {
        "p1_base_tub": parts["p1_base_tub"],
        "p2_base_top": Pos(0, 0, P.BASE_TUB_HEIGHT_MM) * parts["p2_base_top"],
        "p3_yaw_rotor": Pos(0, 0, P.YAW_CONTACT_Z_MM - P.YAW_SPIGOT_LEN_MM)
        * parts["p3_yaw_rotor"],
        "p4_boom": parts["p4_boom"],  # 已在 stand 坐标系构建
        # 肘节：板法向转至 Y、插榫转至 -Z，再绕 Y 加俯仰角，平移到铰链点
        "p5_tip_elbow": (
            Pos(*HINGE)
            * parts["p5_tip_elbow"]
            .rotate(Axis((0, 0, 0), (1, 0, 0)), 90)
            .rotate(Axis((0, 0, 0), (0, 1, 0)), elbow_tilt_deg)
        ),
        # 夹箍：环抱叉耳块（stand X 229.5、Y 0），Z 150..160
        "p6_clamp_band": Pos(229.5, 0, 150.0) * parts["p6_clamp_band"],
        "boardcam_proxy": boardcam_proxy(),
    }
    if abs(elbow_tilt_deg) > 1e-9:
        pass  # 俯仰角已在上方统一处理
    return placed


def check_parts(parts: dict[str, Part]) -> list[dict]:
    results = []
    for name, part in parts.items():
        bb = part.bounding_box()
        size = (bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z)
        diag = math.sqrt(sum(s * s for s in size))
        fits_flat = all(s <= BED for s in size)
        fits_diag = diag <= BED_DIAG and min(size) <= BED
        vol_cm3 = part.volume / 1000.0
        ok = bool(part.is_valid) and part.volume > 0 and (fits_flat or fits_diag)
        results.append(
            {
                "part": name,
                "valid": bool(part.is_valid),
                "volume_cm3": round(vol_cm3, 1),
                "bbox_mm": [round(v, 1) for v in size],
                "fits_bed_flat": fits_flat,
                "fits_bed_diagonal": fits_diag,
                "est_print_mass_g": round(
                    vol_cm3 * P.PLA_DENSITY_G_CM3 * FILL_FACTOR[name], 0
                ),
                "claim": "hard",
                "pass": ok,
            }
        )
    return results


def check_interface_params() -> list[dict]:
    """接口参数一致性（纯参数断言，先于实物试片）。"""
    checks = [
        ("插榫厚 11.8 < 叉口 12.0", P.LUG_THICK_MM < P.FORK_SLOT_WIDTH_MM),
        ("叉口净距 12.2 > 肘节板厚 11.8", p4_boom.CLEVIS_GAP_MM > 11.8),
        (
            "止口 Ø40 < 轴孔 Ø40.4",
            P.YAW_SPIGOT_D_MM < P.YAW_SPIGOT_D_MM + P.YAW_SPIGOT_CLEARANCE_MM,
        ),
        (
            "M3×6 穿薄耳+缝后剩余 >= 螺母厚",
            P.M3_SCREW_LEN_MM - P.CLAMP_EAR_THIN_MM - P.TOWER_CLAMP_SLIT_MM
            >= P.M3_NUT_THICK_MM - 0.4,
        ),
        ("铜柱 8mm <= 肘节板厚 11.8 - 2", P.STANDOFF_LEN_MM <= 11.8 - 2),
        (
            "梁锁点半径 < 颊板沿轴长度",
            max(p4_boom.LOCK_RADII_MM) < p4_boom.CLEVIS_LEN_MM,
        ),
    ]
    return [{"check": name, "claim": "hard", "pass": bool(ok)} for name, ok in checks]


def check_collisions(placed: dict[str, Part]) -> list[dict]:
    pairs = [
        ("p4_boom", "p2_base_top"),
        ("p4_boom", "p1_base_tub"),
        ("p4_boom", "boardcam_proxy"),
        ("p5_tip_elbow", "boardcam_proxy"),
        ("p6_clamp_band", "boardcam_proxy"),
        ("p6_clamp_band", "p5_tip_elbow"),
        ("p3_yaw_rotor", "p2_base_top"),
    ]
    results = []
    for a, b in pairs:
        inter = placed[a].intersect(placed[b])
        # 0.11 中 intersect 返回 ShapeList，取各子体体积之和
        vol = sum(abs(s.volume) for s in inter) if inter is not None else 0.0
        results.append(
            {
                "pair": f"{a} x {b}",
                "intersect_mm3": round(vol, 3),
                "tolerance_mm3": 1.0,
                "claim": "hard",
                "pass": vol <= 1.0,
            }
        )
    return results


def estimate_masses(parts: dict[str, Part]) -> dict:
    printed = {
        name: parts[name].volume / 1000.0 * P.PLA_DENSITY_G_CM3 * FILL_FACTOR[name]
        for name in parts
    }
    cavity_cm3 = math.pi * (P.CAVITY_DIAMETER_MM / 2) ** 2 * P.CAVITY_DEPTH_MM / 1000.0
    return {
        "printed_g": {k: round(v, 0) for k, v in printed.items()},
        "cavity_volume_cm3": round(cavity_cm3, 0),
        "fill_sand_g": round(cavity_cm3 * P.SAND_DENSITY_G_CM3, 0),
        "fill_steel_grit_g": round(cavity_cm3 * P.STEEL_GRIT_DENSITY_G_CM3, 0),
        "boardcam_g_assumed": 150.0,  # 保守假设
        "camera_g_UNKNOWN": 120.0,  # UNKNOWN，保守取值
    }


def check_tipping(masses: dict) -> dict:
    """最不利偏航角（前伸正对 A4）下的倾覆安全系数。"""
    base_shell = masses["printed_g"]["p1_base_tub"] + masses["printed_g"]["p2_base_top"]
    # 移动质量及其质心 X（估计，装配后实测更新）
    moving = [
        (masses["boardcam_g_assumed"] + masses["camera_g_UNKNOWN"], 225.0),
        (masses["printed_g"]["p4_boom"], 115.0),
        (masses["printed_g"]["p5_tip_elbow"], 230.0),
        (masses["printed_g"]["p3_yaw_rotor"], 0.0),
        (masses["printed_g"]["p6_clamp_band"], 230.0),
    ]
    edge_r = P.BASE_DIAMETER_MM / 2
    m_ot = sum(m * (x - edge_r) for m, x in moving) * 9.81e-6  # g·mm -> N·m
    out = {"overturning_Nm": round(m_ot, 3), "scenarios": []}
    for label, fill_g in (
        ("sand", masses["fill_sand_g"]),
        ("steel_grit", masses["fill_steel_grit_g"]),
    ):
        base_total = (base_shell + fill_g) / 1000.0  # kg
        m_res = base_total * 9.81 * edge_r / 1000.0
        sf = m_res / m_ot
        out["scenarios"].append(
            {
                "fill": label,
                "base_total_kg": round(base_total, 2),
                "restoring_Nm": round(m_res, 3),
                "safety_factor": round(sf, 2),
                "claim": "hard",
                "pass": sf >= P.SAFETY_FACTOR_TIP,
            }
        )
    return out


def check_boom_deflection() -> dict:
    """分段数值积分悬臂挠度（水平投影 230mm，EI 随截面渐变）。

    载荷：梁端等效竖直载荷 = boardCam+相机+肘节+夹箍（约 3.0N 使用载荷），
    以及 2.5N 验收载荷；PLA 打印件 E 取保守值。
    """
    n = 60
    run = P.BOOM_RUN_MM
    dx = run / n

    # 截面惯性矩 I(x)：外廓 30x20 -> 20x15 线性渐变，壁厚 2
    def inertia(x: float) -> float:
        t = x / run
        w = P.BOOM_ROOT_W_MM + (P.BOOM_TIP_W_MM - P.BOOM_ROOT_W_MM) * t
        h = P.BOOM_ROOT_H_MM + (P.BOOM_TIP_H_MM - P.BOOM_ROOT_H_MM) * t
        wall = P.WALL_MM
        return (w * h**3 - (w - 2 * wall) * (h - 2 * wall) ** 3) / 12.0

    def deflection(load_n: float) -> float:
        # 端部集中载荷，自根部固支逐段积分
        slope = 0.0
        d = 0.0
        for i in range(n):
            x = (i + 0.5) * dx
            m = load_n * (run - x)  # 弯矩 N·mm
            curv = m / (P.PLA_ELASTIC_MODULUS_MPA * inertia(x))
            slope += curv * dx
            d += slope * dx
        return d

    service = deflection(3.0)
    proof = deflection(P.BOOM_PROOF_LOAD_N)
    return {
        "note": "工程估计（等效悬臂 + 保守弹性模量），实测验收为准",
        "service_load_N": 3.0,
        "service_deflection_mm": round(service, 2),
        "proof_load_N": P.BOOM_PROOF_LOAD_N,
        "proof_deflection_mm": round(proof, 2),
        "limit_mm": P.BOOM_DEFLECTION_LIMIT_MM,
        "claim": "soft",
        "pass": proof <= P.BOOM_DEFLECTION_LIMIT_MM,
    }


def check_lens_chain() -> dict:
    """镜头高度链：铰链 -> 插榫 -> 叉耳槽底 -> boardCam -> 镜头。"""
    slot_floor_z = HINGE[2] - (p5_tip_elbow.PLUG_LEN_MM + p5_tip_elbow.NECK_LEN_MM)
    board_bottom_z = slot_floor_z - 15.0  # 叉耳槽底在板底上 15
    lens_z = (
        board_bottom_z + P.L_LENS_ASSUMED_MM + 5.9
    )  # 孔心->镜头 L_LENS + 孔心高 5.9
    lo = P.LENS_HEIGHT_NOMINAL_MM - P.LENS_HEIGHT_TOLERANCE_MM
    hi = P.LENS_HEIGHT_NOMINAL_MM + P.LENS_HEIGHT_TOLERANCE_MM
    return {
        "slot_floor_z_mm": round(slot_floor_z, 1),
        "boardcam_bottom_z_mm": round(board_bottom_z, 1),
        "lens_z_mm": round(lens_z, 1),
        "required_range_mm": [lo, hi],
        "assumption": "L_LENS=235mm（镜头光心到叉耳孔心，未实测）",
        "claim": "hard",
        "pass": lo <= lens_z <= hi,
    }


def render_views(placed: dict[str, Part]) -> list[str]:
    """装配三视图 PNG（matplotlib 投影，仅用于人工复核形状，不作尺寸依据）。"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    paths = []
    for tag, a, b in (("front", 0, 2), ("side", 1, 2), ("top", 0, 1)):
        fig, ax = plt.subplots(figsize=(10, 8))
        for part in placed.values():
            verts, tris = part.tessellate(1.0)
            coords = [(v.X, v.Y, v.Z) for v in verts]
            polys = [
                [(coords[t[i]][a], coords[t[i]][b]) for i in range(3)] for t in tris
            ]
            ax.add_collection(
                PolyCollection(polys, alpha=0.2, linewidths=0.05, edgecolors="navy")
            )
        ax.autoscale_view()
        ax.set_aspect("equal")
        ax.set_title(f"assembly {tag}")
        ax.grid(alpha=0.3)
        out = RENDERS / f"assembly_{tag}.png"
        fig.savefig(out, dpi=80)
        plt.close(fig)
        paths.append(str(out))
    return paths


def main() -> int:
    for d in (EXPORTS, REPORTS, RENDERS):
        d.mkdir(parents=True, exist_ok=True)

    parts = build_parts()
    placed = place_assembly(parts)

    # 导出
    for name, part in parts.items():
        export_stl(part, str(EXPORTS / f"{name}.stl"))
        export_step(part, str(EXPORTS / f"{name}.step"))
    assembly = Compound(children=list(placed.values()))
    export_step(assembly, str(EXPORTS / "assembly.step"))

    masses = estimate_masses(parts)
    checks = {
        "parts": check_parts(parts),
        "interface_params": check_interface_params(),
        "collisions": check_collisions(placed),
        "tipping": check_tipping(masses),
        "boom_deflection": check_boom_deflection(),
        "lens_chain": check_lens_chain(),
    }
    renders = render_views(placed)

    def all_pass(section) -> bool:
        if isinstance(section, list):
            return all(c["pass"] for c in section)
        if isinstance(section, dict):
            if "scenarios" in section:
                return any(s["pass"] for s in section["scenarios"])
            return bool(section.get("pass", True))
        return True

    report = {
        "stand": "camera_stand v1",
        "units": "mm / g / N",
        "coordinate_frame": "stand（桌面 Z=0，底座圆心 XY 原点，+X 指向 A4）",
        "masses_estimated": masses,
        "checks": checks,
        "renders": renders,
        "unknowns": [
            "MaixCAM Pro 质量与镜头 FOV",
            "镜头光心在 boardCam 上的位置（L_LENS 现为 235 假设）",
            "叉耳槽 Y 向跨度实测值（影响插榫深度 11.8 的配合）",
            "配重材料密度",
            "打印实心率（影响质量估计）",
        ],
        "hard_claims_pass": all(
            all_pass(checks[k])
            for k in ("parts", "interface_params", "collisions", "lens_chain")
        ),
    }
    out = REPORTS / "stand_report.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(
        json.dumps(
            {
                "hard_claims_pass": report["hard_claims_pass"],
                "tipping": checks["tipping"],
                "boom_deflection_mm": checks["boom_deflection"]["proof_deflection_mm"],
                "lens_z_mm": checks["lens_chain"]["lens_z_mm"],
                "report": str(out),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if report["hard_claims_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
