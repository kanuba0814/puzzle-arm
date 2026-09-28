#!/usr/bin/env python3
"""处理剩余单片（piece_03）：检测源区未拼碎片 → 生成单片放置计划。"""
import json
import sys

sys.path.insert(0, "/home/epuzzle/puzzle")
from vision import PuzzleVision  # noqa: E402

v = PuzzleVision()
v.load_calibration("/home/epuzzle/puzzle/homography.npy")
frame = v.capture_frame()
quad = v.quad_from_homography()
frame = v.preprocess(frame)
pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=0.82)
print("检测:", len(pieces), "片")
for p in pieces:
    c = p["centroid"]
    print(
        "  #%d: %d边 %.0fmm² 中心(%.1f,%.1f) 握点(%.1f,%.1f) yaw=%.1f"
        % (p["id"], len(p["vertices"]), p["area_mm2"], c[0], c[1],
           p["grip_point"][0], p["grip_point"][1], p["angle_deg"])
    )

# 找源区（y>148.5）未拼的碎片（piece_03: 约1950mm² 4边）
src_pieces = [p for p in pieces if p["centroid"][1] > 148.5]
print(f"\n源区未拼: {len(src_pieces)} 片")
for p in src_pieces:
    c = p["centroid"]
    print(
        "  #%d: %.0fmm² 握点(%.1f,%.1f) yaw=%.1f"
        % (p["id"], p["area_mm2"], p["grip_point"][0], p["grip_point"][1], p["angle_deg"])
    )

if len(src_pieces) == 1:
    p = src_pieces[0]
    # 单片放置：放到目标区剩余位置（目标区三片已拼，piece_03 补缺口）
    # 用求解器对 4 片完整求解不可行（目标区已占），直接放目标区中心附近空缺
    # 简化：放目标区右下 (105+20, 74.25+10) 附近？—— 不行，必须拼对位置。
    # 正确做法：用原始完整计划的目标位（piece_03 目标 (68.7, 59.9)）
    plan = json.load(open("/home/epuzzle/visionpack_out/plan_visionpack.json"))
    targets = {pl["piece_id"]: pl for pl in plan["solve"]["placements"]}
    # 上一轮 plan 的 piece_03 目标
    if "piece_03" in targets:
        t = targets["piece_03"]["target"]["center_mm"]
        ty = targets["piece_03"]["target"]["yaw_deg"]
        sy = float(p["angle_deg"])
        plan_out = {
            "solve": {
                "status": "SOLVED",
                "error_code": None,
                "inferred_target_mm": [59.3, 97.8],
                "placements": [
                    {
                        "piece_id": "piece_03",
                        "source": {
                            "frame": "table_mm",
                            "center_mm": [p["grip_point"][0], p["grip_point"][1]],
                            "yaw_deg": sy,
                        },
                        "target": {
                            "frame": "table_mm",
                            "center_mm": [t[0], t[1]],
                            "yaw_deg": ty,
                        },
                    }
                ],
            },
            "vision_pack": True,
        }
        json.dump(plan_out, open("/home/epuzzle/visionpack_out/plan_piece03.json", "w"),
                  ensure_ascii=False, indent=2)
        print(f"\npiece_03 单片计划: ({p['grip_point'][0]:.1f},{p['grip_point'][1]:.1f}) → ({t[0]:.1f},{t[1]:.1f}) Δyaw={ty-sy:+.1f}°")
    else:
        print("计划中无 piece_03")
else:
    print("源区碎片数 != 1，需人工确认")

v.release()
