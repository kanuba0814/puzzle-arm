#!/usr/bin/env python3
"""检测+求解结果叠加原图可视化（v8 分析版）。

- 绿框：源区检测碎片（y>=148.5）
- 橙框：目标区检测碎片（y<148.5，疑似残留）
- 红框：求解目标位（拼装矩形位置）
- 黄点：质心  蓝点：握点
- 分界线：y=148.5 虚线
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

frame = cv2.imread("/home/epuzzle/plan_v8/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()
H = v.homography
img = frame.copy()

def w2px(pt):
    p = np.array([pt[0], pt[1], 1.0])
    q = np.linalg.inv(H) @ p
    return (q[:2] / q[2]).astype(np.int32)

def draw_poly_mm(poly, color, thick=2):
    pts = np.array([w2px(v) for v in poly], dtype=np.int32)
    cv2.polylines(img, [pts], True, color, thick)
    return pts.mean(axis=0)

# 分界线 y=148.5
p1 = w2px([0, 148.5]); p2 = w2px([210, 148.5])
cv2.line(img, tuple(p1), tuple(p2), (255, 0, 255), 2, cv2.LINE_AA)
cv2.putText(img, "DIVIDER y=148.5", tuple(p1 + [5, -8]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)

# 检测
pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=0.82)
print(f"检测 {len(pieces)} 片:")
src_count, tgt_count = 0, 0
for p in pieces:
    verts = p["vertices"]
    c = p["centroid"]
    g = p.get("grip_point")
    in_tgt = c[1] < 148.5
    color = (0, 165, 255) if in_tgt else (0, 255, 0)  # 橙=目标区 绿=源区
    tag = "目标区!" if in_tgt else "源区"
    cc = draw_poly_mm(verts, color, 2)
    cv2.circle(img, tuple(w2px(c)), 6, (0, 255, 255), -1)
    if g is not None:
        cv2.circle(img, tuple(w2px(g)), 5, (255, 128, 0), -1)
    label = f"#{p['id']} {p['area_mm2']:.0f}mm2 {tag}"
    cv2.putText(img, label, tuple(w2px(c) + [0, -12]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if in_tgt: tgt_count += 1
    else: src_count += 1
    print(f"  #{p['id']}: {len(verts)}边 {p['area_mm2']:.0f}mm² 中心({c[0]:.1f},{c[1]:.1f}) {tag}")

# 求解结果
try:
    d = json.load(open("/home/epuzzle/plan_v8/result.json"))
    solve = d.get("solve", {})
    placements = solve.get("placements", [])
    status = solve.get("status", "?")
    rect = solve.get("inferred_target_mm")
    print(f"\n求解: {status} 反推矩形: {rect}")
    for pl in placements:
        t = pl["target"]["center_mm"]
        cv2.circle(img, tuple(w2px(t)), 8, (0, 0, 255), -1)
        cv2.putText(img, pl["piece_id"], tuple(w2px(t) + [10, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        print(f"  {pl['piece_id']} → 目标({t[0]:.1f},{t[1]:.1f})")
    # 目标矩形轮廓（从 placements 目标位包围盒）
    if placements:
        ts = np.array([pl["target"]["center_mm"] for pl in placements])
        cx, cy = ts.mean(axis=0)
        w_mm, h_mm = (rect if rect else (98, 61))
        # 画矩形（近似：以质心 bbox 中心为矩形中心）
        lo, hi = ts.min(axis=0), ts.max(axis=0)
        ctr = (lo + hi) / 2
        half = (hi - lo) / 2 + 5
        pts = np.array([[ctr[0]-half[0], ctr[1]-half[1]], [ctr[0]+half[0], ctr[1]-half[1]],
                        [ctr[0]+half[0], ctr[1]+half[1]], [ctr[0]-half[0], ctr[1]+half[1]]])
        draw_poly_mm(pts, (0, 0, 255), 2)
except Exception as exc:
    print(f"求解读取失败: {exc}")

# 图例
cv2.putText(img, "GREEN=源区  ORANGE=目标区残留  RED=求解目标位  YELLOW=质心  BLUE=握点",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
cv2.putText(img, f"源区 {src_count} 片 | 目标区 {tgt_count} 片",
            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

out = "/home/epuzzle/overlay_analysis_v8.jpg"
cv2.imwrite(out, img)
print(f"\n已保存 {out}")
