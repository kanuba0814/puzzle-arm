#!/usr/bin/env python3
"""验证已放置碎片 vs 计划目标位。"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

frame = cv2.imread("/home/epuzzle/verify_clear/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()
pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=0.82)

print("目标区 (y<148.5) 检测:")
placed = {}
for p in pieces:
    c = p["centroid"]
    if c[1] < 148.5:
        placed[p["id"]] = p
        print(
            f"  id{p['id']}: {len(p['vertices'])}边 {p['area_mm2']:.0f}mm² "
            f"中心({c[0]:.1f},{c[1]:.1f}) yaw={p.get('angle_deg', 0):.1f}"
        )

print()
print("计划目标位:")
d = json.load(open("/home/epuzzle/plan_v5/result.json"))
for pl in d["solve"]["placements"]:
    t = pl["target"]["center_mm"]
    print(f"  {pl['piece_id']}: ({t[0]:.1f},{t[1]:.1f})")

print()
print("=== 偏差对比（已放置 vs 目标）===")
# 目标区实测 id03 (514mm²) 应≈piece_00 (498mm²)
for pid, p in placed.items():
    c = p["centroid"]
    print(f"  实测 id{pid} ({p['area_mm2']:.0f}mm²) 中心({c[0]:.1f},{c[1]:.1f})")
    # 找最接近的目标位
    best = None
    best_d = 1e9
    for pl in d["solve"]["placements"]:
        t = pl["target"]["center_mm"]
        dist = np.hypot(c[0] - t[0], c[1] - t[1])
        if dist < best_d:
            best_d = dist
            best = pl
    if best:
        t = best["target"]["center_mm"]
        print(
            f"    → 最近目标 {best['piece_id']} ({t[0]:.1f},{t[1]:.1f}) "
            f"偏差 ({c[0]-t[0]:+.1f},{c[1]-t[1]:+.1f})mm = {best_d:.1f}mm"
        )
