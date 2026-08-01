#!/usr/bin/env python3
"""检测+求解结果叠加原图可视化。

- 绿框：检测到的碎片轮廓（源区）
- 红框：求解出的目标位（拼装矩形 + 各片目标位置）
- 蓝点：握点（抓取点）
- 黄点：质心
"""
import sys
from pathlib import Path

import cv2
import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

frame = cv2.imread("/home/epuzzle/plan_v7/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()

# 检测
pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=0.82)
img = frame.copy()
H = v.homography

def w2px(pt):
    """table_mm -> 像素"""
    p = np.array([pt[0], pt[1], 1.0])
    q = np.linalg.inv(H) @ p
    return (q[:2] / q[2]).astype(np.int32)

def draw_poly_mm(poly, color, thick=2, label=None):
    pts = np.array([w2px(v) for v in poly], dtype=np.int32)
    cv2.polylines(img, [pts], True, color, thick)
    return pts.mean(axis=0)

print(f"检测 {len(pieces)} 片:")
for p in pieces:
    verts = p["vertices"]
    c = p["centroid"]
    g = p.get("grip_point")
    # 绿框：源碎片
    cc = draw_poly_mm(verts, (0, 255, 0), 2)
    # 黄点：质心
    cv2.circle(img, tuple(w2px(c)), 6, (0, 255, 255), -1)
    # 蓝点：握点
    if g is not None:
        cv2.circle(img, tuple(w2px(g)), 5, (255, 128, 0), -1)
    cv2.putText(img, f"#{p['id']} {p['area_mm2']:.0f}mm2", tuple(w2px(c) + [0, -12]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    print(f"  #{p['id']}: {len(verts)}边 {p['area_mm2']:.0f}mm² 质心({c[0]:.1f},{c[1]:.1f})")

# 求解目标位
try:
    import json
    d = json.load(open("/home/epuzzle/plan_v7/result.json"))
    placements = d.get("solve", {}).get("placements", [])
    print(f"\n求解 placements: {len(placements)}")
    for pl in placements:
        t = pl["target"]["center_mm"]
        cv2.circle(img, tuple(w2px(t)), 8, (0, 0, 255), -1)
        cv2.putText(img, pl["piece_id"], tuple(w2px(t) + [10, 0]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        print(f"  {pl['piece_id']} 目标({t[0]:.1f},{t[1]:.1f})")
except Exception as exc:
    print(f"求解结果读取失败: {exc}")

cv2.imwrite("/home/epuzzle/overlay_plan_v7.jpg", img)
print("\n已保存 /home/epuzzle/overlay_plan_v7.jpg")
