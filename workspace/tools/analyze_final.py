#!/usr/bin/env python3
"""最终拼装效果分析：目标区矩形 vs 计划目标位。"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

frame = cv2.imread("/home/epuzzle/final_v9/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()
H = v.homography

def mm2px(mm):
    p = np.array([mm[0], mm[1], 1.0])
    q = np.linalg.inv(H) @ p
    return (q[:2] / q[2]).astype(int)

# 全图白斑分析（不含检测过滤）
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
mask = (gray > 90).astype(np.uint8) * 255
roi = v.paper_mask(frame.shape, quad, erode_px=0)
mask = cv2.bitwise_and(mask, roi)
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

print("=== 全部白斑（mm）===")
for i, c in enumerate(sorted(cnts, key=cv2.contourArea, reverse=True)[:10]):
    cm = v.pixel_to_world(c.reshape(-1, 2))
    if cm is None:
        continue
    a = abs(cv2.contourArea(cm.astype(np.float32)))
    if a < 200:
        continue
    cc = cm.mean(axis=0)
    region = "目标区" if cc[1] < 148.5 else "源区"
    print(f"  {i}: {a:.0f}mm² 中心({cc[0]:.1f},{cc[1]:.1f}) [{region}]")

print()
print("=== 计划目标位 ===")
d = json.load(open("/home/epuzzle/plan_v9/result.json"))
for pl in d["solve"]["placements"]:
    t = pl["target"]["center_mm"]
    print(f"  {pl['piece_id']}: ({t[0]:.1f},{t[1]:.1f})")
