#!/usr/bin/env python3
"""诊断 #4（疑似反光误检）区域像素特征 vs 真碎片。"""
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
pre = v.preprocess(frame)
roi = v.paper_mask(pre.shape, quad, erode_px=8)

wb = v.white_balance(pre, roi)
S = cv2.cvtColor(wb, cv2.COLOR_BGR2HSV)[:, :, 1]
V = cv2.cvtColor(pre, cv2.COLOR_BGR2HSV)[:, :, 2]
pv = float(np.median(V[roi > 0]))
ps = float(np.median(S[roi > 0]))
print(f"paper_v={pv:.0f} paper_s={ps:.0f} black_bg={pv < 80}")

# 定位 #4（目标区 15.3, 22.9）和其他碎片的像素区域
# 通过单应把 mm 转回像素
H = v.homography
def mm2px(mm):
    p = np.array([mm[0], mm[1], 1.0])
    q = np.linalg.inv(H) @ p
    return (q[:2] / q[2]).astype(int)

# 检测所有轮廓，找 #4
bright = V > max(pv + 28.0, 45.0)
chrom = (S > max(ps + 30.0, 55.0)) & (V > max(pv + 15.0, 32.0))
pm = (bright | chrom).astype(np.uint8) * 255
pm = cv2.bitwise_and(pm, roi)
pm = cv2.morphologyEx(pm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
pm = cv2.morphologyEx(pm, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
cnts, _ = cv2.findContours(pm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

print(f"\n轮廓总数: {len(cnts)}")
for i, c in enumerate(sorted(cnts, key=cv2.contourArea, reverse=True)[:8]):
    cm = v.pixel_to_world(c.reshape(-1, 2))
    if cm is None:
        continue
    a_mm = abs(cv2.contourArea(cm.astype(np.float32)))
    c_mm = cm.mean(axis=0)
    x, y, w, h = cv2.boundingRect(c)
    # 区域统计
    blob = np.zeros(pm.shape, np.uint8)
    cv2.drawContours(blob, [c], 0, 255, -1)
    mean_v = float(np.mean(V[blob > 0]))
    mean_s = float(np.mean(S[blob > 0]))
    # 形状特征
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)
    aspect = max(w, h) / max(1, min(w, h))
    print(
        f"  轮廓{i}: {a_mm:.0f}mm² 中心({c_mm[0]:.1f},{c_mm[1]:.1f}) "
        f"bbox=({x},{y},{w},{h}) 长宽比={aspect:.1f} 顶点={len(approx)} "
        f"meanV={mean_v:.0f} meanS={mean_s:.0f}"
    )
