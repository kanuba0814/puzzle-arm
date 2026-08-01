#!/usr/bin/env python3
"""生成 v11 视觉分析叠加图（检测+求解目标位）。"""
import json
import sys

import cv2
import numpy as np

_SRC = None
sys.path.insert(0, "/opt/epuzzle/workspace/src")
from vision._legacy_123_vision import PuzzleVision  # noqa: E402

frame = cv2.imread("/home/epuzzle/plan_v11/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()
H = v.homography
img = frame.copy()


def w2px(pt):
    p = np.array([pt[0], pt[1], 1.0])
    q = np.linalg.inv(H) @ p
    return (q[:2] / q[2]).astype(int)


pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=0.82)
for p in pieces:
    verts = p["vertices"]
    pts = np.array([w2px(x) for x in verts], dtype=np.int32)
    cv2.polylines(img, [pts], True, (0, 255, 0), 2)
    c = w2px(p["centroid"])
    cv2.circle(img, tuple(c), 6, (0, 255, 255), -1)
    label = "#%d %.0fmm2" % (p["id"], p["area_mm2"])
    cv2.putText(img, label, tuple(c + [0, -10]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

d = json.load(open("/home/epuzzle/plan_v11/result.json"))
for pl in d["solve"]["placements"]:
    t = pl["target"]["center_mm"]
    px = w2px(t)
    cv2.circle(img, tuple(px), 8, (0, 0, 255), -1)
    cv2.putText(img, pl["piece_id"], tuple(px + [10, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

p1, p2 = w2px([0, 148.5]), w2px([210, 148.5])
cv2.line(img, tuple(p1), tuple(p2), (255, 0, 255), 1, cv2.LINE_AA)
cv2.putText(img, "GREEN=detected RED=target YELLOW=centroid PURPLE=divider y=148.5",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
cv2.imwrite("/home/epuzzle/visual_analysis_v11.jpg", img)
print("v11 叠加图已生成: /home/epuzzle/visual_analysis_v11.jpg")
