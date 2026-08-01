#!/usr/bin/env python3
"""诊断：各 flat_th 检测 + 求解。"""
import sys
from pathlib import Path

import cv2
import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from vision._legacy_123_vision import PuzzleVision  # noqa: E402
from puzzle._legacy_123_assembly import AssemblySolver  # noqa: E402

frame = cv2.imread("/home/epuzzle/plan_v7/frame.jpg")
v = PuzzleVision(backend=None)
v.load_calibration("/opt/epuzzle/workspace/calibration/table_homography_charuco_20260801.npy")
quad = v.quad_from_homography()

for fth in (0.82, 0.78, 0.86, 0.74, 0.90, 0.70, 0.66):
    pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=fth)
    areas = sorted((round(p["area_mm2"]) for p in pieces), reverse=True)
    nv = sorted((len(p["vertices"]) for p in pieces), reverse=True)
    print(f"flat_th={fth}: {len(pieces)}片 面积={areas} 边数={nv}")
    if 3 <= len(pieces) <= 4:
        verts = [np.array(p["vertices"], dtype=np.float64) for p in pieces]
        solver = AssemblySolver(edge_tol=2.0, verbose=False)
        solver.fit_margin = 0.15
        sol = solver.solve(verts)
        status = "OK " + str(sol["rect_size"]) if sol else "无解"
        print(f"   -> {status}")
