#!/usr/bin/env python3
"""无头标定：自动抓帧 + ChArUco 角点检测 + 单应矩阵生成。

从 Pi 运行（SSH 无 GUI）：
    cd /home/epuzzle/puzzle
    /opt/epuzzle/venv/bin/python headless_calib.py
"""
import sys
import time

import cv2
import numpy as np

from camera_backend import create_camera_backend
from config import (
    A4_HEIGHT,
    CALIB_BOARD_SQUARES,
    CALIB_MARKER_BIG_MM,
    CALIB_SQUARE_BIG_MM,
)

# 页边距和坐标变换直接复用标定脚本的实现，避免重实现出错。
from calibrate_camera import board_obj_to_world, paper_margins_mm

print("========================================")
print(" ChArUco 无头标定（自动单帧）")
print(f" 板: {CALIB_BOARD_SQUARES} DICT_4X4_50")
print(f" 格边实测: {CALIB_SQUARE_BIG_MM} mm")
_ml, _mt = paper_margins_mm()
print(f" 页边距: 左{_ml:.2f} 上{_mt:.2f} mm")
print("========================================")

# 1. 抓帧
backend = create_camera_backend()
cap = backend.cap
print("预热相机 10 帧...")
for _ in range(10):
    cap.read()
time.sleep(0.8)

ret, frame = cap.read()
backend.release()
if not ret or frame is None:
    print("FAIL 抓帧失败")
    sys.exit(1)

h, w = frame.shape[:2]
print(f"抓帧: {w}x{h}")
cv2.imwrite("calib_headless.jpg", frame)

# 2. 构造 ChArUco 板
adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
if hasattr(cv2.aruco, "CharucoBoard"):
    board = cv2.aruco.CharucoBoard(
        CALIB_BOARD_SQUARES, CALIB_SQUARE_BIG_MM, CALIB_MARKER_BIG_MM, adict
    )
else:
    create_fn = vars(cv2.aruco)["CharucoBoard_create"]
    board = create_fn(
        CALIB_BOARD_SQUARES[0],
        CALIB_BOARD_SQUARES[1],
        CALIB_SQUARE_BIG_MM,
        CALIB_MARKER_BIG_MM,
        adict,
    )

# 3. 检测（使用新 API CharucoDetector 或回退旧 API）
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

if hasattr(cv2.aruco, "CharucoDetector"):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.CharucoDetector(board, detectorParams=params)
    ch_corners, ch_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    n_markers = len(marker_ids) if marker_ids is not None else 0
else:
    # OpenCV 4.6 旧 API
    params_fn = vars(cv2.aruco).get("DetectorParameters_create")
    params = params_fn() if params_fn else cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    
    detect_fn = vars(cv2.aruco)["detectMarkers"]
    marker_corners, marker_ids, rejected = detect_fn(gray, adict, parameters=params)
    n_markers = len(marker_ids) if marker_ids is not None else 0
    
    if n_markers < 4:
        print(f"FAIL 检测到 {n_markers} 个 ArUco marker，需要 ≥4")
        sys.exit(1)
    
    interp_fn = vars(cv2.aruco)["interpolateCornersCharuco"]
    ret, ch_corners, ch_ids = interp_fn(marker_corners, marker_ids, gray, board)
    if not ret or ch_ids is None:
        print("FAIL ChArUco 角点插值失败")
        sys.exit(1)

if ch_ids is None or len(ch_ids) < 10:
    n = len(ch_ids) if ch_ids is not None else 0
    print(f"FAIL ChArUco 角点不足 10 个（检测到 {n} 个，ArUco markers {n_markers}）")
    sys.exit(1)

print(f"检测: {n_markers} ArUco markers, {len(ch_ids)} ChArUco 角点")

# 4. 构造世界坐标（板坐标系 → A4 世界 mm）
# 直接用标定脚本的 board_obj_to_world，与生产端定义一致。
obj_pts = board.getChessboardCorners()
selected_obj = np.asarray(
    [obj_pts[int(cid[0])][:2] for cid in ch_ids], dtype=np.float64
)
world_pts = board_obj_to_world(selected_obj).astype(np.float32).reshape(-1, 1, 2)
img_pts = ch_corners.reshape(-1, 1, 2).astype(np.float32)

# 5. 拟合单应矩阵
H, mask = cv2.findHomography(img_pts, world_pts, cv2.RANSAC, 3.0)
inliers = int(mask.sum()) if mask is not None else 0

if inliers < 10:
    print(f"FAIL RANSAC 内点不足 10 个（{inliers}/{len(ch_ids)}）")
    sys.exit(1)

print(f"单应矩阵拟合: {inliers}/{len(ch_ids)} 内点")
print(H)

np.save("homography.npy", H)
print("✓ 已保存 homography.npy")

# 6. 缩放到 1440p 版本（如果当前是 720p）
if h == 720:
    H_1440 = H.copy()
    H_1440[0, :] *= 2.0
    H_1440[1, :] *= 2.0
    np.save("homography_1440.npy", H_1440)
    print("✓ 已生成 homography_1440.npy（2x 缩放）")

print("========================================")
print(" 标定完成")
print("========================================")
