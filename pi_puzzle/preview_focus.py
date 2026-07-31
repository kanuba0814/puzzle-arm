"""相机实时预览窗口 (调焦/标定板摆放确认用)
叠加: ChArUco 标定板检测结果 (marker红框, 角点绿点, 棋盘区域蓝框)
      + 若存在标定单应 (homography_1440.npy 优先, 否则 homography.npy),
        叠加绿色A4面板框 (210x297mm, 含分界线)
q / Esc 退出, s 截图, 左键点击=打印图像坐标
"""
import os
import time
import urllib.request

import cv2
import numpy as np

from config import (MJPEG_SHOT1440_URL, A4_WIDTH, A4_HEIGHT, DIVIDER_LINE_Y,
                    CALIB_BOARD_SQUARES, CALIB_SQUARE_BIG_MM, CALIB_MARKER_BIG_MM)

url = MJPEG_SHOT1440_URL

_adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_board = cv2.aruco.CharucoBoard(CALIB_BOARD_SQUARES, CALIB_SQUARE_BIG_MM,
                                CALIB_MARKER_BIG_MM, _adict)
_detector = cv2.aruco.CharucoDetector(_board)


def load_a4_overlay():
    """世界->1440p像素 变换; 无标定文件返回None"""
    if os.path.exists('homography_1440.npy'):
        return np.linalg.inv(np.load('homography_1440.npy'))
    if os.path.exists('homography.npy'):
        H720 = np.load('homography.npy')          # px720 -> world
        return np.diag([2.0, 2.0, 1.0]) @ np.linalg.inv(H720)
    return None


_H_a4 = load_a4_overlay()
_A4_WORLD = np.array([[0, 0], [A4_WIDTH, 0], [A4_WIDTH, A4_HEIGHT],
                      [0, A4_HEIGHT]], dtype=np.float32)


def draw_a4(frame):
    if _H_a4 is None:
        return
    px = cv2.perspectiveTransform(_A4_WORLD.reshape(-1, 1, 2), _H_a4)
    px = px.reshape(-1, 2)
    cv2.polylines(frame, [px.astype(np.int32)], True, (0, 255, 0), 3)
    div = np.array([[0, DIVIDER_LINE_Y], [A4_WIDTH, DIVIDER_LINE_Y]],
                   dtype=np.float32).reshape(-1, 1, 2)
    dpx = cv2.perspectiveTransform(div, _H_a4).reshape(-1, 2)
    cv2.line(frame, tuple(dpx[0].astype(int)), tuple(dpx[1].astype(int)),
             (0, 200, 255), 2)
    for p, n in zip(px, ['(0,0)', '(210,0)', '(210,297)', '(0,297)']):
        cv2.putText(frame, n, (int(p[0]) + 8, int(p[1]) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


def overlay(frame):
    draw_a4(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, markers, mids = _detector.detectBoard(gray)
    n_mk = 0 if mids is None else len(mids)
    n_ck = 0 if ids is None else len(ids)
    if markers:
        cv2.aruco.drawDetectedMarkers(frame, markers, mids)
    if corners is not None and len(corners):
        pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        for x, y in pts:
            cv2.circle(frame, (int(x), int(y)), 6, (0, 255, 0), 2)
        hull = cv2.convexHull(pts.astype(np.int32))
        cv2.polylines(frame, [hull], True, (255, 0, 0), 3)
    cv2.putText(frame, f'markers={n_mk} corners={n_ck}', (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    return frame


cv2.namedWindow('preview', cv2.WINDOW_NORMAL)
cv2.resizeWindow('preview', 960, 540)


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f'[点击] 图像坐标(1440p) ({x},{y})  对应720p=({x//2},{y//2})', flush=True)


cv2.setMouseCallback('preview', on_mouse)

print(f"预览: {url}\nq/Esc退出  s截图  左键点击=打印坐标", flush=True)
while True:
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            data = r.read()
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    except Exception as e:
        print('取帧失败:', e, flush=True)
        time.sleep(0.5)
        continue
    if frame is None:
        continue
    cv2.imshow('preview', overlay(frame))
    k = cv2.waitKey(1) & 0xFF
    if k in (ord('q'), 27):
        break
    if k == ord('s'):
        p = f'out/demo/focus_{time.strftime("%H%M%S")}.jpg'
        cv2.imwrite(p, frame)
        print('已截图', p, flush=True)
    time.sleep(0.15)
cv2.destroyAllWindows()
