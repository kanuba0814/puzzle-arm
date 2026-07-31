"""
引导式相机标定 (ChArUco 8x11 标定板, A4打印)

解决的问题:
    旧流程每次运行都靠 auto_detect_paper 检测纸边缘重建单应, 精度差且不稳定,
    导致 CV 识别后给出的移动目标/结果不对。本脚本用 ChArUco 标定板建立精确的
    像素->世界(mm) 映射并持久化 (homography.npy), 生产端直接加载使用。

标定板 (celibrater/E题视觉标定板_A4_ChArUco_8x11_22mm.pdf):
    8x11格 DICT_4X4_50, 标称格边22mm, 实测 21.12mm (config.CALIB_SQUARE_BIG_MM)
    OpenCV板坐标系: 原点在网格左上角(页头文字朝上), x向右, y向下, 单位=实测mm

世界坐标系 (与 vision.set_paper_homography 一致):
    原点 = 打印纸左下角 (页头朝上), x向右 0-210 (短边), y向上 0-297 (长边)
    物理网格边距 (打印缩放绕页面中心假设, s=21.12/22=0.96):
        左 20.52mm  上 20.34mm  (可用 config.CALIB_PAPER_MARGIN_OVERRIDE 覆盖)

默认流程（固定机位工作区标定）:
    标定板平放在固定A4工作区，稳定后自动拍摄一帧。利用最多70个角点超定拟合
    工作平面像素->世界(mm)单应。固定相机和A4工作区不移动时只需执行一次，之后
    生产程序直接复用持久化结果，不需要多角度拍摄。

高级流程（仅更换镜头、焦距或相机成像配置时）:
    --full-intrinsics 先采平放帧，再手持标定板倾斜覆盖画面，用于重新估计
    镜头内参/畸变。它不是固定工作区投入使用的必要步骤。

用法:
    python3 calibrate_camera.py                    # 默认：单帧平面工作区标定
    python3 calibrate_camera.py --full-intrinsics  # 高级：多角度内参+工作区标定
    python3 calibrate_camera.py --images DIR       # 复用已存平放帧离线重算
    python3 calibrate_camera.py --preview-only     # 只用已有单应开叠加预览
按键: SPACE=手动拍摄  N=下一阶段  R=重置本阶段  S=截图  Q/Esc=退出
"""

import argparse
import json
import operator
import os
import sys
import time

import cv2
import numpy as np

from config import (
    A4_HEIGHT,
    A4_WIDTH,
    CALIB_BOARD_SQUARES,
    CALIB_IMAGE_SIZE,
    CALIB_MARGIN_MODE,
    CALIB_MARKER_BIG_MM,
    CALIB_MIN_CORNERS_DESK,
    CALIB_MIN_CORNERS_HAND,
    CALIB_MIN_DESK,
    CALIB_MIN_HAND,
    CALIB_PAPER_MARGIN_OVERRIDE,
    CALIB_PDF_GRID_BIG,
    CALIB_SQUARE_BIG_MM,
    CALIB_STABLE_PX,
    CALIB_STABLE_SEC,
    CAMERA_HEIGHT_MM,
    DIVIDER_LINE_Y,
    MAIXCAM_UVC_INDEX,
    MAIXCAM_UVC_SIZE,
    MJPEG_SHOT1440_URL,
    PROD_IMAGE_SIZE,
    SAVE_CALIB_FRAMES,
)

NOMINAL_SQUARE_MM = 22.0  # 大板PDF标称格边

# ---------------- 标定板模型 (obj单位=实测mm, 原点网格左上, x右 y下) ----------------
_adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
if hasattr(cv2.aruco, "CharucoBoard"):
    BOARD = cv2.aruco.CharucoBoard(
        CALIB_BOARD_SQUARES, CALIB_SQUARE_BIG_MM, CALIB_MARKER_BIG_MM, _adict
    )
else:  # OpenCV 4.6 (Raspberry Pi OS Bookworm系统包)
    _create_board = vars(cv2.aruco)["CharucoBoard_create"]
    BOARD = _create_board(
        CALIB_BOARD_SQUARES[0],
        CALIB_BOARD_SQUARES[1],
        CALIB_SQUARE_BIG_MM,
        CALIB_MARKER_BIG_MM,
        _adict,
    )
if hasattr(cv2.aruco, "DetectorParameters"):
    _params = cv2.aruco.DetectorParameters()
else:
    _params = vars(cv2.aruco)["DetectorParameters_create"]()
_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
DETECTOR = (
    cv2.aruco.CharucoDetector(BOARD, detectorParams=_params)
    if hasattr(cv2.aruco, "CharucoDetector")
    else None
)


def paper_margins_mm(mode=None):
    """物理打印: 网格左/上外缘到纸张左/上边缘的距离 (mm)
    mode: 'center'=绕页面中心缩放(默认) / 'proportional'=按打印前比例同比缩放
    OVERRIDE 优先于一切假设"""
    if CALIB_PAPER_MARGIN_OVERRIDE is not None:
        return tuple(CALIB_PAPER_MARGIN_OVERRIDE)
    s = CALIB_SQUARE_BIG_MM / NOMINAL_SQUARE_MM
    g = CALIB_PDF_GRID_BIG
    mode = mode or CALIB_MARGIN_MODE
    if mode == "proportional":
        return s * g["left"], s * g["top"]
    return (
        (A4_WIDTH - s * A4_WIDTH) / 2 + s * g["left"],
        (A4_HEIGHT - s * A4_HEIGHT) / 2 + s * g["top"],
    )


def board_obj_to_world(pts_obj):
    """板obj坐标(实测mm) -> 世界mm (原点纸左下, x右0-210, y上0-297)"""
    ml, mt = paper_margins_mm()
    pts = np.asarray(pts_obj, dtype=np.float64).reshape(-1, 2)
    wx = ml + pts[:, 0]
    wy = (A4_HEIGHT - mt) - pts[:, 1]
    return np.stack([wx, wy], axis=1)


def world_to_board_obj(pts_world, margins=None):
    """世界mm -> 板obj坐标(实测mm), 用于把A4角点投影回图像"""
    ml, mt = margins if margins is not None else paper_margins_mm()
    pts = np.asarray(pts_world, dtype=np.float64).reshape(-1, 2)
    bx = pts[:, 0] - ml
    by = (A4_HEIGHT - mt) - pts[:, 1]
    return np.stack([bx, by], axis=1)


def _dashed_polyline(img, pts, color, dash=16, gap=12, thickness=2):
    pts = pts.astype(np.int32).reshape(-1, 2)
    for i in range(len(pts)):
        p0 = pts[i].astype(np.float64)
        p1 = pts[(i + 1) % len(pts)].astype(np.float64)
        L = np.linalg.norm(p1 - p0)
        if L < 1:
            continue
        d = (p1 - p0) / L
        t = 0.0
        while t < L:
            t2 = min(L, t + dash)
            cv2.line(
                img,
                tuple((p0 + d * t).astype(int)),
                tuple((p0 + d * t2).astype(int)),
                color,
                thickness,
            )
            t += dash + gap


def draw_panel_hypothesis(img, H_obj2px, margins, color, dashed=False, divider=False):
    """按给定页边距假设, 把A4面板从板obj系投影到图像并绘制"""
    a4_obj = world_to_board_obj(A4_CORNERS_WORLD, margins)
    pts = a4_obj.reshape(-1, 1, 2).astype(np.float32)
    px = cv2.perspectiveTransform(pts, H_obj2px).reshape(-1, 2)
    if dashed:
        _dashed_polyline(img, px, color)
    else:
        cv2.polylines(img, [px.astype(np.int32)], True, color, 3)
        for p, n in zip(px, ["(0,0)", "(210,0)", "(210,297)", "(0,297)"]):
            cv2.circle(img, tuple(p.astype(int)), 6, color, -1)
            cv2.putText(
                img,
                n,
                (int(p[0]) + 10, int(p[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2,
            )
    if divider:
        div_obj = world_to_board_obj(
            [[0, DIVIDER_LINE_Y], [A4_WIDTH, DIVIDER_LINE_Y]], margins
        )
        dpx = cv2.perspectiveTransform(
            div_obj.reshape(-1, 1, 2).astype(np.float32), H_obj2px
        ).reshape(-1, 2)
        cv2.line(
            img, tuple(dpx[0].astype(int)), tuple(dpx[1].astype(int)), (0, 200, 255), 2
        )
    return px


A4_CORNERS_WORLD = np.array(
    [[0, 0], [A4_WIDTH, 0], [A4_WIDTH, A4_HEIGHT], [0, A4_HEIGHT]], dtype=np.float64
)  # BL, BR, TR, TL


# ---------------- 检测 ----------------


def detect(frame):
    """检测ChArUco角点；兼容OpenCV 4.6旧API和4.8+新API。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if DETECTOR is not None:
        corners, ids, _, _ = DETECTOR.detectBoard(gray)
    else:
        detect_markers = vars(cv2.aruco)["detectMarkers"]
        interpolate = vars(cv2.aruco)["interpolateCornersCharuco"]
        marker_corners, marker_ids, _ = detect_markers(gray, _adict, parameters=_params)
        if marker_ids is None or len(marker_ids) < 1:
            return None, None
        _, corners, ids = interpolate(marker_corners, marker_ids, gray, BOARD)
    if ids is None or corners is None or len(ids) < 4:
        return None, None
    return corners, ids


def obj_points(ids):
    """检测到的角点id -> 板obj 3D坐标 (z=0, 实测mm)"""
    raw_points = (
        BOARD.getChessboardCorners()
        if hasattr(BOARD, "getChessboardCorners")
        else operator.attrgetter("chessboardCorners")(BOARD)
    )
    allpts = np.asarray(raw_points, dtype=np.float32)  # (70,3)
    indices = np.asarray(ids, dtype=np.intp).ravel()
    return np.take(allpts, indices, axis=0)


# ---------------- 稳定度自动拍摄 ----------------


class Stability:
    def __init__(self):
        self.prev = None  # (ids_set, {id: pt})
        self.stable_since = None
        self.disp = None
        self.armed = True  # 拍完后需移开板子重新武装

    def update(self, corners, ids):
        if corners is None:
            self.prev = None
            self.stable_since = None
            self.disp = None
            return False
        cur = {int(i): p for i, p in zip(ids.ravel(), corners.reshape(-1, 2))}
        now = time.time()
        if self.prev is not None:
            common = [k for k in cur if k in self.prev[1]]
            if len(common) >= 6:
                d = np.mean([np.linalg.norm(cur[k] - self.prev[1][k]) for k in common])
                self.disp = d
                if not self.armed:
                    if d > 5.0:  # 板子已移动, 重新武装
                        self.armed = True
                    self.stable_since = None
                elif d < CALIB_STABLE_PX:
                    if self.stable_since is None:
                        self.stable_since = now
                    elif now - self.stable_since >= CALIB_STABLE_SEC:
                        self.armed = False
                        self.stable_since = None
                        self.prev = (set(cur), cur)
                        return True  # 稳定到时 -> 自动拍摄
                else:
                    self.stable_since = None
        self.prev = (set(cur), cur)
        return False

    def progress(self):
        if self.stable_since is None:
            return 0.0
        return min(1.0, (time.time() - self.stable_since) / CALIB_STABLE_SEC)


# ---------------- 可视化叠加 ----------------


def draw_a4_panel(img, H_world2px, color=(0, 255, 0), thickness=3, label=True):
    """按 世界->像素 单应绘制A4面板: 外框+分界线+角点标注"""
    pts = A4_CORNERS_WORLD.reshape(-1, 1, 2).astype(np.float32)
    px = cv2.perspectiveTransform(pts, H_world2px).reshape(-1, 2)
    cv2.polylines(img, [px.astype(np.int32)], True, color, thickness)
    # 分界线 y=148.5
    div = np.array(
        [[0, DIVIDER_LINE_Y], [A4_WIDTH, DIVIDER_LINE_Y]], dtype=np.float32
    ).reshape(-1, 1, 2)
    dpx = cv2.perspectiveTransform(div, H_world2px).reshape(-1, 2)
    cv2.line(
        img, tuple(dpx[0].astype(int)), tuple(dpx[1].astype(int)), (0, 200, 255), 2
    )
    if label:
        names = ["(0,0)", "(210,0)", "(210,297)", "(0,297)"]
        for p, n in zip(px, names):
            cv2.circle(img, tuple(p.astype(int)), 6, color, -1)
            cv2.putText(
                img,
                n,
                (int(p[0]) + 10, int(p[1]) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2,
            )
    return px


def draw_detection(img, corners, ids):
    if corners is None:
        return
    for p in corners.reshape(-1, 2):
        cv2.circle(img, tuple(p.astype(int)), 5, (0, 255, 0), 2)
    hull = cv2.convexHull(corners.reshape(-1, 2).astype(np.int32))
    cv2.polylines(img, [hull], True, (255, 0, 0), 3)


def put_status(img, lines, y0=50, dy=44):
    for i, (text, color) in enumerate(lines):
        cv2.putText(
            img, text, (25, y0 + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 4
        )
        cv2.putText(
            img, text, (25, y0 + i * dy), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2
        )


# ---------------- 帧采集 (交互) ----------------


class FrameRec:
    """一帧的检测记录"""

    __slots__ = ("corners", "frame", "ids", "stamp")

    def __init__(self, frame, corners, ids):
        self.frame = frame
        self.corners = corners
        self.ids = ids
        self.stamp = time.strftime("%H%M%S")


def collect_frames(backend, save_dir=None, desk_only=False):
    """
    引导式采集: 平放帧(desk) -> 手持帧(hand); desk_only=True 时仅平放帧。
    返回 (desk_frames, hand_frames) = list of FrameRec; 中止返回 (None, None)
    """
    W, H = CALIB_IMAGE_SIZE
    cv2.namedWindow("calib", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("calib", 1280, 720)

    desk, hand = [], []
    stab = Stability()
    coverage = np.zeros((3, 3), bool)  # 手持帧 3x3 覆盖格
    H_anchor = None  # 第1平放帧 世界->像素
    last_shot = 0

    phase = "desk"
    print("\n[采集] 阶段1: 平放帧")
    print("  第1帧: 大板平放在A4工作区上, 页头文字朝上, 纸下边缘朝向机械臂")
    print("  预览绿框 = 计算出的A4面板, 请核对与实际纸边重合")
    required_desk = 1 if desk_only else CALIB_MIN_DESK
    if desk_only:
        print("  保持标定板不动，稳定后自动拍摄1张并计算（固定工作区模式）")
    else:
        print("  稳定后自动拍摄; SPACE手动拍; N满%d张后进手持阶段" % required_desk)

    while True:
        frame = backend.read_rgb()
        if frame is None:
            time.sleep(0.2)
            continue
        if frame.shape[1] != W or frame.shape[0] != H:
            frame = cv2.resize(frame, (W, H))
        corners, ids = detect(frame)
        n_corners = 0 if ids is None else len(ids)
        fire = stab.update(corners, ids)

        vis = frame.copy()
        draw_detection(vis, corners, ids)

        # --- A4面板叠加 ---
        if phase == "desk" and H_anchor is None:
            # 实时: 由当前板位外推 (obj->px 单应 + obj系A4角点)
            # 同时画出两种页边距假设: 实线绿=采用, 虚线紫=备选, 肉眼比对纸边
            if (
                n_corners >= CALIB_MIN_CORNERS_DESK
                and corners is not None
                and ids is not None
            ):
                H_obj2px, _ = cv2.findHomography(
                    obj_points(ids)[:, :2], corners.reshape(-1, 2), method=0
                )
                if H_obj2px is not None:
                    m_act = paper_margins_mm()
                    draw_panel_hypothesis(
                        vis, H_obj2px, m_act, (0, 255, 0), divider=True
                    )
                    if CALIB_PAPER_MARGIN_OVERRIDE is None:
                        alt = (
                            "proportional"
                            if CALIB_MARGIN_MODE == "center"
                            else "center"
                        )
                        m_alt = paper_margins_mm(alt)
                        draw_panel_hypothesis(
                            vis, H_obj2px, m_alt, (255, 0, 255), dashed=True
                        )
                        cv2.putText(
                            vis,
                            f"GREEN: margin L{m_act[0]:.1f} T{m_act[1]:.1f}"
                            f" [{CALIB_MARGIN_MODE}]  MAGENTA(dash):"
                            f" L{m_alt[0]:.1f} T{m_alt[1]:.1f} [{alt}]",
                            (25, H - 80),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8,
                            (0, 255, 255),
                            2,
                        )
        elif H_anchor is not None:
            # 锚点已建立: 固定A4面板叠加
            draw_a4_panel(vis, H_anchor)

        # --- 状态行 (屏幕用英文, 详细中文指引看终端) ---
        if phase == "desk":
            lines = [
                (
                    (
                        f"[WORKSPACE] frame {len(desk)}/{required_desk}"
                        if desk_only
                        else f"[1/2] DESK frames {len(desk)}/{required_desk}"
                    )
                    + f"  corners={n_corners}",
                    (0, 255, 255),
                ),
                (
                    "frame1: board flat on A4 area, header UP, bottom edge to ARM",
                    (200, 200, 200),
                ),
            ]
            if len(desk) == 0:
                lines.append(
                    (
                        "GREEN box = computed A4 panel, check it matches paper edges",
                        (0, 255, 0),
                    )
                )
            else:
                lines.append(
                    (
                        "move board (still flat) for more frames, N = next phase",
                        (200, 200, 200),
                    )
                )
        else:
            n_cov = int(coverage.sum())
            lines = [
                (
                    f"[2/2] HAND frames {len(hand)}/{CALIB_MIN_HAND}"
                    f"  coverage {n_cov}/9  corners={n_corners}",
                    (0, 255, 255),
                ),
                ("hold board tilted, cover image corners/edges", (200, 200, 200)),
            ]
            if len(hand) >= CALIB_MIN_HAND:
                lines.append(("OK: N = start computing", (0, 255, 0)))
        if stab.disp is not None:
            lines.append((f"jitter: {stab.disp:.2f}px", (180, 180, 180)))
        # 稳定进度条
        prog = stab.progress()
        if prog > 0:
            cv2.rectangle(
                vis, (25, H - 60), (25 + int(400 * prog), H - 30), (0, 255, 0), -1
            )
            cv2.rectangle(vis, (25, H - 60), (425, H - 30), (255, 255, 255), 2)
        put_status(vis, lines)

        # 手持覆盖格可视化 (右下角小图)
        if phase == "hand":
            for r in range(3):
                for c in range(3):
                    x0 = W - 170 + c * 50
                    y0 = H - 180 + r * 50
                    col = (0, 255, 0) if coverage[r, c] else (60, 60, 60)
                    cv2.rectangle(vis, (x0, y0), (x0 + 44, y0 + 44), col, -1)
                    cv2.rectangle(vis, (x0, y0), (x0 + 44, y0 + 44), (255, 255, 255), 1)

        disp = cv2.resize(vis, (1280, 720))
        cv2.imshow("calib", disp)
        key = cv2.waitKey(1) & 0xFF

        # --- 拍摄逻辑 ---
        min_corners = (
            CALIB_MIN_CORNERS_DESK if phase == "desk" else CALIB_MIN_CORNERS_HAND
        )
        want = fire or key == ord(" ")
        if (
            want
            and n_corners >= min_corners
            and corners is not None
            and ids is not None
            and time.time() - last_shot > 0.5
        ):
            rec = FrameRec(frame, corners.copy(), ids.copy())
            last_shot = time.time()
            if phase == "desk":
                desk.append(rec)
                print(f"  [平放{len(desk)}] corners={n_corners} 已拍摄")
                if len(desk) == 1:
                    # 建立锚点: world->px = (obj->px) @ inv(obj->world)
                    H_obj2px, _ = cv2.findHomography(
                        obj_points(ids)[:, :2], corners.reshape(-1, 2), method=0
                    )
                    ml_, mt_ = paper_margins_mm()
                    M = np.array(
                        [[1, 0, ml_], [0, -1, A4_HEIGHT - mt_], [0, 0, 1]],
                        dtype=np.float64,
                    )
                    H_anchor = H_obj2px @ np.linalg.inv(M)
                    print("  [锚点] A4面板已锁定, 后续叠加为固定面板")
                if save_dir:
                    cv2.imwrite(
                        os.path.join(save_dir, f"desk_{len(desk) - 1:02d}.jpg"), frame
                    )
            else:
                hand.append(rec)
                # 覆盖格: 角点落入的格子
                for p in corners.reshape(-1, 2):
                    c = min(2, int(p[0] / (W / 3)))
                    r = min(2, int(p[1] / (H / 3)))
                    coverage[r, c] = True
                print(
                    f"  [手持{len(hand)}] corners={n_corners} "
                    f"覆盖{int(coverage.sum())}/9 已拍摄"
                )
                if save_dir:
                    cv2.imwrite(
                        os.path.join(save_dir, f"hand_{len(hand) - 1:02d}.jpg"), frame
                    )

        # --- desk-only: 平放帧满即结束采集 ---
        if desk_only and len(desk) >= required_desk:
            print("  [平放] 单帧采集完成，自动进入计算")
            break

        # --- 阶段切换 ---
        if key == ord("n") or key == ord("N"):
            if not desk_only and phase == "desk" and len(desk) >= required_desk:
                phase = "hand"
                stab = Stability()
                print("\n[采集] 阶段2: 手持帧")
                print("  手持板子倾斜, 覆盖画面四角/边缘 (右下角9宫格全绿)")
                print("  稳定后自动拍摄; N满%d张后开始计算" % CALIB_MIN_HAND)
            elif phase == "hand" and len(hand) >= CALIB_MIN_HAND:
                break
        if key == ord("r") or key == ord("R"):
            if phase == "desk":
                desk, H_anchor = [], None
            else:
                hand = []
                coverage[:] = False
            stab = Stability()
            print(f"  [重置] {phase}阶段已清空")
        if key == ord("s") or key == ord("S"):
            p = f"out/calib_shot_{time.strftime('%H%M%S')}.jpg"
            os.makedirs("out", exist_ok=True)
            cv2.imwrite(p, vis)
            print("  截图", p)
        if key in (ord("q"), 27):
            cv2.destroyWindow("calib")
            return None, None

    cv2.destroyWindow("calib")
    return desk, hand


# ---------------- 标定计算 ----------------


def calibrate_desk_only(desk):
    """单平面模式: 跳过内参, 仅由锚点帧全部角点超定拟合单应 (工作面=桌面)"""
    W, H = CALIB_IMAGE_SIZE
    rec0 = desk[0]
    obj0 = obj_points(rec0.ids)[:, :2]
    world0 = board_obj_to_world(obj0)
    px0 = rec0.corners.reshape(-1, 2).astype(np.float64)
    H1440, _ = cv2.findHomography(px0, world0, method=0)
    proj = cv2.perspectiveTransform(
        px0.reshape(-1, 1, 2).astype(np.float32), H1440.astype(np.float32)
    ).reshape(-1, 2)
    res = np.sqrt(((proj - world0) ** 2).sum(axis=1))
    print(
        f"\n[单应] 锚点帧 {len(px0)}角点 残差: "
        f"RMS={np.sqrt((res**2).mean()):.3f}mm  max={res.max():.3f}mm"
    )

    # 其余平放帧一致性交叉检查 (若有多帧): 同一桌面, 残差应同量级
    for k, rec in enumerate(desk[1:], 1):
        objk = obj_points(rec.ids)[:, :2]
        worldk = board_obj_to_world(objk)
        pxk = rec.corners.reshape(-1, 2).astype(np.float64)
        projk = cv2.perspectiveTransform(
            pxk.reshape(-1, 1, 2).astype(np.float32), H1440.astype(np.float32)
        ).reshape(-1, 2)
        rk = np.sqrt(((projk - worldk) ** 2).sum(axis=1))
        print(
            f"  平放{k}: {len(pxk)}角点 残差RMS={np.sqrt((rk**2).mean()):.3f}mm "
            f"max={rk.max():.3f}mm"
        )

    sx = W / PROD_IMAGE_SIZE[0]
    sy = H / PROD_IMAGE_SIZE[1]
    S = np.diag([sx, sy, 1.0])
    H720 = H1440 @ S
    # 若标定分辨率不是2560x1440 (如UVC 720p通道), 由标定分辨率单应派生1440p版
    if (W, H) != (2560, 1440):
        H1440 = H1440 @ np.diag([W / 2560.0, H / 1440.0, 1.0])

    invH720 = np.linalg.inv(H720)
    quad720 = cv2.perspectiveTransform(
        A4_CORNERS_WORLD.reshape(-1, 1, 2).astype(np.float32),
        invH720.astype(np.float32),
    ).reshape(4, 2)

    return {
        "K1440": None,
        "dist": None,
        "rms": None,
        "K720": None,
        "H1440": H1440,
        "H720": H720,
        "homo_res_rms_mm": float(np.sqrt((res**2).mean())),
        "homo_res_max_mm": float(res.max()),
        "desk_z_mm": None,
        "desk_dist_mm": None,
        "distortion_max_px": None,
        "quad720": quad720,
        "per_view_err": [],
    }


def calibrate(desk, hand):
    """内参标定 + 锚点单应, 返回结果dict; 失败返回None"""
    W, H = CALIB_IMAGE_SIZE
    frames = desk + hand
    obj_pts, img_pts = [], []
    for rec in frames:
        obj_pts.append(obj_points(rec.ids))
        img_pts.append(rec.corners.reshape(-1, 2).astype(np.float32))
    print(
        f"\n[标定] {len(desk)}平放 + {len(hand)}手持 = {len(frames)}视角, 求解内参..."
    )
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, (W, H), None, None
    )
    print(f"  重投影RMS = {rms:.3f} px")
    print(f"  K =\n{np.round(K, 2)}")
    print(f"  dist = {np.round(dist.ravel(), 5)}")

    # 每视角误差
    errs = []
    for i in range(len(frames)):
        proj, _ = cv2.projectPoints(obj_pts[i], rvecs[i], tvecs[i], K, dist)
        e = np.sqrt(((proj.reshape(-1, 2) - img_pts[i]) ** 2).sum(axis=1))
        errs.append(float(e.mean()))
    for i, e in enumerate(errs):
        tag = "desk" if i < len(desk) else "hand"
        print(f"    视角{i:02d} [{tag}] 平均误差 {e:.3f}px")

    # --- 锚点单应 (第1平放帧, 全部角点超定拟合) ---
    rec0 = desk[0]
    obj0 = obj_points(rec0.ids)[:, :2]
    world0 = board_obj_to_world(obj0)
    px0 = rec0.corners.reshape(-1, 2).astype(np.float64)
    H1440, mask = cv2.findHomography(px0, world0, method=0)
    proj = cv2.perspectiveTransform(
        px0.reshape(-1, 1, 2).astype(np.float32), H1440.astype(np.float32)
    ).reshape(-1, 2)
    res = np.sqrt(((proj - world0) ** 2).sum(axis=1))
    print(
        f"\n[单应] 锚点帧 {len(px0)}角点 残差: "
        f"RMS={np.sqrt((res**2).mean()):.3f}mm  max={res.max():.3f}mm"
    )

    # 720p 生产版: world = H1440 @ px1440, px1440 = 2*px720
    sx = W / PROD_IMAGE_SIZE[0]
    sy = H / PROD_IMAGE_SIZE[1]
    S = np.diag([sx, sy, 1.0])
    H720 = H1440 @ S

    # K720 (仅供参考/后续去畸变用)
    K720 = K.copy()
    K720[0, 0] /= sx
    K720[1, 1] /= sy
    K720[0, 2] /= sx
    K720[1, 2] /= sy

    # --- 桌面距离检查 (实测格边/相机高度的自洽性) ---
    ok, rvec, tvec = cv2.solvePnP(obj_points(rec0.ids), px0.astype(np.float32), K, dist)
    tvec = tvec.ravel()
    z = float(tvec[2])
    dist_mm = float(np.linalg.norm(tvec))
    print(
        f"[检查] 锚点板距相机: z={z:.0f}mm |t|={dist_mm:.0f}mm "
        f"(config.CAMERA_HEIGHT_MM={CAMERA_HEIGHT_MM})"
    )
    if abs(dist_mm - CAMERA_HEIGHT_MM) / CAMERA_HEIGHT_MM > 0.15:
        print(
            "  [警告] 与CAMERA_HEIGHT_MM偏差>15%: 请检查实测格边21.12mm/"
            "相机高度/纸张边距假设"
        )

    # --- 畸变量评估 (图像四角去畸变位移) ---
    corners_px = np.array(
        [[0, 0], [W, 0], [W, H], [0, H], [W / 2, H / 2]], dtype=np.float32
    ).reshape(-1, 1, 2)
    und = cv2.undistortPoints(corners_px, K, dist, P=K).reshape(-1, 2)
    shift = np.sqrt(((und - corners_px.reshape(-1, 2)) ** 2).sum(axis=1))
    print(f"[畸变] 角点最大位移 {shift.max():.2f}px @1440p")

    # A4四角在图像中的位置 (720p, 供生产端ROI/核对)
    invH720 = np.linalg.inv(H720)
    quad720 = cv2.perspectiveTransform(
        A4_CORNERS_WORLD.reshape(-1, 1, 2).astype(np.float32),
        invH720.astype(np.float32),
    ).reshape(4, 2)

    return {
        "K1440": K,
        "dist": dist,
        "rms": float(rms),
        "K720": K720,
        "H1440": H1440,
        "H720": H720,
        "homo_res_rms_mm": float(np.sqrt((res**2).mean())),
        "homo_res_max_mm": float(res.max()),
        "desk_z_mm": z,
        "desk_dist_mm": dist_mm,
        "distortion_max_px": float(shift.max()),
        "quad720": quad720,
        "per_view_err": errs,
    }


def save_results(res, save_dir):
    """持久化: homography.npy(720p) / homography_1440.npy / npz / summary"""
    np.save("homography.npy", res["H720"])
    np.save("homography_1440.npy", res["H1440"])
    ml, mt = paper_margins_mm()
    npz = {
        "H1440": res["H1440"],
        "H720": res["H720"],
        "image_size_1440": np.array(CALIB_IMAGE_SIZE),
        "image_size_720": np.array(PROD_IMAGE_SIZE),
        "square_mm": CALIB_SQUARE_BIG_MM,
        "margin_left_mm": ml,
        "margin_top_mm": mt,
        "margin_mode": str(CALIB_PAPER_MARGIN_OVERRIDE or CALIB_MARGIN_MODE),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if res["K1440"] is not None:
        npz.update({"K1440": res["K1440"], "dist": res["dist"], "K720": res["K720"]})
    np.savez("calibration_1440.npz", **npz)
    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "mode": "desk-only (单平面)" if res["K1440"] is None else "full",
        "square_mm_measured": CALIB_SQUARE_BIG_MM,
        "paper_margin_mm": {"left": ml, "top": mt},
        "paper_margin_mode": str(CALIB_PAPER_MARGIN_OVERRIDE or CALIB_MARGIN_MODE),
        "homography_residual_mm": {
            "rms": res["homo_res_rms_mm"],
            "max": res["homo_res_max_mm"],
        },
        "a4_quad_720p_px": res["quad720"].tolist(),
        "frames_dir": save_dir,
        "world_frame": "origin=纸左下角(页头朝上), x右0-210, y上0-297",
    }
    if res["K1440"] is not None:
        summary.update(
            {
                "rms_reproj_px": res["rms"],
                "desk_distance_mm": res["desk_dist_mm"],
                "camera_height_config_mm": CAMERA_HEIGHT_MM,
                "distortion_max_px_1440": res["distortion_max_px"],
            }
        )
    os.makedirs("out", exist_ok=True)
    with open("out/calib_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(
        "\n[保存] homography.npy (720p生产用), homography_1440.npy, "
        "calibration_1440.npz, out/calib_summary.json"
    )


def save_verification_images(desk, res):
    """锚点帧叠加A4面板存图, 供离线核对"""
    rec0 = desk[0]
    vis = rec0.frame.copy()
    draw_detection(vis, rec0.corners, rec0.ids)
    draw_a4_panel(vis, np.linalg.inv(res["H1440"]))
    p = f"out/calib_anchor_{time.strftime('%H%M%S')}.jpg"
    cv2.imwrite(p, vis)
    print(f"[保存] 锚点核对图 {p}")


# ---------------- 验证预览 ----------------


def verify_preview(backend):
    """实时预览 + 固定A4面板叠加 + 板上角点mm读数, q退出"""
    try:
        H1440 = np.load("homography_1440.npy")
    except OSError:
        print("[预览] 无 homography_1440.npy, 请先标定")
        return
    invH = np.linalg.inv(H1440)
    W, H = CALIB_IMAGE_SIZE
    cv2.namedWindow("verify", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("verify", 1280, 720)
    print("\n[验证] 实时预览: 绿框=A4面板, 角点标注世界mm坐标")
    print("  可用尺子在板上/桌面上比对; q退出")
    while True:
        frame = backend.read_rgb()
        if frame is None:
            time.sleep(0.2)
            continue
        if frame.shape[1] != W:
            frame = cv2.resize(frame, (W, H))
        corners, ids = detect(frame)
        vis = frame.copy()
        draw_detection(vis, corners, ids)
        draw_a4_panel(vis, invH)
        if corners is not None:
            pts = corners.reshape(-1, 2)
            world = cv2.perspectiveTransform(
                pts.reshape(-1, 1, 2).astype(np.float32), H1440.astype(np.float32)
            ).reshape(-1, 2)
            # 抽样标注几个角点的世界坐标
            for k in range(0, len(pts), max(1, len(pts) // 8)):
                x, y = pts[k]
                cv2.putText(
                    vis,
                    f"{world[k][0]:.0f},{world[k][1]:.0f}",
                    (int(x) + 8, int(y) + 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
        cv2.imshow("verify", cv2.resize(vis, (1280, 720)))
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyWindow("verify")


# ---------------- 离线帧复用 ----------------


def load_frames_from_dir(d):
    """复用 SAVE_CALIB_FRAMES 保存的帧 (desk_XX.jpg / hand_XX.jpg)"""
    import glob

    desk, hand = [], []
    for pat, lst in [("desk_*.jpg", desk), ("hand_*.jpg", hand)]:
        for f in sorted(glob.glob(os.path.join(d, pat))):
            frame = cv2.imread(f)
            if frame is None:
                continue
            corners, ids = detect(frame)
            if ids is None:
                print(f"  [警告] {f} 未检测到板, 跳过")
                continue
            lst.append(FrameRec(frame, corners, ids))
    print(f"[复用] {d}: desk={len(desk)} hand={len(hand)}")
    return desk, hand


# ---------------- 主流程 ----------------


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--camera",
        choices=["usb", "mjpeg"],
        default="usb",
        help="取帧通道: usb=MaixCam内置UVC(默认,生产环境) / mjpeg=HTTP快照(旧方式)",
    )
    ap.add_argument("--images", help="复用已存帧目录 (debug_imgs/calib_*)")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument(
        "--full-intrinsics",
        action="store_true",
        help="高级模式: 多角度重标镜头内参/畸变; 固定机位通常不需要",
    )
    mode.add_argument(
        "--desk-only", action="store_true", help="兼容旧命令; 单平面模式现已是默认行为"
    )
    ap.add_argument(
        "--preview-only",
        action="store_true",
        help="只做验证预览 (需已有 homography_1440.npy)",
    )
    args = ap.parse_args()
    desk_only = not args.full_intrinsics

    print("=" * 56)
    print("  ChArUco 固定工作区标定  (8x11, 格边实测 %.2fmm)" % CALIB_SQUARE_BIG_MM)
    print(
        "  模式: "
        + ("单帧平面（默认，无需多角度）" if desk_only else "完整内参（高级，多角度）")
    )
    if CALIB_PAPER_MARGIN_OVERRIDE is not None:
        ml, mt = CALIB_PAPER_MARGIN_OVERRIDE
        print(f"  页边距: 左 {ml:.2f} 上 {mt:.2f} mm [OVERRIDE 尺量值]")
    else:
        for m in ("center", "proportional"):
            l, t = paper_margins_mm(m)
            tag = " [采用]" if m == CALIB_MARGIN_MODE else " [备选]"
            print(f"  页边距假设[{m}]: 左 {l:.2f} 上 {t:.2f} mm{tag}")
        print("  预览: 实线绿=采用 虚线紫=备选; 看哪个贴住实际纸边")
    print("=" * 56)

    if args.images:
        desk, hand = load_frames_from_dir(args.images)
        if len(desk) < 1 or (not desk_only and len(desk) + len(hand) < 3):
            print("帧不足, 无法标定")
            return 2
        if desk_only:
            res = calibrate_desk_only(desk)
        else:
            res = calibrate(desk, hand)
        if res is None:
            return 3
        save_results(res, args.images)
        save_verification_images(desk, res)
        return 0

    if args.camera == "usb":
        global CALIB_IMAGE_SIZE
        CALIB_IMAGE_SIZE = MAIXCAM_UVC_SIZE
        print(
            f"[相机] MaixCam内置UVC: /dev/video{MAIXCAM_UVC_INDEX} "
            f"{MAIXCAM_UVC_SIZE[0]}x{MAIXCAM_UVC_SIZE[1]} (MJPG)"
        )
        from camera_backend import USBCameraBackend

        backend = USBCameraBackend(
            index=MAIXCAM_UVC_INDEX,
            width=MAIXCAM_UVC_SIZE[0],
            height=MAIXCAM_UVC_SIZE[1],
            mjpg=True,
        )
    else:
        from camera_backend import MjpegSnapshotBackend

        backend = MjpegSnapshotBackend(shot_url=MJPEG_SHOT1440_URL)
    if not backend.open():
        print("相机打开失败")
        return 1

    if args.preview_only:
        verify_preview(backend)
        backend.release()
        return 0

    save_dir = None
    if SAVE_CALIB_FRAMES:
        save_dir = f"debug_imgs/calib_{time.strftime('%m%d_%H%M%S')}"
        os.makedirs(save_dir, exist_ok=True)
        print(f"采集帧将保存到 {save_dir}")

    desk, hand = collect_frames(backend, save_dir, desk_only=desk_only)
    if desk is None:
        print("用户中止")
        backend.release()
        return 130

    if desk_only:
        res = calibrate_desk_only(desk)
    else:
        res = calibrate(desk, hand)
    if res is None:
        backend.release()
        return 3
    save_results(res, save_dir)
    save_verification_images(desk, res)

    print("\n进入验证预览 (可换放真实A4纸核对绿框)...")
    verify_preview(backend)
    backend.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
