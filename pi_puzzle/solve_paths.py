"""
当前阶段主流程: 任意形状碎片 -> 反推矩形 -> 机械臂直线路径 (计算+可视化)

流程:
    取帧 -> A4纸自动检测+单应标定 -> 碎片多边形检测(mm)
    -> AssemblySolver 反推拼接矩形 -> 目标布局放入下半区
    -> 每片: 抓取点/放置点/旋转角/直线路径 -> 贪心排序
    -> 输出 paths.json + 标注图 + 俯视规划图

用法:
    python3 solve_paths.py                  # MaixCam 实拍 (优先加载 homography.npy 标定)
    python3 solve_paths.py --image xx.jpg   # 离线图片测试
    python3 solve_paths.py --frames 5       # 多帧投票(默认5)
    python3 solve_paths.py --no-homography  # 不用标定文件, 回退到每次检测纸边缘
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

from config import (
    A4_HEIGHT,
    A4_WIDTH,
    ARM_BASE_X,
    ARM_BASE_Y,
    ASSEMBLY_TARGET_X,
    ASSEMBLY_TARGET_Y,
    DIVIDER_LINE_Y,
    MAX_PIECES,
)
from rect_assembly import (
    AssemblySolver,
    apply_transform,
    compose,
    make_transform,
    transform_angle,
)
from vision import PuzzleVision

OUT_DIR = "out"


def get_frame(vision, args):
    """获取一帧 (离线图片或相机)"""
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"[Main] 图片读取失败: {args.image}")
        return frame
    return vision.capture_frame()


def detect_with_voting(vision, args, fixed_quad=None):
    """
    多帧投票检测: 取N帧, 以碎片数的众数为准, 返回
    (frame, quad, pieces)。处理单帧检测抖动。
    fixed_quad: 由标定单应反推的固定A4四角; 给定后不再每帧检测纸边缘
    """
    results = []
    n_frames = 1 if args.image else args.frames
    for k in range(n_frames):
        frame = get_frame(vision, args)
        if frame is None:
            continue
        frame = vision.preprocess(frame)  # 统一预处理, 保证坐标系一致
        # 暗光场景: 非局部均值去噪 (FPN噪声是检测精度杀手, 0.4s可接受)
        if frame.mean() < 100:
            frame = cv2.fastNlMeansDenoisingColored(frame, None, 15, 15, 7, 21)
        if fixed_quad is not None:
            quad = fixed_quad
        else:
            quad = vision.auto_detect_paper(frame)
            if quad is None:
                continue
            vision.set_paper_homography(quad)
        pieces = vision.detect_pieces_arbitrary(frame, quad)
        results.append((frame, quad, pieces))
        if n_frames > 1:
            time.sleep(0.15)

    if not results:
        return None, None, None

    # 碎片数众数投票
    counts = [len(r[2]) for r in results]
    best_count = max(set(counts), key=counts.count)
    candidates = [r for r in results if len(r[2]) == best_count]
    frame, quad, pieces = candidates[len(candidates) // 2]
    if len(set(counts)) > 1:
        print(f"[Main] 各帧碎片数 {counts}, 采用 {best_count}")
    return frame, quad, pieces


def fix_paper_orientation(vision, quad, pieces):
    """
    保证碎片位于世界系上半区 (y > 148.5)。
    单应矩阵初建时y方向任意, 若碎片中位数在下半区则翻转y轴重建。
    """
    if not pieces:
        return pieces
    ys = sorted(p["centroid"][1] for p in pieces)
    median_y = ys[len(ys) // 2]
    if median_y < DIVIDER_LINE_Y:
        vision.set_paper_homography(quad, flip_y=True)
        # flip_y 等价于世界坐标 (x, y) -> (x, 297-y), 直接解析镜像即可
        for p in pieces:
            p["vertices"][:, 1] = A4_HEIGHT - p["vertices"][:, 1]
            p["vertices"] = p["vertices"][::-1].copy()  # 镜像后保持CCW
            c = p["vertices"].mean(axis=0)
            p["centroid"] = (float(c[0]), float(c[1]))
            gx, gy = p["grip_point"]
            p["grip_point"] = (gx, A4_HEIGHT - gy)
        print("[Main] 世界系y轴已翻转 (碎片在上半区)")
    return pieces


def compute_placement(solution):
    """
    把拼接解(任意朝向/位置)平移旋转到下半区中心:
    矩形长边对齐x轴, 中心 = (ASSEMBLY_TARGET_X, ASSEMBLY_TARGET_Y)
    返回放置变换 (R, t)
    """
    corners = solution["rect_corners"]
    center = corners.mean(axis=0)
    # 长边方向
    e0 = corners[1] - corners[0]
    e1 = corners[2] - corners[1]
    long_edge = e0 if np.linalg.norm(e0) >= np.linalg.norm(e1) else e1
    phi = np.arctan2(long_edge[1], long_edge[0])
    # 让长边方向落在 [-90, 90) 避免无谓的180度翻转
    if phi >= np.pi / 2:
        phi -= np.pi
    elif phi < -np.pi / 2:
        phi += np.pi
    R, _ = make_transform(-phi, [0, 0])
    t = np.array([ASSEMBLY_TARGET_X, ASSEMBLY_TARGET_Y]) - R @ center
    return (R, t)


def build_tasks(pieces, solution, T_place):
    """生成每片的 抓取/放置/旋转/路径 任务"""
    tasks = []
    for i, p in enumerate(pieces):
        T_final = compose(T_place, solution["transforms"][i])
        grip = np.array(p["grip_point"])
        place_pt = apply_transform(T_final, grip.reshape(1, 2))[0]
        rotation = transform_angle(T_final)
        rotation = ((rotation + 180) % 360) - 180
        target_verts = apply_transform(T_final, p["vertices"])
        tasks.append(
            {
                "id": p["id"],
                "area_mm2": p["area_mm2"],
                "pick": {"x": float(grip[0]), "y": float(grip[1])},
                "place": {"x": float(place_pt[0]), "y": float(place_pt[1])},
                "rotation_deg": float(rotation),
                "path_mm": [
                    [float(grip[0]), float(grip[1])],
                    [float(place_pt[0]), float(place_pt[1])],
                ],
                "_target_verts": target_verts,  # 内部可视化用
                "_pick_angle": p["angle_deg"],
            }
        )
    return tasks


def optimize_order(tasks, start=(ARM_BASE_X, ARM_BASE_Y)):
    """贪心最近邻排序: 从机械臂基座出发最小化空驶距离"""
    remaining = list(tasks)
    ordered = []
    cur = np.array(start, dtype=float)
    while remaining:
        d = [
            np.linalg.norm(np.array([t["pick"]["x"], t["pick"]["y"]]) - cur)
            for t in remaining
        ]
        idx = int(np.argmin(d))
        t = remaining.pop(idx)
        ordered.append(t)
        cur = np.array([t["place"]["x"], t["place"]["y"]])
    return ordered


# ==================== 可视化 ====================


def draw_annotated(frame, quad, pieces, tasks, T_finals, invH):
    """在原图上叠加: 纸框/检测碎片/目标位置/路径箭头"""
    img = frame.copy()
    cv2.polylines(img, [quad.astype(np.int32)], True, (255, 200, 0), 2)

    def w2px(pt):
        p = np.array([[pt]], dtype=np.float32).reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(p, invH)
        return tuple(out.reshape(2).astype(int))

    for p in pieces:
        cnt = p["contour_px"]
        cv2.drawContours(img, [cnt], 0, (0, 255, 0), 2)
        gx, gy = p["grip_point"]
        cx, cy = w2px((gx, gy))
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(
            img,
            f"#{p['id']}",
            (cx + 8, cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    for t in tasks:
        # 目标多边形 (蓝色)
        tv = t["_target_verts"]
        px = [w2px(v) for v in tv]
        cv2.polylines(img, [np.array(px, np.int32)], True, (255, 0, 0), 2)
        # 路径箭头 (红)
        p0 = w2px((t["pick"]["x"], t["pick"]["y"]))
        p1 = w2px((t["place"]["x"], t["place"]["y"]))
        cv2.arrowedLine(img, p0, p1, (0, 0, 255), 2, tipLength=0.05)
        cv2.circle(img, p1, 5, (255, 0, 255), -1)
        label = f"#{t['id']} r={t['rotation_deg']:.0f}deg"
        cv2.putText(
            img,
            label,
            (p1[0] + 8, p1[1] + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )
    return img


def draw_topdown(pieces, tasks, rect_corners_world, px_per_mm=3):
    """俯视规划图: 世界坐标系全貌 (当前位置 + 目标布局 + 路径)"""
    W = int(A4_WIDTH * px_per_mm)
    H = int(A4_HEIGHT * px_per_mm)
    img = np.ones((H, W, 3), np.uint8) * 245

    def w2px(x, y):
        return (int(x * px_per_mm), int((A4_HEIGHT - y) * px_per_mm))

    # A4边界 + 分界线
    cv2.rectangle(img, w2px(0, A4_HEIGHT), w2px(A4_WIDTH, 0), (0, 0, 0), 2)
    cv2.line(
        img, w2px(0, DIVIDER_LINE_Y), w2px(A4_WIDTH, DIVIDER_LINE_Y), (128, 128, 128), 1
    )
    cv2.putText(
        img,
        "divider",
        w2px(3, DIVIDER_LINE_Y + 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (128, 128, 128),
        1,
    )
    # 机械臂基座
    base = w2px(ARM_BASE_X, ARM_BASE_Y)
    cv2.circle(img, base, 8, (0, 0, 0), 2)
    cv2.putText(
        img,
        "ARM",
        (base[0] - 15, base[1] + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 0, 0),
        2,
    )

    # 当前碎片 (红)
    for p in pieces:
        pts = np.array([w2px(v[0], v[1]) for v in p["vertices"]], np.int32)
        cv2.polylines(img, [pts], True, (0, 0, 255), 2)
        g = w2px(*p["grip_point"])
        cv2.circle(img, g, 4, (0, 0, 255), -1)
        cv2.putText(
            img,
            f"#{p['id']}",
            (g[0] + 6, g[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2,
        )

    # 目标矩形 (蓝)
    rc = np.array([w2px(v[0], v[1]) for v in rect_corners_world], np.int32)
    cv2.polylines(img, [rc], True, (255, 0, 0), 1)

    # 目标碎片 (绿) + 路径箭头
    for order, t in enumerate(tasks):
        tv = np.array([w2px(v[0], v[1]) for v in t["_target_verts"]], np.int32)
        cv2.polylines(img, [tv], True, (0, 180, 0), 2)
        p0 = w2px(t["pick"]["x"], t["pick"]["y"])
        p1 = w2px(t["place"]["x"], t["place"]["y"])
        cv2.arrowedLine(img, p0, p1, (200, 0, 200), 2, tipLength=0.04)
        mid = ((p0[0] + p1[0]) // 2, (p0[1] + p1[1]) // 2)
        cv2.putText(
            img, str(order + 1), mid, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 0, 200), 2
        )
        cv2.putText(
            img,
            f"r={t['rotation_deg']:.0f}",
            (p1[0] + 5, p1[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 150, 0),
            1,
        )
    return img


def draw_failure_diagnostic(frame, quad, pieces, message, invH):
    """求解失败时仍输出检测诊断图，便于判断粘连/漏检/标定问题。"""
    img = frame.copy()
    cv2.polylines(img, [quad.astype(np.int32)], True, (255, 200, 0), 2)

    def w2px(pt):
        p = np.array([pt[0], pt[1], 1.0])
        q = invH @ p
        return (q[:2] / q[2]).astype(np.int32)

    for p in pieces:
        pts = np.array([w2px(v) for v in p["vertices"]])
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        center = w2px(p["centroid"])
        cv2.putText(
            img,
            f"#{p['id']} {p['area_mm2']:.0f}mm2",
            tuple(center),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
        )
    cv2.rectangle(img, (0, 0), (img.shape[1], 54), (30, 30, 30), -1)
    cv2.putText(
        img,
        message,
        (12, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 80, 255),
        2,
    )
    return img


# ==================== 主流程 ====================


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", help="离线图片路径 (不用相机)")
    parser.add_argument("--frames", type=int, default=5, help="多帧投票数")
    parser.add_argument(
        "--expected-pieces", type=int, default=0, help="期望碎片数；0=自动接受检测数量"
    )
    parser.add_argument("--show", action="store_true", help="显示窗口")
    parser.add_argument(
        "--homography",
        default="homography.npy",
        help="标定单应文件 (calibrate_camera.py 产物)",
    )
    parser.add_argument(
        "--no-homography", action="store_true", help="强制回退到每次检测纸边缘的旧行为"
    )
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    vision = PuzzleVision()

    # 相机模式默认加载标定单应 (解决纸边缘检测不稳定导致的坐标错误);
    # 离线图片分辨率/机位未知, 仅在显式指定时使用
    fixed_quad = None
    use_file_H = (not args.no_homography) and (
        not args.image or args.homography != "homography.npy"
    )
    if use_file_H and vision.load_calibration(args.homography):
        fixed_quad = vision.quad_from_homography()
        print(f"[Main] 使用标定单应 {args.homography}, 固定A4面板")
    elif not args.image:
        print(
            "[Main] [警告] 无标定单应, 回退到纸边缘检测 (建议先运行 "
            "calibrate_camera.py)"
        )

    # 1. 多帧检测
    frame, quad, pieces = detect_with_voting(vision, args, fixed_quad)
    if frame is None or quad is None:
        print("[Main] A4纸检测失败")
        vision.release()
        return 1
    if not pieces:
        print("[Main] 未检测到碎片")
        vision.release()
        return 1
    pieces0 = pieces
    print(f"[Main] 检测到 {len(pieces0)} 片碎片")
    for p in pieces0:
        print(
            f"  #{p['id']}: {len(p['vertices'])}顶点 "
            f"边长={[f'{e:.1f}' for e in p['edge_lengths']]} "
            f"面积={p['area_mm2']:.0f}mm² 中心=({p['centroid'][0]:.0f},"
            f"{p['centroid'][1]:.0f})"
        )

    if len(pieces0) > MAX_PIECES:
        print(f"[Main] 警告: 碎片数超过题目上限 {MAX_PIECES}")

    # 2. 保证碎片在上半区 (标定单应下世界系固定, 无需重建)
    if fixed_quad is None:
        vision.set_paper_homography(quad)

    # 3. 拼图求解: 扫描检测阈值, 噪声大时单一参数可能不自洽,
    #    取求解评分最优的一组 (评分=矩形角度/边长/面积偏差)
    FLAT_TH_SWEEP = (0.82, 0.78, 0.86, 0.74, 0.90)
    solution = None
    pieces = None
    best_score = float("inf")
    n_target = args.expected_pieces or len(pieces0)
    if args.expected_pieces and len(pieces0) != args.expected_pieces:
        print(
            f"[Main] 警告: 检测到{len(pieces0)}片，"
            f"期望{args.expected_pieces}片；将继续尝试清晰轮廓模式"
        )
    prev_sig = None
    for fth in FLAT_TH_SWEEP:
        pieces_f = vision.detect_pieces_arbitrary(frame, quad, flat_th=fth)
        # 亮光模式下flat_th不起作用, 检测结果与上次相同则跳过
        sig = [p["vertices"].tobytes() for p in pieces_f]
        if sig == prev_sig:
            continue
        prev_sig = sig
        if len(pieces_f) != n_target:
            continue
        if fixed_quad is None:
            pieces_f = fix_paper_orientation(vision, quad, pieces_f)
        sol = AssemblySolver(verbose=False).solve([p["vertices"] for p in pieces_f])
        if sol is not None and sol.get("score", 999) < best_score:
            best_score = sol.get("score", 999)
            solution, pieces = sol, pieces_f
            print(
                f"[Main] flat_th={fth}: 可解 "
                f"{sol['rect_size'][0]:.1f}x{sol['rect_size'][1]:.1f}mm "
                f"评分{best_score:.2f}"
            )
        else:
            print(f"[Main] flat_th={fth}: 无解")
    # 黑底UVC扑克牌碎片: 默认可靠平滑/边缘精化适合噪声场景,
    # 但可能把扑克牌碎片的尖角和短边拟合坏。若第一遍只得到
    # 近似解或无解, 用“清晰轮廓模式”保留原始尖角再解一次。
    if solution is None or solution.get("approximate"):
        # detect_with_voting 返回的 frame 已经过 preprocess；这里重新取原图，
        # 让 detect_pieces_arbitrary 只做一次中值滤波，避免二次模糊吞掉尖角。
        frame_sharp = cv2.imread(args.image) if args.image else vision.capture_frame()
        pieces_sharp = (
            []
            if frame_sharp is None
            else vision.detect_pieces_arbitrary(
                frame_sharp,
                quad,
                flat_th=0.82,
                smooth_mask=False,
                refine_edges=False,
            )
        )
        if len(pieces_sharp) >= 2 and (
            not args.expected_pieces or len(pieces_sharp) == args.expected_pieces
        ):
            if fixed_quad is None:
                pieces_sharp = fix_paper_orientation(vision, quad, pieces_sharp)
            sol_sharp = AssemblySolver(verbose=False).solve(
                [p["vertices"] for p in pieces_sharp]
            )
            if sol_sharp is not None:
                sharp_score = sol_sharp.get("score", 999)
                replace = solution is None
                if solution is not None:
                    if (
                        solution.get("approximate")
                        and not sol_sharp.get("approximate")
                        or (
                            solution.get("approximate") == sol_sharp.get("approximate")
                            and sharp_score < best_score
                        )
                    ):
                        replace = True
                if replace:
                    best_score = sharp_score
                    solution, pieces = sol_sharp, pieces_sharp
                    print(
                        f"[Main] 清晰轮廓模式: 可解 "
                        f"{sol_sharp['rect_size'][0]:.1f}x"
                        f"{sol_sharp['rect_size'][1]:.1f}mm "
                        f"评分{best_score:.2f}"
                        f"{' (近似)' if sol_sharp.get('approximate') else ''}"
                    )

    # 兜底1: 片数与投票不一致的参数组合也试 (投票本身可能错)
    if solution is None:
        for fth in FLAT_TH_SWEEP:
            pieces_f = vision.detect_pieces_arbitrary(frame, quad, flat_th=fth)
            if len(pieces_f) == n_target or len(pieces_f) < 2:
                continue
            if fixed_quad is None:
                pieces_f = fix_paper_orientation(vision, quad, pieces_f)
            sol = AssemblySolver(verbose=False).solve([p["vertices"] for p in pieces_f])
            if sol is not None and sol.get("score", 999) < best_score:
                best_score = sol.get("score", 999)
                solution, pieces = sol, pieces_f
                print(
                    f"[Main] flat_th={fth} (片数{len(pieces_f)}): "
                    f"可解 {sol['rect_size'][0]:.1f}x"
                    f"{sol['rect_size'][1]:.1f}mm 评分{best_score:.2f}"
                )

    # 兜底2: 宽松模式 (暗光检测误差大: 边容差8mm, 面积余量30%,
    #        允许近似解 — 比赛按相邻顶点距离计分, 近似拼接也能得分)
    if solution is None:
        import rect_assembly as _ra

        saved = (_ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG)
        _ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG = (
            0.12,
            12.0,
            15.0,
        )
        try:
            pieces_f = vision.detect_pieces_arbitrary(frame, quad)
            if len(pieces_f) == n_target:
                if fixed_quad is None:
                    pieces_f = fix_paper_orientation(vision, quad, pieces_f)
                sv = AssemblySolver(edge_tol=8.0, verbose=False)
                sv.fit_margin = 0.30
                sol = sv.solve([p["vertices"] for p in pieces_f])
                if sol is not None:
                    solution, pieces = sol, pieces_f
                    best_score = sol.get("score", 999)
                    print(
                        f"[Main] 宽松模式: {sol['rect_size'][0]:.1f}x"
                        f"{sol['rect_size'][1]:.1f}mm "
                        f"评分{best_score:.2f}"
                        f"{' (近似)' if sol.get('approximate') else ''}"
                    )
        finally:
            (_ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG) = saved

    if solution is None:
        print("[Main] 拼图求解失败")
        diag_msg = f"检测{len(pieces0)}/{n_target}片，无法生成安全路径"
        diag = draw_failure_diagnostic(
            frame,
            quad,
            pieces0,
            diag_msg,
            np.linalg.inv(vision.homography),
        )
        diag_path = os.path.join(OUT_DIR, "diagnostic.jpg")
        cv2.imwrite(diag_path, diag)
        print(f"[Main] 诊断图已保存: {diag_path}")
        vision.release()
        return 2
    W, H = solution["rect_size"]
    print(
        f"[Main] 目标矩形 {W:.1f} x {H:.1f} mm"
        f"{' [近似解]' if solution.get('approximate') else ''}"
    )

    # 4. 放置到下半区 + 生成任务
    T_place = compute_placement(solution)
    tasks = build_tasks(pieces, solution, T_place)
    tasks = optimize_order(tasks)

    print("[Main] 执行序列 (已按路径优化排序):")
    for k, t in enumerate(tasks):
        print(
            f"  {k + 1}. 片#{t['id']}: "
            f"抓({t['pick']['x']:.1f},{t['pick']['y']:.1f}) -> "
            f"放({t['place']['x']:.1f},{t['place']['y']:.1f}) "
            f"旋转{t['rotation_deg']:+.1f}°"
        )

    # 5. 输出 paths.json
    invH = np.linalg.inv(vision.homography)
    rect_world = apply_transform(T_place, solution["rect_corners"])
    result = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "approximate": bool(solution.get("approximate", False)),
        "score": float(best_score),
        "rect": {
            "width_mm": float(W),
            "height_mm": float(H),
            "corners_mm": rect_world.tolist(),
            "center_mm": [ASSEMBLY_TARGET_X, ASSEMBLY_TARGET_Y],
        },
        "pieces": [
            {
                "id": t["id"],
                "area_mm2": t["area_mm2"],
                "pick": t["pick"],
                "place": t["place"],
                "rotation_deg": t["rotation_deg"],
                "path_mm": t["path_mm"],
            }
            for t in tasks
        ],
        "order": [t["id"] for t in tasks],
        "arm_base": {"x": ARM_BASE_X, "y": ARM_BASE_Y},
        "coordinate": "world mm, 原点A4角, x短边y长边, 上半区=碎片源区",
        "detected_count": len(pieces),
        "expected_count": args.expected_pieces or len(pieces),
    }
    json_path = os.path.join(OUT_DIR, "paths.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"[Main] 路径已保存: {json_path}")

    # 6. 可视化
    T_finals = [compose(T_place, solution["transforms"][i]) for i in range(len(pieces))]
    annotated = draw_annotated(frame, quad, pieces, tasks, T_finals, invH)
    topdown = draw_topdown(pieces, tasks, rect_world)
    p1 = os.path.join(OUT_DIR, "annotated.jpg")
    p2 = os.path.join(OUT_DIR, "plan_topdown.jpg")
    cv2.imwrite(p1, annotated)
    cv2.imwrite(p2, topdown)
    print(f"[Main] 可视化已保存: {p1}, {p2}")

    if args.show:
        cv2.imshow("annotated", annotated)
        cv2.imshow("topdown", topdown)
        cv2.waitKey(0)

    vision.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
