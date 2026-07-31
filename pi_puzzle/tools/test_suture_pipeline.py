"""缝合流水线验证 (宿主离线, 不碰机械臂/相机)。

覆盖:
1. 合成场景端到端: 检测(123) + 求解(AssemblySolver) + task1 10×6 校验 + 放置任务
2. main.py 缝合版 _detect/_solve 集成 (mock 相机后端, 不执行机械臂)

用法:
    conda run -n yolo python tools/test_suture_pipeline.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import cv2  # noqa: E402

from config import ASSEMBLY_TARGET_X, ASSEMBLY_TARGET_Y, TASK1_RECT_TOL_MM  # noqa: E402
from rect_assembly import AssemblySolver  # noqa: E402
from solve_paths import build_tasks, compute_placement, optimize_order  # noqa: E402
from vision import PuzzleVision  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SYNTH = os.path.join(ROOT, "out", "synth_suture.jpg")


def make_synth(dark=False):
    """生成合成场景 (100×65 矩形切 4 片)"""
    sys.argv = ["make_synthetic_scene.py", "--out", SYNTH]
    if dark:
        sys.argv += ["--dark"]
    import runpy

    runpy.run_path(os.path.join(ROOT, "tools", "make_synthetic_scene.py"), run_name="__main__")
    return cv2.imread(SYNTH)


class MockBackend:
    """相机后端 mock: 固定返回合成图"""

    def __init__(self, path):
        self._frame = cv2.imread(path)
        self.cap = None

    def read_rgb(self):
        return self._frame.copy()

    def read_depth(self):
        return None

    def release(self):
        pass


class MockArm:
    """机械臂 mock: 记录调用, 不动作"""

    def __init__(self):
        self.calls = []

    def go_home(self):
        self.calls.append("go_home")

    def wait_for_stop(self):
        self.calls.append("wait_for_stop")

    def magnet_off(self):
        self.calls.append("magnet_off")

    def close(self):
        self.calls.append("close")


def build_homography(v, frame):
    """用给定 vision 实例从合成图重建像素->世界单应 (模拟离线无标定单应路径)"""
    quad = v.auto_detect_paper(frame)
    assert quad is not None, "A4 纸检测失败"
    v.set_paper_homography(quad)
    return v.homography, quad


def check_tasks(tasks, label, expect_rect, allow_approximate=True):
    assert tasks, f"{label}: 任务为空"
    assert len(tasks) == 4, f"{label}: 任务数 {len(tasks)} != 4"
    for i, t in enumerate(tasks):
        px, py = t["pick_pos"]
        qx, qy = t["place_pos"]
        assert 0 <= px <= 210 and 0 <= py <= 297, f"{label}: pick 越界 {t}"
        assert 0 <= qx <= 210 and 0 <= qy <= 297, f"{label}: place 越界 {t}"
        # 放置目标必须在 A4 下半区 (分界线 y=148.5 之下)
        assert qy < 148.5, f"{label}: place 不在下半区 {t}"
        assert all(np.isfinite(v) for v in (px, py, qx, qy, t["pick_angle"], t["place_angle"]))
    if expect_rect is not None:
        w, h = tasks[0]["rect_mm"]
        assert abs(w - expect_rect[0]) <= TASK1_RECT_TOL_MM, (
            f"{label}: 矩形宽 {w} != {expect_rect[0]}±{TASK1_RECT_TOL_MM}"
        )
        assert abs(h - expect_rect[1]) <= TASK1_RECT_TOL_MM, (
            f"{label}: 矩形高 {h} != {expect_rect[1]}±{TASK1_RECT_TOL_MM}"
        )
    print(f"[OK] {label}: {len(tasks)} 任务, 首任务 place=({tasks[0]['place_pos'][0]:.1f},"
          f"{tasks[0]['place_pos'][1]:.1f}), rect={tasks[0]['rect_mm']}")


def test_pipeline_offline(frame):
    """离线端到端: 检测 + 求解 + task1 校验 + 任务生成"""
    v = PuzzleVision.__new__(PuzzleVision)  # 跳过 __init__: 不创建真实相机后端
    v.homography = None
    H, quad = build_homography(v, frame)

    # 检测 (flat_th 扫描, 同缝合版 _detect)
    pieces_best = None
    for fth in (0.82, 0.78, 0.86):
        pieces = v.detect_pieces_arbitrary(frame, quad, flat_th=fth)
        if pieces_best is None or len(pieces) > len(pieces_best):
            pieces_best = pieces
    assert pieces_best and len(pieces_best) == 4, (
        f"检测失败: {len(pieces_best) if pieces_best else 0} 片"
    )
    areas = [f"{p['area_mm2']:.0f}" for p in pieces_best]
    print(f"[OK] 检测: 4 片, 面积={areas}mm²")
    # 求解
    sol = AssemblySolver(verbose=False).solve(
        [p["vertices"] for p in pieces_best]
    )
    assert sol is not None, "求解失败"
    w, h = sol["rect_size"]
    print(f"[OK] 求解: 矩形 {w:.1f}x{h:.1f}mm 评分{sol['score']:.2f}"
          f"{' [近似]' if sol.get('approximate') else ''}")
    # 合成真值 100×65
    assert abs(w - 100) <= 15 and abs(h - 65) <= 15, f"反推矩形 {w}x{h} 偏差过大"

    # task1 校验 (10×6=100×60, 容差15)
    from main import PuzzleRobot

    assert PuzzleRobot._matches_task1_rect(sol), "task1 10×6 校验未通过"

    # 放置任务
    T_place = compute_placement(sol)
    raw = build_tasks(pieces_best, sol, T_place)
    raw = optimize_order(raw)
    tasks = []
    for t in raw:
        tasks.append(
            {
                "pick_pos": (float(t["pick"]["x"]), float(t["pick"]["y"])),
                "pick_angle": float(t["_pick_angle"]),
                "place_pos": (float(t["place"]["x"]), float(t["place"]["y"])),
                "place_angle": float(t["rotation_deg"]),
                "id": t["id"],
                "rect_mm": [float(w), float(h)],
            }
        )
    check_tasks(tasks, "离线端到端 task1", expect_rect=(100, 65))

    # 放置中心应接近下半区中心
    cx = np.mean([t["place_pos"][0] for t in tasks])
    cy = np.mean([t["place_pos"][1] for t in tasks])
    assert abs(cx - ASSEMBLY_TARGET_X) < 30, f"放置中心 x={cx} 偏离 {ASSEMBLY_TARGET_X}"
    assert abs(cy - ASSEMBLY_TARGET_Y) < 30, f"放置中心 y={cy} 偏离 {ASSEMBLY_TARGET_Y}"
    print(f"[OK] 放置中心 ({cx:.1f},{cy:.1f}) ≈ ({ASSEMBLY_TARGET_X},{ASSEMBLY_TARGET_Y})")
    return H


def test_main_detect_solve(frame, H):
    """main.py 缝合版 _detect/_solve 集成 (mock 后端)"""
    import main as main_mod

    # main.py 内是 `from camera_backend import create_camera_backend` (名字绑定),
    # 必须 patch main 模块的全局名, 否则会打开真实 USB 摄像头
    main_mod.create_camera_backend = lambda camera_type=None, **kw: MockBackend(SYNTH)
    from main import PuzzleRobot

    robot = None
    try:
        robot = PuzzleRobot(arm=MockArm(), camera_type="usb")
        robot.vision.homography = H

        detection = robot._detect("task1")
        assert detection is not None, "_detect 返回 None"
        frame_out, quad, candidates = detection
        assert len(candidates) >= 1, "无检测候选"
        assert robot.detected_count == 4, f"检测数 {robot.detected_count} != 4"
        print(f"[OK] _detect: {len(candidates)} 组候选, 投票 {robot.detected_count} 片")

        tasks = robot._solve("task1", detection)
        assert tasks, "_solve 返回空"
        check_tasks(tasks, "main._detect/_solve task1", expect_rect=(100, 65))
        assert robot.solve_meta["pieces"] == 4
        print(f"[OK] solve_meta: {robot.solve_meta}")

        # task2 不校验目标矩形
        detection2 = robot._detect("task2")
        tasks2 = robot._solve("task2", detection2)
        check_tasks(tasks2, "main._detect/_solve task2", expect_rect=None)
        print(f"[OK] task2 (无矩形校验) 通过")
    finally:
        robot.cleanup()


if __name__ == "__main__":
    print("=" * 60)
    print("缝合流水线验证")
    print("=" * 60)
    frame = make_synth(dark=False)
    H = test_pipeline_offline(frame)
    test_main_detect_solve(frame, H)

    print("\n--- 暗光场景 ---")
    frame_dark = make_synth(dark=True)
    test_pipeline_offline(frame_dark)
    test_main_detect_solve(frame_dark, H)

    print("\n全部通过 ✅")
