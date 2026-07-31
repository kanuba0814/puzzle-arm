"""
脱机调试脚本: 视觉识别 + 拼图算法
不需要机械臂硬件，只需要图片或摄像头

用法:
  python debug_vision.py --image test.jpg    # 用图片测试
  python debug_vision.py --camera            # 用摄像头实时测试
  python debug_vision.py --mock              # 用模拟数据测试拼图算法
"""
import cv2
import numpy as np
import argparse
import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from vision import PuzzleVision
from puzzle_solver import PuzzleSolver
from config import *


def test_with_image(image_path, mode='color'):
    """用图片测试视觉检测"""
    print(f"\n{'='*50}")
    print(f"  图片测试: {image_path}")
    print(f"  模式: {mode}")
    print(f"{'='*50}")

    # 读取图片
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"无法读取图片: {image_path}")
        return

    print(f"图片尺寸: {frame.shape}")

    # 创建vision对象(不打开摄像头)
    vision = PuzzleVision.__new__(PuzzleVision)
    vision.cap = None
    vision.homography = None
    vision.calibration_points = []

    # 尝试加载标定
    vision.load_calibration()

    # 检测
    if mode == 'color':
        pieces = vision.detect_colored_pieces(frame)
    else:
        pieces = vision.detect_pieces_by_edge(frame)

    print(f"\n检测到 {len(pieces)} 个碎片:")
    for i, p in enumerate(pieces):
        cx, cy = p['center_world'] if vision.homography is not None else p['center_pixel']
        print(f"  碎片{i}: 中心=({cx:.1f}, {cy:.1f}), "
              f"角度={p['angle']:.1f}°", end='')
        if 'edge_lengths' in p:
            print(f", 边长={[f'{e:.1f}' for e in p['edge_lengths']]}", end='')
        if 'num_vertices' in p:
            print(f", 顶点数={p['num_vertices']}", end='')
        print()

    # 如果有标定，继续测试拼图
    if vision.homography is not None and pieces:
        test_puzzle(pieces, mode)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_with_camera(mode='color', show_depth=False):
    """用摄像头实时测试"""
    print(f"\n{'='*50}")
    print(f"  摄像头实时测试")
    print(f"  模式: {mode}, 深度图: {show_depth}")
    print(f"{'='*50}")

    vision = PuzzleVision()
    vision.load_calibration()

    print("\n按 'q' 退出, 'c' 切换模式, 'd' 显示深度图, 's' 拍照并分析")

    while True:
        frame = vision.capture_frame()
        if frame is None:
            print("摄像头读取失败")
            break

        if mode == 'color':
            pieces = vision.detect_colored_pieces(frame)
        else:
            pieces = vision.detect_pieces_by_edge(frame)

        print(f"\r检测到 {len(pieces)} 个碎片", end='', flush=True)

        if show_depth:
            depth = vision.capture_depth()
            if depth is not None:
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=255/1000),
                    cv2.COLORMAP_JET)
                cv2.imshow('Depth', depth_vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            mode = 'edge' if mode == 'color' else 'color'
            print(f"\n切换到 {mode} 模式")
        elif key == ord('d'):
            show_depth = not show_depth
            print(f"\n深度图显示: {show_depth}")
        elif key == ord('s'):
            # 拍照分析
            filename = f"capture_{int(cv2.getTickCount())}.jpg"
            cv2.imwrite(filename, frame)
            print(f"\n已保存: {filename}")
            if vision.homography is not None:
                test_puzzle(pieces, mode)

    vision.release()


def test_mock():
    """用模拟数据测试拼图算法"""
    print(f"\n{'='*50}")
    print(f"  模拟数据测试拼图算法")
    print(f"{'='*50}")

    solver = PuzzleSolver()

    # === 测试第1项 ===
    print("\n--- 第1项: 自备碎片 ---")
    mock_pieces_task1 = [
        {'center_world': (50, 30), 'angle': 45},
        {'center_world': (150, 20), 'angle': 90},
        {'center_world': (80, 100), 'angle': 0},
        {'center_world': (180, 80), 'angle': 30},
    ]
    print(f"输入碎片: {len(mock_pieces_task1)}个")
    for i, p in enumerate(mock_pieces_task1):
        print(f"  碎片{i}: ({p['center_world'][0]:.0f},{p['center_world'][1]:.0f}) "
              f"angle={p['angle']:.0f}°")

    tasks = solver.solve_task1(mock_pieces_task1)
    tasks = solver.optimize_order(tasks)

    print(f"\n输出任务: {len(tasks)}个")
    for i, t in enumerate(tasks):
        print(f"  任务{i}: 拾取({t['pick_pos'][0]:.0f},{t['pick_pos'][1]:.0f}) "
              f"{t['pick_angle']:.0f}° -> "
              f"放置({t['place_pos'][0]:.0f},{t['place_pos'][1]:.0f}) "
              f"{t['place_angle']:.0f}°")

    # === 测试第2项 ===
    print("\n--- 第2项: 现场碎片 ---")
    # 模拟一个9cm×5cm的矩形被切成3片
    mock_pieces_task2 = [
        {
            'center_world': (50, 30), 'angle': 0,
            'vertices': [(0, 0), (30, 0), (30, 50), (0, 50)],
            'edge_lengths': [30, 50, 30, 50],
        },
        {
            'center_world': (150, 30), 'angle': 0,
            'vertices': [(0, 0), (30, 0), (30, 50), (0, 50)],
            'edge_lengths': [30, 50, 30, 50],
        },
        {
            'center_world': (100, 120), 'angle': 0,
            'vertices': [(0, 0), (30, 0), (30, 30), (0, 30)],
            'edge_lengths': [30, 30, 30, 30],
        },
    ]
    print(f"输入碎片: {len(mock_pieces_task2)}个")
    for i, p in enumerate(mock_pieces_task2):
        print(f"  碎片{i}: ({p['center_world'][0]:.0f},{p['center_world'][1]:.0f}) "
              f"顶点数={len(p['vertices'])} "
              f"边长={[f'{e:.0f}' for e in p['edge_lengths']]}")

    tasks2 = solver.solve_task2(mock_pieces_task2)
    print(f"\n输出任务: {len(tasks2)}个")
    for i, t in enumerate(tasks2):
        print(f"  任务{i}: 拾取({t['pick_pos'][0]:.0f},{t['pick_pos'][1]:.0f}) "
              f"-> 放置({t['place_pos'][0]:.0f},{t['place_pos'][1]:.0f})")


def test_puzzle(pieces, mode):
    """对检测结果运行拼图算法"""
    print("\n--- 拼图求解 ---")
    solver = PuzzleSolver()

    if mode == 'color':
        tasks = solver.solve_task1(pieces)
    else:
        tasks = solver.solve_task2(pieces)

    tasks = solver.optimize_order(tasks)

    print(f"生成 {len(tasks)} 个任务:")
    for i, t in enumerate(tasks):
        print(f"  任务{i}: 拾取({t['pick_pos'][0]:.0f},{t['pick_pos'][1]:.0f}) "
              f"-> 放置({t['place_pos'][0]:.0f},{t['place_pos'][1]:.0f})")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='视觉+拼图脱机调试')
    parser.add_argument('--image', type=str, help='图片路径')
    parser.add_argument('--camera', action='store_true', help='使用摄像头')
    parser.add_argument('--mock', action='store_true', help='使用模拟数据')
    parser.add_argument('--mode', type=str, default='color',
                        choices=['color', 'edge'], help='检测模式')
    parser.add_argument('--rgbd', action='store_true',
                        help='实时显示深度图 (仅 Astra 相机有效)')
    args = parser.parse_args()

    if args.mock:
        test_mock()
    elif args.image:
        test_with_image(args.image, args.mode)
    elif args.camera:
        test_with_camera(args.mode, args.rgbd)
    else:
        print("请指定测试模式:")
        print("  python debug_vision.py --mock              # 模拟数据测试")
        print("  python debug_vision.py --image test.jpg    # 图片测试")
        print("  python debug_vision.py --camera            # 摄像头测试")
        print("  python debug_vision.py --camera --rgbd     # 同时显示深度图")
        print("  python debug_vision.py --image test.jpg --mode edge  # 边缘检测模式")
