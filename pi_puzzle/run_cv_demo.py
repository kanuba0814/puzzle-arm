"""
本地视觉主控: 标定 -> CV识别 -> 下发树莓派模拟搬运

流程:
  1. 记录: 读Pi端当前舵机位置 + 拍一张现场照片存档
  2. 驻停: 机械臂保持/回到不遮挡相机的位置 (go_home)
  3. 标定: 加载 homography.npy (本地标定算法产物), 失败则自动检测A4纸重标
  4. 识别: CV检测碎片 (task1颜色 / task2边缘)
  5. 求解: 生成拾取-放置任务
  6. 下发: 任务JSON通过SSH发到Pi, Pi归零后开始模拟搬运 (无电磁铁)

用法: python run_cv_demo.py [task1|task2]
"""
import sys
import os
import json
import time
import subprocess

import cv2
import numpy as np

from config import *
from camera_backend import create_camera_backend, release_camera_backend
from vision import PuzzleVision
from puzzle_solver import PuzzleSolver

PI_SSH = ['ssh', '-i', os.path.expanduser('~/.ssh/epuzzle-image-access-ed25519'),
          '-o', 'BindInterface=wlo1', 'epuzzle@192.168.1.110']
PI_DIR = '/home/epuzzle/puzzle'
PI_PY = f'{PI_DIR}/venv/bin/python'

OUT_DIR = 'out/demo'

# 演示场景: 基座J1轴心的世界坐标 (板坐标系mm)
# 手动测量: 标定板图像右侧边缘(x=0边)外侧58mm, 纵向板中
ARM_BASE_WORLD = (-58.0, 117.0)
# 碎片过滤: 只保留落在黑色A4平台上的 (板坐标系mm, 棋盘区170x234加边距)
PIECE_WORLD_RECT = (-10, 180, -10, 245)  # x_min, x_max, y_min, y_max
# 臂可达半径 (演示用安全环形区)
REACH_R_MIN, REACH_R_MAX = 150.0, 170.0


def pi_run(cmd, capture=False, timeout=300):
    """在Pi上执行命令"""
    full = PI_SSH + [cmd]
    if capture:
        r = subprocess.run(full, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip(), r.returncode
    r = subprocess.run(full, timeout=timeout)
    return '', r.returncode


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'task1'
    os.makedirs(OUT_DIR, exist_ok=True)
    stamp = time.strftime('%H%M%S')

    print('=' * 50)
    print(f'  CV识别 + Pi模拟搬运演示  模式: {mode}')
    print('=' * 50)

    # ---------- 1. 记录当前状态 ----------
    print('\n[1/6] 记录当前机械臂位置...')
    out, rc = pi_run(f'cd {PI_DIR} && {PI_PY} pi_sim_run.py record', capture=True, timeout=60)
    if rc == 0 and out:
        # 最后一行是JSON
        line = [l for l in out.splitlines() if l.startswith('{')][-1]
        record = json.loads(line)
        rec_path = f'{OUT_DIR}/arm_record_{stamp}.json'
        with open(rec_path, 'w') as f:
            json.dump(record, f, indent=2)
        print(f'  舵机raw: {record["raw"]}  已存 {rec_path}')
    else:
        print('  [警告] 记录失败, 继续 (rc=%d) %s' % (rc, out[-200:]))

    # ---------- 2. 驻停到记录的不遮挡位置 ----------
    print('\n[2/6] 机械臂移动到记录的驻车位 (不遮挡相机)...')
    _, rc = pi_run(f'cd {PI_DIR} && {PI_PY} pi_sim_run.py park', timeout=120)
    print('  驻停完成' if rc == 0 else '  [警告] 驻停指令异常, 继续')

    # ---------- 3. 视觉标定 ----------
    print('\n[3/6] 视觉标定...')
    backend = create_camera_backend()
    vision = PuzzleVision(backend)

    if vision.load_calibration():
        print('  已加载 homography.npy (本地标定产物)')
    else:
        print('  无标定文件, 自动检测A4纸标定...')
        quad = vision.auto_detect_paper()
        if quad is None:
            print('  标定失败: 未检测到A4纸!')
            release_camera_backend()
            return False
        vision.set_paper_homography(quad)
        print('  自动标定完成, 已保存 homography.npy')

    # ---------- 4. CV识别 (期间机械臂保持驻停) ----------
    print('\n[4/6] CV识别碎片...')
    best = []
    best_frame = None
    for _ in range(3):
        frame = vision.capture_frame()
        if frame is None:
            continue
        if mode == 'task1':
            pieces = vision.detect_colored_pieces(frame)
        else:
            pieces = vision.detect_pieces_by_edge(frame)
        # 只保留落在黑色A4平台上的 (滤掉红色舵机线/卡扣等)
        x0, x1, y0, y1 = PIECE_WORLD_RECT
        pieces = [p for p in pieces
                  if x0 <= p['center_world'][0] <= x1
                  and y0 <= p['center_world'][1] <= y1]
        if len(pieces) > len(best):
            best, best_frame = pieces, frame
        time.sleep(0.1)

    if not best:
        print('  未识别到碎片!')
        release_camera_backend()
        return False

    print(f'  识别到 {len(best)} 个碎片:')
    n_bad = 0
    for i, p in enumerate(best):
        cx, cy = p['center_world']
        in_paper = 0 <= cx <= A4_WIDTH and 0 <= cy <= A4_HEIGHT
        n_bad += not in_paper
        print(f'    碎片{i}: ({cx:.1f},{cy:.1f})mm 角度={p["angle"]:.1f}°'
              + ('' if in_paper else '  [超出A4范围!]'))
    if n_bad:
        print(f'  [警告] {n_bad}个碎片坐标超出A4纸范围, 标定可能与当前相机位置不符,')
        print('         建议删除 homography.npy 重新标定后再运行')

    # 存档识别现场
    if best_frame is not None:
        cv2.imwrite(f'{OUT_DIR}/scene_{stamp}.jpg', best_frame)

    # ---------- 5. 生成演示任务 ----------
    # 演示模式: 不用PuzzleSolver的A4坐标系目标; 以基座世界坐标为准,
    # 放置点 = 拾取点向基座侧平移, 并钳位到可达环形区
    print('\n[5/6] 生成演示任务 (可达性钳位)...')
    bx, by = ARM_BASE_WORLD
    print(f'  基座世界坐标: ({bx:.1f},{by:.1f})mm (手动测量: 板右缘外58mm)')

    def clamp_reach(x, y):
        dx, dy = x - bx, y - by
        r = (dx ** 2 + dy ** 2) ** 0.5
        if r > REACH_R_MAX:
            s = REACH_R_MAX / r
            x, y = bx + dx * s, by + dy * s
            print(f'    [钳位] r={r:.0f}mm 超程, 缩到 {REACH_R_MAX:.0f}mm')
        elif r < REACH_R_MIN:
            s = REACH_R_MIN / max(r, 1e-6)
            x, y = bx + dx * s, by + dy * s
        return (x, y)

    tasks = []
    for i, p in enumerate(best):
        px_, py_ = p['center_world']
        pick = clamp_reach(px_, py_)
        # 放置点: 拾取点绕基座旋转约30°方向错开, 同半径
        dx, dy = pick[0] - bx, pick[1] - by
        r = (dx ** 2 + dy ** 2) ** 0.5
        place = (bx + dy * 0.6 + dx * 0.4, by - dx * 0.6 + dy * 0.4)
        place = clamp_reach(*place)
        tasks.append({'pick_pos': pick, 'pick_angle': 0,
                      'place_pos': place, 'place_angle': 0})

    for i, t in enumerate(tasks):
        print(f'    任务{i}: ({t["pick_pos"][0]:.0f},{t["pick_pos"][1]:.0f}) -> '
              f'({t["place_pos"][0]:.0f},{t["place_pos"][1]:.0f})')

    release_camera_backend()
    vision.release()

    # ---------- 6. 下发Pi执行 (Pi端先归零再开始) ----------
    print('\n[6/6] 下发树莓派, 归零后开始模拟搬运...')
    tasks_path = f'{OUT_DIR}/tasks_{stamp}.json'
    with open(tasks_path, 'w') as f:
        json.dump({'arm_base': [bx, by], 'tasks': tasks}, f)

    subprocess.run(['scp', '-i', os.path.expanduser('~/.ssh/epuzzle-image-access-ed25519'),
                    '-o', 'BindInterface=wlo1',
                    tasks_path, f'epuzzle@192.168.1.110:{PI_DIR}/tasks_demo.json'],
                   check=True, capture_output=True)

    _, rc = pi_run(f'cd {PI_DIR} && {PI_PY} pi_sim_run.py run tasks_demo.json', timeout=600)

    print('\n' + '=' * 50)
    print('  演示完成!' if rc == 0 else '  Pi端执行异常!')
    print('=' * 50)
    return rc == 0


if __name__ == '__main__':
    try:
        ok = main()
        sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        print('\n用户中断')
        sys.exit(130)
