#!/usr/bin/env python3
"""
基座<->世界坐标系 外参标定 (触点法, 在本机运行)

原理: 手扶机械臂, 用末端磁铁尖依次碰纸面上 N(>=2, 建议3-4) 个已知世界坐标点,
每点记录舵机raw -> FK得臂系坐标 (x,y), 与视觉世界坐标 (X,Y) 配对,
最小二乘解算 相似变换: world = R(theta) @ arm + t  (绕Z旋转theta + 平移t)。

前置:
  - homography_1440.npy 已生成 (calibrate_camera.py --desk-only)
  - Pi 在线 (SSH), 机械臂已上电
  - 相机能看到纸面和机械臂末端

流程:
  1. 脚本先在 Pi 上释放舵机力矩 (可手掰; 请全程扶住机械臂!)
  2. 预览窗口: 鼠标移动实时显示世界mm; 左键点击设置触点目标(十字线)
  3. 手扶臂把磁铁尖碰到十字点, 保持, 按 SPACE 记录该点 (>=2个点, 建议3-4分散)
  4. 按 D 解算并保存 out/arm_base_world.json; 最后恢复力矩

按键: 左键=选触点  SPACE=记录  U=撤销上点  D=解算  S=截图  Q/Esc=中止
"""
import os
import sys
import json
import time
import argparse

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import CALIB_IMAGE_SIZE, MJPEG_SHOT1440_URL, SERVO_IDS
from kinematics import forward_kinematics, servo_to_deg
from camera_backend import MjpegSnapshotBackend
from run_cv_demo import pi_run

W, H = CALIB_IMAGE_SIZE
OUT_JSON = 'out/arm_base_world.json'


# ---------------- 舵机读取 -> 臂系坐标 ----------------

def read_arm_xyz():
    """SSH读Pi舵机raw -> FK -> 臂系 (x,y,z) mm"""
    out, rc = pi_run('cd /home/epuzzle/puzzle && '
                     'venv/bin/python pi_sim_run.py record',
                     capture=True, timeout=60)
    if rc != 0 or not out:
        print('  [错误] 舵机读取失败 rc=%d %s' % (rc, out[-150:]))
        return None
    line = [l for l in out.splitlines() if l.startswith('{')][-1]
    raw = {int(k): v for k, v in json.loads(line)['raw'].items()}
    degs = [servo_to_deg(raw[sid], sid) for sid in SERVO_IDS]
    xyz = forward_kinematics(degs[:4])
    return xyz, raw, degs


# ---------------- 相似变换解算 ----------------

def solve_similarity(pairs):
    """
    pairs: [(arm_x, arm_y, world_x, world_y), ...]
    解 world = R(theta) @ arm + t; 返回 (theta_deg, t, residuals, scale_diag)
    """
    a = np.array([[p[0], p[1]] for p in pairs])
    w = np.array([[p[2], p[3]] for p in pairs])
    ac, wc = a.mean(axis=0), w.mean(axis=0)
    A, Wm = a - ac, w - wc
    # 复数域最小二乘: theta = arg(sum(W * conj(A)))
    z = np.sum((Wm[:, 0] + 1j * Wm[:, 1]) * np.conj(A[:, 0] + 1j * A[:, 1]))
    theta = np.angle(z)
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    t = wc - R @ ac
    pred = (R @ a.T).T + t
    res = np.sqrt(((pred - w) ** 2).sum(axis=1))
    # 尺度诊断: |W|/|A| 应为1 (否则连杆参数需实测)
    scale = np.sqrt((Wm ** 2).sum() / (A ** 2).sum())
    return np.degrees(theta), t, res, scale


# ---------------- 交互 ----------------

def main():
    if not os.path.exists('homography_1440.npy'):
        print('缺少 homography_1440.npy, 先运行 calibrate_camera.py --desk-only')
        return 1
    H1440 = np.load('homography_1440.npy')   # px1440 -> world
    invH = np.linalg.inv(H1440)

    backend = MjpegSnapshotBackend(shot_url=MJPEG_SHOT1440_URL)
    if not backend.open():
        print('相机打开失败')
        return 1

    mouse = {'px': None, 'target_px': None}
    pairs = []          # (arm_x, arm_y, world_x, world_y)
    shots = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            mouse['px'] = (x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            mouse['target_px'] = (x, y)
            wpt = cv2.perspectiveTransform(
                np.array([[[x, y]]], dtype=np.float32), H1440)[0, 0]
            print(f'[触点] 目标已设: px=({x},{y}) -> 世界 '
                  f'({wpt[0]:.1f},{wpt[1]:.1f}) mm')

    cv2.namedWindow('base_calib', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('base_calib', 1280, 720)
    cv2.setMouseCallback('base_calib', on_mouse)

    print('=' * 56)
    print('  基座<->世界 外参标定 (触点法)')
    print('=' * 56)
    ap = argparse.ArgumentParser()
    ap.add_argument('--go', action='store_true',
                    help='跳过确认直接释放力矩 (由对话确认后后台启动用)')
    args = ap.parse_args()
    if not args.go:
        r = input('释放Pi舵机力矩, 准备手掰机械臂? [y/N] ').strip().lower()
        if r != 'y':
            print('已取消')
            backend.release()
            return 130
    pi_run('cd /home/epuzzle/puzzle && '
           'venv/bin/python tools/servo_tool.py torque off', timeout=60)
    print('[Pi] 力矩已释放, 请全程扶住机械臂!')
    print('左键=选触点  SPACE=记录(尖碰十字)  U=撤销  D=解算  Q=中止')

    try:
        while True:
            frame = backend.read_rgb()
            if frame is None:
                time.sleep(0.2)
                continue
            if frame.shape[1] != W:
                frame = cv2.resize(frame, (W, H))
            vis = frame.copy()

            # A4面板
            a4 = np.array([[0, 0], [210, 0], [210, 297], [0, 297]],
                          dtype=np.float32).reshape(-1, 1, 2)
            px = cv2.perspectiveTransform(a4, invH).reshape(-1, 2)
            cv2.polylines(vis, [px.astype(np.int32)], True, (0, 255, 0), 2)

            # 目标十字
            if mouse['target_px'] is not None:
                tx, ty = mouse['target_px']
                cv2.drawMarker(vis, (tx, ty), (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 40, 3)

            # 已记录点
            for i, p in enumerate(pairs):
                wpt = np.array([[[p[2], p[3]]]], dtype=np.float32)
                ipx = cv2.perspectiveTransform(wpt, invH)[0, 0].astype(int)
                cv2.circle(vis, tuple(ipx), 10, (255, 0, 255), 2)
                cv2.putText(vis, f'P{i+1}', (ipx[0] + 12, ipx[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)

            # 鼠标世界坐标
            if mouse['px'] is not None:
                mx, my = mouse['px']
                wpt = cv2.perspectiveTransform(
                    np.array([[[mx, my]]], dtype=np.float32), H1440)[0, 0]
                cv2.putText(vis, f'({wpt[0]:.1f},{wpt[1]:.1f})mm',
                            (mx + 12, my + 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.9, (0, 255, 255), 2)

            cv2.putText(vis, f'points={len(pairs)}  '
                        'L-click=target SPACE=record D=solve Q=abort',
                        (25, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 255, 255), 2)
            cv2.imshow('base_calib', cv2.resize(vis, (1280, 720)))
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and mouse['target_px'] is not None:
                tx, ty = mouse['target_px']
                wpt = cv2.perspectiveTransform(
                    np.array([[[tx, ty]]], dtype=np.float32), H1440)[0, 0]
                got = read_arm_xyz()
                if got is None:
                    continue
                xyz, raw, degs = got
                pairs.append((float(xyz[0]), float(xyz[1]),
                              float(wpt[0]), float(wpt[1])))
                shots.append({'world': [float(wpt[0]), float(wpt[1])],
                              'arm': [float(xyz[0]), float(xyz[1]),
                                      float(xyz[2])],
                              'raw': raw, 'deg': [float(d) for d in degs]})
                print(f'  [记录P{len(pairs)}] 世界({wpt[0]:.1f},{wpt[1]:.1f}) '
                      f'<-> 臂系({xyz[0]:.1f},{xyz[1]:.1f},z={xyz[2]:.1f})')
                mouse['target_px'] = None
            elif key == ord('u') or key == ord('U'):
                if pairs:
                    pairs.pop()
                    shots.pop()
                    print(f'  [撤销] 剩余 {len(pairs)} 点')
            elif key == ord('s') or key == ord('S'):
                p = f'out/base_calib_{time.strftime("%H%M%S")}.jpg'
                cv2.imwrite(p, vis)
                print('  截图', p)
            elif key == ord('d') or key == ord('D'):
                if len(pairs) >= 2:
                    break
                print('  至少需要2个点')
            elif key in (ord('q'), 27):
                raise KeyboardInterrupt
    except KeyboardInterrupt:
        print('\n[中止] 恢复力矩...')
        pi_run('cd /home/epuzzle/puzzle && '
               'venv/bin/python tools/servo_tool.py torque on', timeout=60)
        backend.release()
        cv2.destroyAllWindows()
        return 130

    # ---------------- 解算 ----------------
    theta, t, res, scale = solve_similarity(pairs)
    print('\n' + '=' * 56)
    print(f'  解算结果 ({len(pairs)}点):')
    print(f'    旋转 theta = {theta:+.2f}°  (臂系 -> 世界)')
    print(f'    基座世界坐标 t = ({t[0]:.2f}, {t[1]:.2f}) mm')
    print(f'    各点残差: {[f"{r:.2f}" for r in res]} mm  '
          f'max={res.max():.2f}mm')
    print(f'    尺度诊断 |W|/|A| = {scale:.4f} (应≈1.000, '
          f'偏差大则连杆L2/L3/L4需实测)')
    zs = [s['arm'][2] for s in shots]
    print(f'    触点臂系z: {[f"{z:.0f}" for z in zs]} '
          f'(应≈基座到桌面高差, 各点应一致)')
    print('=' * 56)

    result = {
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'rot_deg': float(theta),
        'base_x': float(t[0]), 'base_y': float(t[1]),
        'residuals_mm': [float(r) for r in res],
        'fk_scale': float(scale),
        'shots': shots,
        'config_lines': {
            'ARM_BASE_X': round(float(t[0]), 2),
            'ARM_BASE_Y': round(float(t[1]), 2),
            'ARM_BASE_ROT_DEG': round(float(theta), 2),
        },
    }
    os.makedirs('out', exist_ok=True)
    with open(OUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f'已保存 {OUT_JSON}')
    print('config.py 对应行 (告诉我即可, 我来回填):')
    print(f"  ARM_BASE_X = {result['config_lines']['ARM_BASE_X']}")
    print(f"  ARM_BASE_Y = {result['config_lines']['ARM_BASE_Y']}")
    print(f"  ARM_BASE_ROT_DEG = {result['config_lines']['ARM_BASE_ROT_DEG']}")

    print('\n恢复力矩...')
    pi_run('cd /home/epuzzle/puzzle && '
           'venv/bin/python tools/servo_tool.py torque on', timeout=60)
    backend.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
