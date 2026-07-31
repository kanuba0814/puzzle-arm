#!/usr/bin/env python3
"""
演示: 机械臂依次悬停A4两个区域中心各1秒, 然后回到当前位置 (在Pi上运行)

需要同目录 arm_base.json (基座世界坐标, 由触点标定或尺量得到):
  {"base_x": -58.0, "base_y": 148.5, "rot_deg": 0.0}

流程 (全程关节空间运动, 避开笛卡尔走廊的J2限位问题):
  记录当前pose -> 下半区中心上方悬停1s
  -> 上半区中心上方悬停1s -> 回到记录的初始pose
任何一步IK不可达则不动作直接退出 (安全)。

用法: cd ~/puzzle && venv/bin/python tools/demo_two_centers.py
"""
import os
import sys
import json
import time
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arm_controller import ArmController
from kinematics import (forward_kinematics, inverse_kinematics,
                        servo_to_deg, ik_to_servo_values)
from config import SERVO_IDS

# 世界坐标 (mm): 原点纸左下角, x右0-210, y上0-297, 分界线y=148.5
LOWER_CENTER = (105.0, 74.25)    # 下半区 (装配区) 中心
UPPER_CENTER = (105.0, 222.75)   # 上半区 (碎片区) 中心
Z_HOVER = 50.0     # 悬停高度 (不接触纸面)
WRIST = -70        # 腕俯仰 (-90会顶限位, 见 pi_sim_run)
HOVER_SEC = 1.0
MOVE_STEPS = 25    # 关节空间插值步数
MOVE_DT = 0.09     # 每步耗时(s) -> 单程约2.3s


def world_to_arm(wx, wy, base):
    dx, dy = wx - base['base_x'], wy - base['base_y']
    th = math.radians(-base.get('rot_deg', 0.0))
    c, s = math.cos(th), math.sin(th)
    return (c * dx - s * dy, s * dx + c * dy)


def joint_move(arm, raw_from, raw_to, steps=MOVE_STEPS, dt=MOVE_DT):
    """关节空间(舵机raw值)线性插值运动"""
    for k in range(1, steps + 1):
        t = k / steps
        mid = {sid: int(round(raw_from[sid] + (raw_to[sid] - raw_from[sid]) * t))
               for sid in SERVO_IDS}
        arm.write_positions_sync(mid)
        time.sleep(dt)
    arm.wait_for_stop()


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    cfg_path = os.path.join(here, 'arm_base.json')
    if not os.path.exists(cfg_path):
        print(f'缺少 {cfg_path} (基座世界坐标), 先做基座标定')
        return 1
    with open(cfg_path) as f:
        base = json.load(f)
    print(f'基座世界坐标: ({base["base_x"]:.1f},{base["base_y"]:.1f}) '
          f'rot={base.get("rot_deg", 0.0):+.2f}°')

    targets = []
    for name, w in [('下半区', LOWER_CENTER), ('上半区', UPPER_CENTER)]:
        cx, cy = world_to_arm(w[0], w[1], base)
        print(f'{name}中心 世界{w} -> 臂系 ({cx:.1f},{cy:.1f})')
        ik = inverse_kinematics(cx, cy, Z_HOVER, wrist_pitch_deg=WRIST)
        if ik is None:
            print(f'[中止] {name}中心 z={Z_HOVER} 不可达, 不动作')
            return 2
        sv = ik_to_servo_values(ik)
        targets.append((name, sv))
        print(f'   IK ✓ J={[round(v,1) for v in ik]} -> raw={sv}')

    arm = ArmController()
    raw0 = {sid: arm.read_position(sid) for sid in SERVO_IDS}
    degs0 = [servo_to_deg(raw0[s], s) for s in SERVO_IDS]
    p0 = forward_kinematics(degs0[:4])
    print(f'初始位置(臂系): ({p0[0]:.1f},{p0[1]:.1f},{p0[2]:.1f}) raw={raw0}')

    try:
        cur = raw0
        for i, (name, sv) in enumerate(targets, 1):
            print(f'{i}. 移动到{name}中心上方 z={Z_HOVER:.0f}mm')
            joint_move(arm, cur, sv)
            cur = sv
            print(f'   >>> 悬停 {HOVER_SEC}s ({name}中心)')
            time.sleep(HOVER_SEC)

        print(f'{len(targets)+1}. 回到初始位置')
        joint_move(arm, cur, raw0)
        print('完成, 已回到初始位置')
    except KeyboardInterrupt:
        print('\n[中断] 尝试回初始位置...')
        arm.write_positions_sync(raw0)
        arm.wait_for_stop()
    finally:
        arm.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
