#!/usr/bin/env python3
"""高位悬停验证 (步骤2): 把末端移到指定 pick 点上方 APPROACH_Z=80mm, 目视确认后回 PARK。

用法 (在 Pi 上, 有人看护):
    ./venv/bin/python tools/high_hover_check.py <pick_x> <pick_y> <roll_deg>

流程:
    1. 连接机械臂, 读取当前关节 (应确认在纸外 PARK)
    2. 移动到 (pick_x, pick_y) 上方 80mm, wrist_pitch=-90 (命令值)
    3. 等待操作者目视确认 (回车继续 / q 回 PARK)
    4. 回 PARK 姿态
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_controller import ArmController  # noqa: E402
from config import APPROACH_Z, SERVO_BAUDRATE, SERVO_PORT  # noqa: E402
from motion_planner import world_angle_to_arm, world_to_arm  # noqa: E402


def main():
    hold_seconds = 0.0
    if "--hold-seconds" in sys.argv:
        idx = sys.argv.index("--hold-seconds")
        hold_seconds = float(sys.argv[idx + 1])
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) < 3:
        print("用法: high_hover_check.py <pick_x> <pick_y> <roll_deg> "
              "[--hold-seconds N]")
        return 1
    pick = (float(args[0]), float(args[1]))
    roll_deg = float(args[2])

    arm = ArmController(SERVO_PORT, SERVO_BAUDRATE)
    if arm.simulation:
        print("[Hover] 机械臂处于模拟模式 (串口未连接), 禁止执行")
        return 1
    try:
        print("[Hover] 当前关节 raw:", arm.get_current_position())
        # 世界 -> 臂系
        px, py = world_to_arm(*pick)
        roll = world_angle_to_arm(roll_deg)
        # 悬停高度按点自适应 (近基座点 J2 超限)
        from kinematics import inverse_kinematics

        z_hover = None
        for z in (APPROACH_Z, 60.0, 50.0, 40.0, 30.0, 25.0, 20.0):
            if inverse_kinematics(px, py, z, -90.0, 0.0) is not None:
                z_hover = z
                break
        if z_hover is None:
            print("[Hover] 该点无可达悬停高度, 中止")
            return 1
        print(f"[Hover] 目标: 世界({pick[0]:.1f},{pick[1]:.1f}) -> "
              f"臂系({px:.1f},{py:.1f}) z={z_hover:.0f} roll={roll:.1f}° "
              f"(自适应高度)")
        print("[Hover] 开始移动到拾取点上方 (不下降)...")
        ok = arm.move_to_xyz(px, py, z_hover, wrist_pitch=-90, gripper_roll=roll)
        if not ok:
            print("[Hover] IK 不可达, 中止")
            return 1
        arm.wait_for_stop()
        print("[Hover] 已悬停。请目视确认:")
        print("      1) 磁铁工作面竖直向下 (命令 -90°, 观察实际俯仰)")
        print("      2) 磁铁中心对准第 1 片碎片中心")
        print("      3) 高度 80mm 安全, 无碰撞风险")
        if hold_seconds > 0:
            print(f"[Hover] 悬停保持 {hold_seconds:.0f} 秒, 超时自动回 PARK")
            for remaining in range(int(hold_seconds), 0, -1):
                print(f"  剩余 {remaining:3d}s...", end="\r")
                time.sleep(1)
            print()
        else:
            input("  确认无误按回车回 PARK; 输入 q 直接回 PARK: ")
        arm.go_home()
        arm.wait_for_stop()
        print("[Hover] 已回 PARK。悬停验证完成")
        return 0
    except KeyboardInterrupt:
        print("\n[Hover] 中断, 回 PARK")
        arm.go_home()
        arm.wait_for_stop()
        return 1
    finally:
        arm.close()


if __name__ == "__main__":
    sys.exit(main())
