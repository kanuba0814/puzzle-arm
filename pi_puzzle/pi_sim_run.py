"""
Pi端: 模拟搬运执行脚本 (无电磁铁, 仅机械臂动作)

用法 (在 Pi 上运行, venv python):
  python pi_sim_run.py record            # 记录当前舵机位置, 输出JSON
  python pi_sim_run.py home              # 回go_home驻车位(不遮挡相机)
  python pi_sim_run.py run tasks.json    # 归零后按任务表模拟拾取-放置
"""
import sys
import json

from arm_controller import ArmController
from motion_planner import MotionPlanner
from config import SERVO_IDS


def make_arm():
    arm = ArmController()
    # 无电磁铁: 吸放动作全部替换为空操作
    arm.magnet_on = lambda: print("  [Sim] magnet_on (跳过, 未接电磁铁)")
    arm.magnet_off = lambda: print("  [Sim] magnet_off (跳过)")
    arm.open_gripper = arm.magnet_off
    arm.close_gripper = arm.magnet_on
    # 演示: 腕俯仰 -90 会把 J4 顶到限位, 放宽到 -70
    _orig_move = arm.move_to_xyz
    def move70(x, y, z, wrist_pitch=-90, gripper_roll=0, **kw):
        return _orig_move(x, y, z, -70, gripper_roll, **kw)
    arm.move_to_xyz = move70
    return arm


def cmd_record():
    arm = make_arm()
    raw = {sid: arm.read_position(sid) for sid in SERVO_IDS}
    print(json.dumps({'raw': raw}))
    arm.close()


def cmd_home():
    arm = make_arm()
    arm.go_home()
    arm.wait_for_stop()
    print("[Pi] 已到驻车/零位")
    arm.close()


def cmd_park():
    """移动到 park_pose.json 记录的不遮挡相机位姿"""
    with open('park_pose.json') as f:
        raw = {int(k): v for k, v in json.load(f).items()}
    arm = make_arm()
    arm.write_positions_sync(raw)
    arm.wait_for_stop()
    print(f"[Pi] 已到记录驻车位: {raw}")
    arm.close()


def cmd_run(tasks_path):
    with open(tasks_path) as f:
        data = json.load(f)
    # 兼容两种格式: 纯任务list, 或 {'arm_base': [x,y], 'tasks': [...]}
    if isinstance(data, dict):
        import motion_planner as mp
        mp.ARM_BASE_X, mp.ARM_BASE_Y = data['arm_base']
        mp.ARM_BASE_ROT_DEG = float(data.get('arm_rot_deg', 0.0))
        tasks = data['tasks']
        print(f"[Pi] 基座世界坐标覆盖: {data['arm_base']} "
              f"rot={mp.ARM_BASE_ROT_DEG:+.2f}deg")
    else:
        import motion_planner as mp
        tasks = data
    # 演示(无电磁铁): J2<=82°时可达环为 r>=140mm, 抬高拾取高度
    mp.APPROACH_Z = 100.0
    mp.PICK_Z = 60.0
    print(f"[Pi] 收到 {len(tasks)} 个任务, 先归零再开始")
    arm = make_arm()
    planner = MotionPlanner(arm)

    # 归零开始
    arm.go_home()
    arm.wait_for_stop()

    for i, t in enumerate(tasks):
        print(f"\n=== 任务 {i+1}/{len(tasks)} ===")
        ok = planner.pick_and_place(
            tuple(t['pick_pos']), t.get('pick_angle', 0),
            tuple(t['place_pos']), t.get('place_angle', 0))
        print(f"  -> {'完成' if ok else '失败'}")

    # 结束后回驻车位 (有记录姿态则回驻车位, 否则回零位)
    import os
    if os.path.exists('park_pose.json'):
        with open('park_pose.json') as f:
            raw = {int(k): v for k, v in json.load(f).items()}
        arm.write_positions_sync(raw)
    else:
        arm.go_home()
    arm.wait_for_stop()
    print("[Pi] 全部完成, 已回驻车位")
    arm.close()


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'home'
    if cmd == 'record':
        cmd_record()
    elif cmd == 'home':
        cmd_home()
    elif cmd == 'park':
        cmd_park()
    elif cmd == 'run':
        cmd_run(sys.argv[2])
    else:
        print(__doc__)
        sys.exit(1)
