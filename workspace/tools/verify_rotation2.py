#!/usr/bin/env python3
"""验证旋转补偿：实际碎片旋转 ≡ −Δyaw (mod 360)。

标准视角（2026-08-01 统一）：俯视 A4、z 向上、磁吸面朝下。
实测硬件方向：pan 物理角增大 = 俯视逆时针，roll 物理角增大 = 俯视顺时针，
因此实际碎片旋转（俯视顺时针为正）= Δroll_phys − Δpan_phys。
求解器 Δyaw 以俯视逆时针为正，故实际旋转必须等于 −Δyaw。
命令 yaw 增量取 Δyaw_cmd = 2*Δpan − Δyaw（见 run_assembly_4pieces.py）。
"""
import json
import sys
from pathlib import Path

import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402
from control.motion_controller import MotionConfig  # noqa: E402
from control.kinematics import ToolPose  # noqa: E402
from integration.sweep_planner import PieceRef  # noqa: E402

TABLE_TO_ROBOT_YAW_DEG = 4.43177

mc = MotionConfig(15.0, 10.0, 10.0, 3.0, 1, 0.02, 3.0, -20.0, -30.0, 180.0)
ad = InspectionHardwareAdapter(
    calibration_path=Path("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"),
    transform_path=Path("/opt/epuzzle/workspace/calibration/contact_sheet_to_robot_base_20260731.json"),
    park_pose_path=Path("/opt/epuzzle/workspace/calibration/park_pose_single_arm_20260731.json"),
    port=Path("/dev/epuzzle-arm"),
    lerobot_calibration_dir=Path("/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"),
    motion_config=mc,
)
kin = ad.kinematics
geom = kin.geometry


def to_phys(logical, idx):
    return logical * geom.joint_directions[idx] + geom.joint_zero_offsets_deg[idx]


def verify_piece(pid, src_xy, src_yaw, tgt_xy, tgt_yaw):
    src_ref = PieceRef(pid, src_xy)
    tgt_ref = PieceRef(pid + "-place", tgt_xy)
    # PICK yaw 用径向默认（与 run_assembly_4pieces 一致；0° 基准下
    # src_yaw 恒为 0，碎片初始朝向不测量）
    pick_base = ad.pose_for(src_ref, 60.0)
    pick_yaw = pick_base.yaw_deg
    pick_pose = ToolPose(pick_base.frame, pick_base.position_mm, pick_base.pitch_deg, pick_yaw)
    pm = kin.inverse(pick_pose).as_mapping()
    pan_pick_phys = to_phys(pm["shoulder_pan.pos"], 0)
    roll_pick_phys = to_phys(pm["wrist_roll.pos"], 4)

    place_base = ad.pose_for(tgt_ref, 60.0)
    pan_place_phys = float(np.degrees(np.arctan2(place_base.position_mm[1], place_base.position_mm[0])))

    delta_yaw = (tgt_yaw - src_yaw + 180.0) % 360.0 - 180.0
    # 修正命令角：实际碎片旋转（俯视顺正）= Δroll−Δpan = Δyaw_cmd−2Δpan，
    # 要求等于 −Δyaw → Δyaw_cmd = 2*Δpan − Δyaw
    pan_delta_phys = pan_place_phys - pan_pick_phys
    delta_yaw_cmd = 2.0 * pan_delta_phys - delta_yaw
    roll_limits = geom.joint_limits_deg[4]
    roll_lo = min(roll_limits[0] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4],
                  roll_limits[1] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4])
    roll_hi = max(roll_limits[0] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4],
                  roll_limits[1] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4])

    chosen = None
    for cand in [delta_yaw_cmd, delta_yaw_cmd + 360.0, delta_yaw_cmd - 360.0]:
        rp = roll_pick_phys + cand - pan_delta_phys
        if roll_lo - 1e-6 <= rp <= roll_hi + 1e-6:
            chosen = cand
            print(
                f"{pid}: Δyaw={delta_yaw:+.1f}° 修正命令角 {cand:+.1f}° | "
                f"pan_phys {pan_pick_phys:+.1f}→{pan_place_phys:+.1f} | "
                f"roll_phys {roll_pick_phys:+.1f}→{rp:+.1f} ✓ 限位[{roll_lo:.0f},{roll_hi:.0f}]"
            )
            break
    if chosen is None:
        print(f"{pid}: ⚠ 无限位内等价角!"); return False

    # 验证：place 的 IK 实际 roll 物理值 → 实际碎片旋转（俯视顺正）
    place_yaw = pick_yaw + chosen
    place_pose = ToolPose(place_base.frame, place_base.position_mm, place_base.pitch_deg, place_yaw)
    lm = kin.inverse(place_pose).as_mapping()
    roll_place_actual = to_phys(lm["wrist_roll.pos"], 4)
    actual_rot = (roll_place_actual - roll_pick_phys) - (pan_place_phys - pan_pick_phys)
    ok = abs((actual_rot + delta_yaw + 180.0) % 360.0 - 180.0) < 0.5
    print(f"    实际碎片旋转 {actual_rot:+.1f}° vs 目标 −Δyaw {(-delta_yaw):+.1f}° → {'✓ 一致' if ok else '✗ 偏差!'}")
    return ok


plan_path = sys.argv[1] if len(sys.argv) > 1 else "/home/epuzzle/plan_v6/result.json"
d = json.load(open(plan_path))
ps = d["solve"]["placements"]
print("=== 旋转补偿验证（物理角）===")
all_ok = True
for p in ps:
    s = p["source"]["center_mm"]
    t = p["target"]["center_mm"]
    ok = verify_piece(p["piece_id"], s, p["source"]["yaw_deg"], t, p["target"]["yaw_deg"])
    all_ok = all_ok and ok
print()
print("全部通过 ✓" if all_ok else "有偏差 ✗")
