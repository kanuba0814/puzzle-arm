#!/usr/bin/env python3
"""
[废弃 2026-08-01] 旋转口径已被标准视角统一取代（俯视 A4、顺逆时针为准；实际碎片旋转 = Δroll_phys − Δpan_phys，目标 −Δyaw）。仅供历史参考，不得用于旋转验证结论。
验证旋转补偿：碎片旋转量 = 末端 yaw 变化 = Δyaw（含 pan 补偿）。

关键公式（用户要求的实现）：
  roll_place = roll_pick + Δyaw - (pan_place - pan_pick)
即 IK 的 roll = yaw_deg - pan，配合 place_yaw = pick_yaw + Δyaw。
"""
import sys
from pathlib import Path

import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402
from control.motion_controller import MotionConfig  # noqa: E402
from control.kinematics import ToolPose  # noqa: E402

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


def yaw_pose(xy, height, yaw_table_deg):
    rp = ad.transform.table_to_robot(xy, height)
    return ToolPose(
        "robot_base_mm", rp, -87.5, yaw_table_deg + TABLE_TO_ROBOT_YAW_DEG
    )


def verify(src_xy, src_yaw, tgt_xy, tgt_yaw):
    pick = yaw_pose(src_xy, 5.0, src_yaw)
    place = yaw_pose(tgt_xy, 9.0, tgt_yaw)
    pj = kin.inverse(pick)
    lj = kin.inverse(place)
    pm = pj.as_mapping()
    lm = lj.as_mapping()
    pan_pick = pm["shoulder_pan.pos"]
    pan_place = lm["shoulder_pan.pos"]
    roll_pick = pm["wrist_roll.pos"]
    roll_place = lm["wrist_roll.pos"]

    delta_yaw = (tgt_yaw - src_yaw + 180.0) % 360.0 - 180.0
    # 碎片实际旋转 = 末端 yaw 变化（robot 系）
    yaw_pick = pan_pick + roll_pick
    yaw_place = pan_place + roll_place
    actual_rot = (yaw_place - yaw_pick + 180.0) % 360.0 - 180.0
    ok = abs(actual_rot - delta_yaw) < 0.5

    print(
        f"  pan {pan_pick:7.1f}→{pan_place:7.1f} (Δ{pan_place-pan_pick:+7.1f}) | "
        f"roll {roll_pick:7.1f}→{roll_place:7.1f} | "
        f"末端yaw {yaw_pick:7.1f}→{yaw_place:7.1f} | "
        f"实际旋转 {actual_rot:7.1f} vs 目标Δyaw {delta_yaw:7.1f} {'✓' if ok else '✗'}"
    )
    return ok


# 用 v7 检测结果（如果求解成功）或 v6 计划
import json
try:
    d = json.load(open("/home/epuzzle/plan_v7/result.json"))
    ps = d.get("solve", {}).get("placements", [])
    if not ps:
        raise ValueError("v7 无解")
except Exception:
    d = json.load(open("/home/epuzzle/plan_v6/result.json"))
    ps = d["solve"]["placements"]

print("=== 旋转补偿验证（含 pan 补偿 + robot 系转换）===")
all_ok = True
for p in ps:
    s = p["source"]["center_mm"]
    t = p["target"]["center_mm"]
    print(f"{p['piece_id']}: 源({s[0]:.1f},{s[1]:.1f}) yaw={p['source']['yaw_deg']:.1f} → 目标({t[0]:.1f},{t[1]:.1f}) yaw={p['target']['yaw_deg']:.1f}")
    ok = verify(s, p["source"]["yaw_deg"], t, p["target"]["yaw_deg"])
    all_ok = all_ok and ok

print()
print("全部通过 ✓" if all_ok else "有失败 ✗")
