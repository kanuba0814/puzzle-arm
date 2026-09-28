#!/usr/bin/env python3
"""检查 piece_03 源位/目标位各高度 IK 可达性。"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/opt/epuzzle/workspace/src")
from control.motion_controller import MotionConfig  # noqa: E402
from control.kinematics import ToolPose  # noqa: E402
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402

mc = MotionConfig(15.0, 10.0, 10.0, 3.0, 1, 0.02, 3.0, -20.0, -30.0, 180.0,
                  transit_cartesian_step_mm=25.0, follow_tolerance_deg=8.0)
ad = InspectionHardwareAdapter(
    calibration_path=Path("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"),
    transform_path=Path("/opt/epuzzle/workspace/calibration/contact_sheet_to_robot_base_20260731.json"),
    park_pose_path=Path("/opt/epuzzle/workspace/calibration/park_pose_single_arm_20260731.json"),
    port=Path("/dev/epuzzle-arm"),
    lerobot_calibration_dir=Path("/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"),
    motion_config=mc,
)
kin = ad.kinematics

print("=== piece_03 源位 (173.7, 251.2) ===")
for h in (60.0, 40.0, 20.0):
    rp = ad.transform.table_to_robot((173.7, 251.2), h)
    tp = ToolPose("robot_base_mm", rp, -87.5, 68.9 + 4.43177)
    try:
        jv = kin.inverse(tp)
        m = jv.as_mapping()
        print("  h=%.0f OK pan=%.1f lift=%.1f elbow=%.1f wf=%.1f roll=%.1f" % (
            h, m["shoulder_pan.pos"], m["shoulder_lift.pos"], m["elbow_flex.pos"],
            m["wrist_flex.pos"], m["wrist_roll.pos"]))
    except Exception as e:
        print(f"  h={h}: 不可达 {str(e)[:80]}")

print("\n=== piece_03 目标位 (68.7, 59.9) ===")
for h in (60.0, 20.0):
    rp = ad.transform.table_to_robot((68.7, 59.9), h)
    tp = ToolPose("robot_base_mm", rp, -87.5, 171.2 + 4.43177)
    try:
        jv = kin.inverse(tp)
        m = jv.as_mapping()
        print("  h=%.0f OK pan=%.1f lift=%.1f elbow=%.1f wf=%.1f roll=%.1f" % (
            h, m["shoulder_pan.pos"], m["shoulder_lift.pos"], m["elbow_flex.pos"],
            m["wrist_flex.pos"], m["wrist_roll.pos"]))
    except Exception as e:
        print(f"  h={h}: 不可达 {str(e)[:80]}")

print("\n=== 目标区已拼碎片位置可达性（对比）===")
for name, xy in [("piece_02目标", (134.7, 83.0)), ("piece_01目标", (89.9, 80.4))]:
    rp = ad.transform.table_to_robot(xy, 60.0)
    tp = ToolPose("robot_base_mm", rp, -87.5, 0.0)
    try:
        jv = kin.inverse(tp)
        m = jv.as_mapping()
        print("  %s OK pan=%.1f lift=%.1f elbow=%.1f" % (name, m["shoulder_pan.pos"],
              m["shoulder_lift.pos"], m["elbow_flex.pos"]))
    except Exception as e:
        print(f"  {name}: 不可达 {str(e)[:80]}")
