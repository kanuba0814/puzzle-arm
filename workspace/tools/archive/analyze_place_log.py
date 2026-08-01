#!/usr/bin/env python3
"""
[废弃 2026-08-01] 旋转口径已被标准视角统一取代（俯视 A4、顺逆时针为准；实际碎片旋转 = Δroll_phys − Δpan_phys，目标 −Δyaw）。仅供历史参考，不得用于旋转验证结论。
从执行日志反推实际落点（关节角 → TCP → table_mm）。"""
import json
import sys
from pathlib import Path

import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from control.kinematics import PlanarArmKinematics, JointVector  # noqa: E402
from control.calibration import ArmCalibration  # noqa: E402

cal = ArmCalibration.load("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json")
kin = PlanarArmKinematics(cal.require_accepted_geometry())

# table->robot 逆变换（robot -> table）
yaw_deg = 4.43177
th = np.radians(yaw_deg)
ct, st = np.cos(th), np.sin(th)
tx, ty, tz = 55.888937, -140.734659, -25.237852
R_t2r = np.array([[ct, -st], [st, ct]])
R_r2t = R_t2r.T
t_r2t = -R_r2t @ np.array([tx, ty])

def robot_to_table(xyz):
    xy = R_r2t @ np.array(xyz[:2]) + t_r2t
    return xy[0], xy[1]

lines = open("/opt/epuzzle/workspace/logs/supervisor/assembly-4pieces.jsonl").readlines()
seen = set()
for line in lines:
    if '"goal_id": "place-piece_' in line and "final_joint_deg" in line:
        d = json.loads(line)
        j = d["payload"]["final_joint_deg"]
        pid = d["piece_id"]
        if pid in seen:
            continue
        seen.add(pid)
        jv = JointVector.from_mapping(j)
        pose = kin.forward(jv)
        x, y = robot_to_table(pose.position_mm)
        print(f"{pid}: TCP_table=({x:.1f},{y:.1f})mm  yaw_robot={pose.yaw_deg:.1f}")
