#!/usr/bin/env python3
"""分析 place-piece_00 的所有执行记录。"""
import json
import sys
from pathlib import Path

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from control.kinematics import PlanarArmKinematics, JointVector  # noqa: E402
from control.calibration import ArmCalibration  # noqa: E402
import numpy as np

cal = ArmCalibration.load("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json")
kin = PlanarArmKinematics(cal.require_accepted_geometry())

yaw_deg = 4.43177
th = np.radians(yaw_deg)
ct, st = np.cos(th), np.sin(th)
tx, ty = 55.888937, -140.734659
R_r2t = np.array([[ct, st], [-st, ct]])
t_r2t = -R_r2t @ np.array([tx, ty])

def robot_to_table(xyz):
    xy = R_r2t @ np.array(xyz[:2]) + t_r2t
    return xy[0], xy[1]

lines = open("/opt/epuzzle/workspace/logs/supervisor/assembly-4pieces.jsonl").readlines()
count = 0
for line in lines:
    if "place-piece_00" in line and "final_joint_deg" in line:
        count += 1
        d = json.loads(line)
        j = d["payload"]["final_joint_deg"]
        jv = JointVector.from_mapping(j)
        pose = kin.forward(jv)
        x, y = robot_to_table(pose.position_mm)
        print(
            f"place-piece_00 第{count}次: TCP_table=({x:.1f},{y:.1f}) "
            f"pan={j['shoulder_pan.pos']:.1f} roll={j['wrist_roll.pos']:.1f} "
            f"yaw_robot={pose.yaw_deg:.1f}"
        )
