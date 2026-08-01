#!/usr/bin/env python3
"""分析旋转偏差：source_yaw（质心→最远顶点）vs 检测 PCA 角 vs 实际执行旋转。"""
import json
import sys
from pathlib import Path

import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from control.kinematics import PlanarArmKinematics, JointVector  # noqa: E402
from control.calibration import ArmCalibration  # noqa: E402

CAL = ArmCalibration.load("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json")
KIN = PlanarArmKinematics(CAL.require_accepted_geometry())

YAW_T2R = 4.43177
TH = np.radians(YAW_T2R)
CT, ST = np.cos(TH), np.sin(TH)
TX, TY = 55.888937, -140.734659
R_R2T = np.array([[CT, ST], [-ST, CT]])
T_R2T = -R_R2T @ np.array([TX, TY])


def robot_to_table(xyz):
    xy = R_R2T @ np.array(xyz[:2]) + T_R2T
    return float(xy[0]), float(xy[1])


d = json.load(open("/home/epuzzle/plan_v10/result.json"))
lines = open("/opt/epuzzle/workspace/logs/supervisor/assembly-4pieces.jsonl").readlines()

# 每片最后一次 pick/place 的关节角
last_pick = {}
last_place = {}
for line in lines:
    if "motion_actual_readback" not in line:
        continue
    dd = json.loads(line)
    gid = dd["payload"].get("goal_id", "")
    pid = dd.get("piece_id", "")
    if gid.startswith("pick-"):
        last_pick[pid] = dd["payload"]["final_joint_deg"]
    elif gid.startswith("place-"):
        last_place[pid] = dd["payload"]["final_joint_deg"]

print(f"{'片':10s} {'PCA角':>7s} {'源yaw':>7s} {'R角':>7s} {'Δyaw':>7s} {'执行旋转':>8s} {'偏差':>7s}")
for pl in d["solve"]["placements"]:
    pid = pl["piece_id"]
    # PCA
    piece = [p for p in d["pieces"] if f"piece_{p['id']:02d}" == pid]
    pca = piece[0]["angle_deg"] if piece else float("nan")
    src_yaw = pl["source"]["yaw_deg"]
    tgt_yaw = pl["target"]["yaw_deg"]
    dy = (tgt_yaw - src_yaw + 180) % 360 - 180
    # 执行旋转（物理角）
    if pid in last_pick and pid in last_place:
        pj = last_pick[pid]
        lj = last_place[pid]
        # 物理角
        geom = CAL.require_accepted_geometry()
        pan_p = pj["shoulder_pan.pos"] * geom.joint_directions[0] + geom.joint_zero_offsets_deg[0]
        roll_p = pj["wrist_roll.pos"] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4]
        pan_l = lj["shoulder_pan.pos"] * geom.joint_directions[0] + geom.joint_zero_offsets_deg[0]
        roll_l = lj["wrist_roll.pos"] * geom.joint_directions[4] + geom.joint_zero_offsets_deg[4]
        exec_rot = (pan_l + roll_l) - (pan_p + roll_p)
        err = (exec_rot - dy + 180) % 360 - 180
        print(f"{pid:10s} {pca:7.1f} {src_yaw:7.1f} {tgt_yaw-src_yaw:7.1f} {dy:7.1f} {exec_rot:8.1f} {err:7.1f}")
    else:
        print(f"{pid:10s} {pca:7.1f} {src_yaw:7.1f} {tgt_yaw-src_yaw:7.1f} {dy:7.1f} {'?':>8s}")

print()
print("注意: 源yaw = 质心→最远顶点方向（适配器计算）; PCA = 检测惯性主轴角")
print("如果 源yaw 与 PCA 差 ~90° 或 ~180°，说明参考方向选择导致旋转基准偏移")
