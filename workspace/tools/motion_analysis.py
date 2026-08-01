#!/usr/bin/env python3
"""生成完整的运动参数分析日志。

从 assembly-4pieces.jsonl 提取所有运动数据：
- 计划参数（源/目标位、yaw、Δyaw）
- 每次运动的关键关节角（pick/place 的 pan/roll/lift）
- 实际落点（关节角 → FK → table_mm）
- 旋转执行验证
- 偏差分析（实际 vs 计划）
输出：JSON + Markdown 双格式报告
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

_SRC = Path("/opt/epuzzle/workspace/src")
sys.path.insert(0, str(_SRC))
from control.kinematics import PlanarArmKinematics, JointVector  # noqa: E402
from control.calibration import ArmCalibration  # noqa: E402

# ---------- 变换 ----------
CAL = ArmCalibration.load("/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json")
KIN = PlanarArmKinematics(CAL.require_accepted_geometry())

YAW_T2R = 4.43177  # table -> robot yaw
TH = np.radians(YAW_T2R)
CT, ST = np.cos(TH), np.sin(TH)
TX, TY = 55.888937, -140.734659
R_R2T = np.array([[CT, ST], [-ST, CT]])
T_R2T = -R_R2T @ np.array([TX, TY])


def robot_to_table(xyz):
    xy = R_R2T @ np.array(xyz[:2]) + T_R2T
    return float(xy[0]), float(xy[1])


def to_phys(logical, idx):
    return logical * CAL.require_accepted_geometry().joint_directions[idx] + \
        CAL.require_accepted_geometry().joint_zero_offsets_deg[idx]


def main() -> None:
    log_path = Path("/opt/epuzzle/workspace/logs/supervisor/assembly-4pieces.jsonl")
    lines = [json.loads(l) for l in log_path.read_text().splitlines() if l.strip()]

    # 计划
    plan = json.load(open("/home/epuzzle/plan_v10/result.json"))
    placements = plan["solve"]["placements"]

    # 每个 goal_id 的运动记录
    motions: dict[str, dict] = {}
    for d in lines:
        ev = d.get("event", "")
        if ev == "motion_actual_readback":
            goal_id = d["payload"].get("goal_id", "")
            motions[goal_id] = {
                "piece": d.get("piece_id", ""),
                "state": d.get("state", ""),
                "joints": d["payload"]["final_joint_deg"],
                "waypoints": d["payload"].get("executed_waypoints"),
            }

    report = {
        "meta": {
            "plan": "/home/epuzzle/plan_v10/result.json",
            "log": str(log_path),
            "solve_status": plan["solve"]["status"],
            "inferred_rect_mm": plan["solve"]["inferred_target_mm"],
        },
        "pieces": [],
    }

    print("=" * 100)
    print("运动参数分析报告 (plan_v10)")
    print("=" * 100)
    print(f"求解状态: {plan['solve']['status']}  反推矩形: {plan['solve']['inferred_target_mm']}")
    print()

    for pl in placements:
        pid = pl["piece_id"]
        src = pl["source"]
        tgt = pl["target"]
        delta_yaw = (tgt["yaw_deg"] - src["yaw_deg"] + 180) % 360 - 180

        # 找该片的 pick/place 运动
        pick_goals = [g for g, m in motions.items() if m["piece"] == pid and g.startswith("pick-")]
        place_goals = [g for g, m in motions.items() if m["piece"] == pid and g.startswith("place-")]

        piece_rec = {
            "piece_id": pid,
            "plan": {
                "source_table_mm": src["center_mm"],
                "source_yaw_table_deg": src["yaw_deg"],
                "target_table_mm": tgt["center_mm"],
                "target_yaw_table_deg": tgt["yaw_deg"],
                "delta_yaw_deg": round(delta_yaw, 1),
            },
            "executions": [],
        }

        print(f"--- {pid} ---")
        print(f"  计划: 源({src['center_mm'][0]:.1f},{src['center_mm'][1]:.1f}) yaw={src['yaw_deg']:.1f} "
              f"→ 目标({tgt['center_mm'][0]:.1f},{tgt['center_mm'][1]:.1f}) yaw={tgt['yaw_deg']:.1f} Δyaw={delta_yaw:.1f}")

        # 每次执行（pick 次数 = 执行次数）
        n_exec = max(len(pick_goals), len(place_goals))
        for i in range(n_exec):
            exec_rec = {"index": i + 1}
            pick_j = motions.get(pick_goals[i], {}).get("joints") if i < len(pick_goals) else None
            place_j = motions.get(place_goals[i], {}).get("joints") if i < len(place_goals) else None

            if pick_j:
                jv = JointVector.from_mapping(pick_j)
                pose = KIN.forward(jv)
                x, y = robot_to_table(pose.position_mm)
                exec_rec["pick"] = {
                    "tcp_table_mm": [round(x, 1), round(y, 1)],
                    "pan_logical": round(pick_j["shoulder_pan.pos"], 2),
                    "lift_logical": round(pick_j["shoulder_lift.pos"], 2),
                    "roll_logical": round(pick_j["wrist_roll.pos"], 2),
                    "yaw_robot_deg": round(pose.yaw_deg, 1),
                }
            if place_j:
                jv = JointVector.from_mapping(place_j)
                pose = KIN.forward(jv)
                x, y = robot_to_table(pose.position_mm)
                exec_rec["place"] = {
                    "tcp_table_mm": [round(x, 1), round(y, 1)],
                    "pan_logical": round(place_j["shoulder_pan.pos"], 2),
                    "lift_logical": round(place_j["shoulder_lift.pos"], 2),
                    "roll_logical": round(place_j["wrist_roll.pos"], 2),
                    "yaw_robot_deg": round(pose.yaw_deg, 1),
                }
                # 偏差
                dx = x - tgt["center_mm"][0]
                dy = y - tgt["center_mm"][1]
                dist = np.hypot(dx, dy)
                exec_rec["place"]["error_vs_target_mm"] = [round(dx, 1), round(dy, 1)]
                exec_rec["place"]["error_dist_mm"] = round(float(dist), 1)
                # 旋转
                if pick_j:
                    pjv = JointVector.from_mapping(pick_j)
                    ppose = KIN.forward(pjv)
                    actual_rot = pose.yaw_deg - ppose.yaw_deg
                    exec_rec["actual_rotation_deg"] = round(float(actual_rot), 1)
                    print(f"  执行{i+1}: pick=({exec_rec['pick']['tcp_table_mm'][0]:.1f},{exec_rec['pick']['tcp_table_mm'][1]:.1f}) "
                          f"place=({x:.1f},{y:.1f}) 偏差={exec_rec['place']['error_dist_mm']}mm "
                          f"实际旋转={exec_rec['actual_rotation_deg']}° (Δyaw={delta_yaw:.1f}°)")

            piece_rec["executions"].append(exec_rec)

        # 最后执行的偏差
        if piece_rec["executions"]:
            last = piece_rec["executions"][-1]
            if "place" in last:
                print(f"  最终: 偏差 {last['place']['error_dist_mm']}mm "
                      f"({last['place']['error_vs_target_mm'][0]:+.1f},{last['place']['error_vs_target_mm'][1]:+.1f})")
        print()

        report["pieces"].append(piece_rec)

    # 汇总
    print("=" * 100)
    print("汇总（最终执行）")
    print("=" * 100)
    print(f"{'片':12s} {'目标位':16s} {'实际落点':16s} {'偏差mm':8s} {'Δyaw':8s} {'实际旋转':10s}")
    for pr in report["pieces"]:
        last = pr["executions"][-1] if pr["executions"] else {}
        if "place" in last:
            t = pr["plan"]["target_table_mm"]
            p = last["place"]["tcp_table_mm"]
            e = last["place"]["error_dist_mm"]
            dy = pr["plan"]["delta_yaw_deg"]
            ar = last.get("actual_rotation_deg", "?")
            print(f"{pr['piece_id']:12s} ({t[0]:6.1f},{t[1]:6.1f}) ({p[0]:6.1f},{p[1]:6.1f}) {e:7.1f} {dy:7.1f} {ar!s:>10}")

    # 写 JSON
    out_json = Path("/home/epuzzle/motion_analysis_v10.json")
    out_json.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"\nJSON 报告: {out_json}")


if __name__ == "__main__":
    main()
