#!/usr/bin/env python3
"""详细运动日志分析：从 motion_detail/*.jsonl 生成运动细节与偏差分析报告。

分析内容：
1. 每个 goal：目标位姿、规划步数、关节跨度
2. 每个 waypoint：目标关节、实际关节、逐关节误差、收敛读回次数
3. 偏差来源：PICK/PLACE 终点的残余误差（决定碎片落位精度）
4. 汇总：各 goal 最大误差、超时、重试

用法：
    python3 tools/analyze_motion_log.py [--log DIR|FILE] [--out REPORT.md]
"""
import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from control.kinematics import PlanarArmKinematics, JointVector  # noqa: E402
from control.calibration import ArmCalibration  # noqa: E402

FEATURES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
]


def find_logs(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(path.glob("motion-*.jsonl"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("/opt/epuzzle/workspace/logs/motion_detail"),
    )
    parser.add_argument("--out", type=Path, default=Path("/home/epuzzle/motion_detail_report.md"))
    args = parser.parse_args()

    logs = find_logs(args.log)
    if not logs:
        print(f"无日志: {args.log}")
        return 1
    print(f"日志文件: {len(logs)} 个")

    # 加载标定（FK 用）
    cal = ArmCalibration.load(
        "/opt/epuzzle/workspace/calibration/arm_kinematics_single_arm_accepted_20260731.json"
    )
    kin = PlanarArmKinematics(cal.require_accepted_geometry())

    goals: dict[str, dict] = {}
    order: list[str] = []
    for logf in logs:
        for line in logf.read_text().splitlines():
            if not line.strip():
                continue
            rec = json.loads(line)
            ev = rec["event"]
            gid = rec.get("goal_id", "")
            if ev == "goal":
                if gid not in goals:
                    goals[gid] = {"events": [], "file": str(logf)}
                    order.append(gid)
                goals[gid]["events"].append(rec)
            elif gid in goals:
                goals[gid]["events"].append(rec)

    # 生成报告
    lines: list[str] = []
    lines.append("# 运动细节与偏差分析报告")
    lines.append("")
    lines.append(f"- 日志: {args.log} ({len(logs)} 文件)")
    lines.append(f"- 运动目标数: {len(order)}")
    lines.append("")

    summary_rows = []
    for gid in order:
        evs = goals[gid]["events"]
        goal_rec = next((e for e in evs if e["event"] == "goal"), None)
        plan_rec = next((e for e in evs if e["event"] == "plan"), None)
        result_rec = next((e for e in evs if e["event"] == "result"), None)
        if goal_rec is None:
            continue
        phase = goal_rec.get("phase", "?")
        speed = goal_rec.get("speed_profile", "?")
        pos = goal_rec.get("position_mm", [])
        yaw = goal_rec.get("yaw_deg", 0.0)

        lines.append(f"## {gid}  [{phase}/{speed}]")
        lines.append("")
        lines.append(f"- 目标位姿: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})mm yaw={yaw:.1f}°")
        if plan_rec:
            lines.append(
                f"- 规划: {plan_rec['steps']} 步, 关节跨度 {plan_rec['joint_span_deg']}°"
            )
            lines.append(f"  - 起始关节: {json.dumps(plan_rec['start_joints'])}")
            lines.append(f"  - 目标关节: {json.dumps(plan_rec['target_joints'])}")

        # 逐 waypoint 收敛分析
        reads_by_wp: dict[int, list] = defaultdict(list)
        sent_by_wp: dict[int, dict] = {}
        for e in evs:
            if e["event"] == "waypoint_sent":
                sent_by_wp[e["index"]] = e["target_joints"]
            elif e["event"] == "readback":
                reads_by_wp[e["index"]].append(e)
        conv = {e["index"]: e for e in evs if e["event"] == "waypoint_converged"}
        to = {e["index"]: e for e in evs if e["event"] == "waypoint_timeout"}

        if reads_by_wp:
            lines.append("")
            lines.append("| waypoint | 目标 | 最终实际 | 最大误差° | 读回次数 | 状态 |")
            lines.append("|---|---|---|---|---|---|")
            for idx in sorted(set(list(sent_by_wp.keys()) + list(reads_by_wp.keys()))):
                target = sent_by_wp.get(idx, {})
                rds = reads_by_wp.get(idx, [])
                if not rds:
                    lines.append(f"| {idx} | {json.dumps(target)} | - | - | 0 | 未读回 |")
                    continue
                last = rds[-1]
                actual = last["actual_joints"]
                err = last["error_deg"]
                max_err = max(err.values()) if err else float("nan")
                # 最终实际关节（取最后一次读回）
                status = "✓ 收敛" if idx in conv else ("✗ 超时" if idx in to else "?")
                lines.append(
                    f"| {idx} | {json.dumps(target)} | {json.dumps(actual)} | "
                    f"{max_err:.2f} | {len(rds)} | {status} |"
                )

        # 终点（PICK/PLACE 精度关键）
        if result_rec:
            final_j = result_rec.get("final_joints", {})
            lines.append("")
            lines.append(
                f"- **结果**: {'✓ 成功' if result_rec['success'] else '✗ 失败'} "
                f"({result_rec['error_code']}), 执行 {result_rec['executed']}/{result_rec['planned']} "
                f"步, 耗时 {result_rec['duration_s']}s"
            )
            lines.append(f"  - 最终关节: {json.dumps(final_j)}")
            # 与目标关节误差
            if plan_rec:
                tj = plan_rec.get("target_joints", {})
                errs = {}
                for k in FEATURES:
                    if k in tj and k in final_j:
                        errs[k] = abs((final_j[k] - tj[k] + 180) % 360 - 180)
                if errs:
                    lines.append(f"  - 终点残余误差: {json.dumps({k: round(v, 3) for k, v in errs.items()})}")
                    max_e = max(errs.values())
                    summary_rows.append((gid, phase, max_e, result_rec["success"]))
        lines.append("")

    # 汇总表
    lines.append("# 汇总：终点残余误差（决定碎片落位精度）")
    lines.append("")
    lines.append("| goal | phase | 最大终点误差° | 成功 |")
    lines.append("|---|---|---|---|")
    for gid, phase, err, ok in summary_rows:
        lines.append(f"| {gid} | {phase} | {err:.2f} | {'✓' if ok else '✗'} |")
    lines.append("")

    report = "\n".join(lines)
    args.out.write_text(report, encoding="utf-8")
    print(f"报告已写入: {args.out}")
    print(report[:2000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
