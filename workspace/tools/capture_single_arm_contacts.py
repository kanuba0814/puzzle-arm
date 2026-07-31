#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""断力矩、磁铁关闭条件下，人工对中触点并只读采集五轴位置。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any

from lerobot_robot_so101_5d import (
    JOINT_NAMES,
    SO101Follower5D,
    SO101Follower5DConfig,
)

CONFIRM_TEXT = "CAPTURE-CONTACTS-MAGNET-OFF"
DEFAULT_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00")
DEFAULT_CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
JOINT_FEATURES = tuple(f"{name}.pos" for name in JOINT_NAMES)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="单臂五轴人工触点只读采集")
    parser.add_argument("plan", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument("--robot-id", default="epuzzle-single-arm")
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source_file:
            for block in iter(lambda: source_file.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise RuntimeError(f"无法读取 {path}: {exc}") from exc
    return digest.hexdigest()


def finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{field} 必须是有限数值")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"{field} 无法转换为有限数值") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"{field} 必须是有限数值")
    return result


def load_plan(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取采点计划 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"采点计划不是有效 JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("采点计划顶层必须是对象")
    if payload.get("schema_version") != 1:
        raise RuntimeError("采点计划 schema_version 必须为 1")
    if payload.get("frame") != "calibration_sheet_mm":
        raise RuntimeError("采点计划 frame 必须是 calibration_sheet_mm")
    alignment = payload.get("tcp_alignment")
    if not isinstance(alignment, dict):
        raise RuntimeError("tcp_alignment 必须是对象")
    magnet_activation_allowed = alignment.get("magnet_activation_allowed")
    if not isinstance(magnet_activation_allowed, bool) or magnet_activation_allowed:
        raise RuntimeError("采点计划必须显式禁止磁铁通电")
    targets = payload.get("targets")
    if not isinstance(targets, list):
        raise RuntimeError("targets 必须是数组")

    labels: set[str] = set()
    role_counts = {"fit": 0, "holdout": 0}
    normalized_targets: list[dict[str, object]] = []
    for index, raw_target in enumerate(targets):
        if not isinstance(raw_target, dict):
            raise RuntimeError(f"targets[{index}] 必须是对象")
        label = raw_target.get("label")
        role = raw_target.get("role")
        position = raw_target.get("position_mm")
        if not isinstance(label, str) or not label or label in labels:
            raise RuntimeError(f"targets[{index}].label 必须是非空唯一字符串")
        if role not in role_counts:
            raise RuntimeError(f"targets[{index}].role 必须是 fit 或 holdout")
        if not isinstance(position, list) or len(position) != 3:
            raise RuntimeError(f"targets[{index}].position_mm 必须恰好有三个数")
        normalized_position = [
            finite_number(value, f"targets[{index}].position_mm[{axis}]")
            for axis, value in enumerate(position)
        ]
        labels.add(label)
        role_counts[str(role)] += 1
        normalized_targets.append(
            {"label": label, "role": role, "position_mm": normalized_position}
        )
    if role_counts["fit"] < 3 or role_counts["holdout"] < 4:
        raise RuntimeError("采点计划至少需要 3 个 fit 和 4 个 holdout")
    payload["targets"] = normalized_targets
    return payload


def atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    except OSError as exc:
        raise RuntimeError(f"无法写入采点证据 {path}: {exc}") from exc


def stable_sample(
    robot: SO101Follower5D,
) -> tuple[list[list[int]], dict[str, int], dict[str, int], dict[str, float]]:
    rows: list[list[int]] = []
    for _ in range(11):
        readings = robot.bus.sync_read("Present_Position", normalize=False)
        if set(readings) != set(JOINT_NAMES):
            raise RuntimeError("舵机读回不是精确五轴")
        row: list[int] = []
        for motor in JOINT_NAMES:
            value = readings[motor]
            try:
                raw = int(value)
            except (TypeError, ValueError, OverflowError) as exc:
                raise RuntimeError(f"{motor} 返回无效 raw: {value!r}") from exc
            if not 0 <= raw <= 4095:
                raise RuntimeError(f"{motor} raw 超出 0..4095: {raw}")
            row.append(raw)
        rows.append(row)
        time.sleep(0.05)

    medians: dict[str, int] = {}
    spreads: dict[str, int] = {}
    joint_deg: dict[str, float] = {}
    for index, motor in enumerate(JOINT_NAMES):
        values = [row[index] for row in rows]
        try:
            median = int(statistics.median(values))
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(f"{motor} 无法计算中值") from exc
        spread = max(values) - min(values)
        if spread > 12:
            raise RuntimeError(f"{motor} 采样不稳定：spread={spread} counts")
        calibration = robot.calibration[motor]
        if not calibration.range_min <= median <= calibration.range_max:
            raise RuntimeError(
                f"{motor}={median} 超出标定范围 "
                f"[{calibration.range_min}, {calibration.range_max}]"
            )
        midpoint = (calibration.range_min + calibration.range_max) / 2.0
        medians[motor] = median
        spreads[motor] = spread
        joint_deg[motor] = (median - midpoint) * 360.0 / 4095.0
    return rows, medians, spreads, joint_deg


def main() -> int:
    args = build_parser().parse_args()
    plan_path = args.plan.expanduser().resolve(strict=False)
    output_path = args.output.expanduser().resolve(strict=False)
    port = args.port.expanduser().resolve(strict=False)
    calibration_dir = args.calibration_dir.expanduser().resolve(strict=False)
    calibration_path = calibration_dir / f"{args.robot_id}.json"
    try:
        plan = load_plan(plan_path)
        plan_sha256 = sha256_file(plan_path)
        calibration_sha256 = sha256_file(calibration_path)
    except RuntimeError as exc:
        print(f"拒绝：{exc}", file=sys.stderr)
        return 2
    if output_path.exists():
        print(f"拒绝：输出已存在，不覆盖: {output_path}", file=sys.stderr)
        return 2
    if not port.exists():
        print(f"拒绝：串口不存在: {port}", file=sys.stderr)
        return 2

    targets = plan["targets"]
    print(f"计划: {plan_path} ({plan_sha256})")
    print(f"输出: {output_path}")
    print(f"目标数: {len(targets)}；固定五轴: {', '.join(JOINT_NAMES)}")
    print("只读 Present_Position；不发送 Goal_Position；不控制电磁铁。")
    if not args.execute:
        print("预检通过；未连接总线、未改变力矩、未采点。")
        return 0
    if args.confirm != CONFIRM_TEXT:
        print(f"拒绝：必须传 --confirm {CONFIRM_TEXT}", file=sys.stderr)
        return 2
    if not sys.stdin.isatty():
        print("拒绝：实际采点必须在交互式终端运行", file=sys.stderr)
        return 2
    confirmation = input(
        "确认磁铁已断电、机械臂由人在旁托住、触点纸关键尺寸已复核。"
        f"\n输入 {CONFIRM_TEXT} 才连接: "
    ).strip()
    if confirmation != CONFIRM_TEXT:
        print("已取消；未连接总线。")
        return 2

    config = SO101Follower5DConfig(
        port=str(port),
        id=args.robot_id,
        calibration_dir=calibration_dir,
        disable_torque_on_disconnect=True,
        use_degrees=True,
    )
    robot = SO101Follower5D(config)
    evidence: dict[str, object] = {
        "schema_version": 1,
        "capture_status": "in_progress",
        "fixture_id": plan.get("fixture_id"),
        "target_frame": plan.get("frame"),
        "position_unit": "mm",
        "joint_frame": "joint_deg",
        "joint_features": list(JOINT_FEATURES),
        "motor_calibration_file": str(calibration_path),
        "motor_calibration_sha256": calibration_sha256,
        "plan_file": str(plan_path),
        "plan_sha256": plan_sha256,
        "goal_position_commands_sent": 0,
        "magnet_activated": False,
        "points": [],
    }
    try:
        robot.bus.connect()
        robot.bus.disable_torque()
        if robot.bus.read_calibration() != robot.calibration:
            raise RuntimeError("舵机标定寄存器与五轴文件不一致")
        for target in targets:
            label = target["label"]
            role = target["role"]
            position = target["position_mm"]
            while True:
                input(
                    f"\n[{label} / {role}] 将磁铁工作面平放，使 20 mm 外缘"
                    f"对准圆或与方框四边中点相切；目标 {position}；"
                    "保持不动后按 Enter 采样 >>> "
                )
                try:
                    rows, medians, spreads, joint_deg = stable_sample(robot)
                except RuntimeError as exc:
                    print(f"本次未保存：{exc}")
                    continue
                print(f"raw median: {medians}")
                print(
                    "joint_deg: "
                    + ", ".join(
                        f"{motor}={joint_deg[motor]:.4f}" for motor in JOINT_NAMES
                    )
                )
                decision = (
                    input("保存该点? [y=保存/r=重采/q=安全中止] ").strip().lower()
                )
                if decision == "r":
                    continue
                if decision == "q":
                    evidence["capture_status"] = "aborted_by_operator"
                    atomic_write_json(output_path, evidence)
                    return 2
                if decision != "y":
                    print("输入无效，本点重新采集。")
                    continue
                point = {
                    "label": label,
                    "role": role,
                    "target": {
                        "frame": plan["frame"],
                        "position_mm": position,
                    },
                    "samples_raw": rows,
                    "median_raw": medians,
                    "spread_raw_counts": spreads,
                    "joint_deg": joint_deg,
                    "timestamp_ns": time.time_ns(),
                }
                points = evidence["points"]
                if not isinstance(points, list):
                    raise RuntimeError("内部错误：points 不是数组")
                points.append(point)
                atomic_write_json(output_path, evidence)
                break
        evidence["capture_status"] = "complete"
        evidence["completed_timestamp_ns"] = time.time_ns()
        atomic_write_json(output_path, evidence)
        print(f"采点完成: {output_path}")
        print(f"SHA256: {sha256_file(output_path)}")
        print("舵机保持断力矩；数据尚未拟合或通过 holdout，不能解锁运动。")
        return 0
    except KeyboardInterrupt:
        evidence["capture_status"] = "interrupted"
        atomic_write_json(output_path, evidence)
        print(f"\n已中止；部分证据保存在 {output_path}", file=sys.stderr)
        return 130
    finally:
        if robot.bus.is_connected:
            robot.bus.disable_torque()
            robot.bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    raise SystemExit(main())
