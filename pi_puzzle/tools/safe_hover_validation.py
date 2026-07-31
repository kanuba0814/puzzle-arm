#!/usr/bin/env python3
"""单点高悬停恢复验证；默认仅预检，--execute 才连接硬件。

危险标定修正不会被导入：本工具固定使用物理 L1 和零运动学偏置，
并固定软件关节俯仰和 -90 度。首轮只允许 MNOP 会话中的 M 点；
先到 Z=51mm，再按用户显式 force 请求下降到 Z=21mm（块顶上方模型20mm）。
"""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

config = importlib.import_module("config")
kinematics = importlib.import_module("kinematics")
L1 = config.L1
L2 = config.L2
L3 = config.L3
L4 = config.L4
PARK_SERVO_SPEED = config.PARK_SERVO_SPEED
SERVO_IDS = config.SERVO_IDS
SERVO_LIMITS = config.SERVO_LIMITS
deg_to_servo = kinematics.deg_to_servo
servo_to_deg = kinematics.servo_to_deg

SESSION = ROOT / "out" / "contact_calibration_MNOP.json"
PARK_RAW = {1: 1126, 2: 710, 3: 3277, 4: 2818, 5: 1114}
STABLE_S1_RAW = {1: 1125, 2: 2174, 3: 2357, 4: 2791, 5: 1116}
COMPACT = [88.35, 70.0, -95.0, -75.0, 0.0]
COMPACT_ROTATED = [-88.0, 70.0, -95.0, -75.0, 0.0]
P3 = [-88.0, 61.0, -86.0, -66.0, 0.0]
HIGH_Z_MM = 51.0
FORCED_TARGET_Z_MM = 21.0
PITCH_DEG = -90.0
MIN_TRANSIT_MODEL_Z_MM = 38.0
MIN_FORCED_MODEL_Z_MM = 20.0
MAX_MODEL_RADIUS_MM = 210.0


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"无法读取 {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise RuntimeError(f"JSON 根节点无效: {path}")
    return value


def finite_float(value: Any, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"{name} 不是数值: {value!r}") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"{name} 不是有限数值")
    return result


def load_base(path: Path) -> tuple[float, float, float]:
    data = load_json(path)
    return (
        finite_float(data.get("base_x"), "base_x"),
        finite_float(data.get("base_y"), "base_y"),
        finite_float(data.get("rot_deg"), "rot_deg"),
    )


def load_m(path: Path) -> tuple[float, float]:
    data = load_json(path)
    targets = data.get("targets")
    if not isinstance(targets, dict):
        raise RuntimeError("会话 targets 无效")
    target = targets.get("M")
    if not isinstance(target, list) or len(target) != 3:
        raise RuntimeError("会话 M 目标无效")
    if finite_float(target[2], "M z") != 1.0:
        raise RuntimeError("M 必须是已确认的 1mm 块顶目标")
    return finite_float(target[0], "M x"), finite_float(target[1], "M y")


def world_to_arm(wx: float, wy: float, base_path: Path) -> tuple[float, float]:
    bx, by, rotation = load_base(base_path)
    dx, dy = wx - bx, wy - by
    angle = math.radians(-rotation)
    c, s = math.cos(angle), math.sin(angle)
    return c * dx - s * dy, s * dx + c * dy


def rollback_ik(x: float, y: float, z: float) -> list[float]:
    j1 = math.degrees(math.atan2(y, x))
    radius = math.hypot(x, y)
    theta = math.radians(PITCH_DEG)
    wrist_r = radius - L4 * math.cos(theta)
    wrist_z = z - L1 - L4 * math.sin(theta)
    distance = math.hypot(wrist_r, wrist_z)
    if not abs(L2 - L3) + 1.0 <= distance <= L2 + L3 - 1.0:
        raise RuntimeError(f"M 不可达: wrist distance={distance:.2f}mm")
    cosine = (distance**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    j3 = -math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    alpha = math.atan2(wrist_z, wrist_r)
    beta = math.atan2(
        L3 * math.sin(math.radians(j3)),
        L2 + L3 * math.cos(math.radians(j3)),
    )
    j2 = math.degrees(alpha - beta)
    j4 = PITCH_DEG - j2 - j3
    result = [j1, j2, j3, j4, 0.0]
    for servo_id, angle in zip(SERVO_IDS, result, strict=True):
        low, high = SERVO_LIMITS[servo_id]
        if not low <= angle <= high:
            raise RuntimeError(f"关节 {servo_id} 超限: {angle:.2f}°")
    return result


def fk(joints: list[float]) -> tuple[float, float, float, float]:
    _, j2, j3, j4, _ = joints
    q2 = math.radians(j2)
    q23 = math.radians(j2 + j3)
    pitch = math.radians(j2 + j3 + j4)
    elbow_z = L1 + L2 * math.sin(q2)
    wrist_z = elbow_z + L3 * math.sin(q23)
    radius = L2 * math.cos(q2) + L3 * math.cos(q23) + L4 * math.cos(pitch)
    tcp_z = wrist_z + L4 * math.sin(pitch)
    return radius, tcp_z, wrist_z, math.degrees(pitch)


def raw_to_continuous(raw: dict[int, int]) -> list[float]:
    return [
        finite_float(servo_to_deg(raw[sid], sid), f"舵机 {sid} 软件角度")
        for sid in SERVO_IDS
    ]


def interpolate(start: list[float], end: list[float], steps: int) -> list[list[float]]:
    return [
        [a + (b - a) * index / steps for a, b in zip(start, end, strict=True)]
        for index in range(1, steps + 1)
    ]


def preflight(
    session_path: Path, base_path: Path, resume_s1: bool = False
) -> tuple[list[tuple[str, list[list[float]]]], list[float]]:
    wx, wy = load_m(session_path)
    ax, ay = world_to_arm(wx, wy, base_path)
    high_target = rollback_ik(ax, ay, HIGH_Z_MM)
    forced_target = rollback_ik(ax, ay, FORCED_TARGET_Z_MM)
    park_angles = raw_to_continuous(PARK_RAW)
    if resume_s1:
        stable_s1 = raw_to_continuous(STABLE_S1_RAW)
        rotated_s1 = stable_s1.copy()
        rotated_s1[0] = P3[0]
        route = [
            ("实际S1紧凑旋转", interpolate(stable_s1, rotated_s1, 90)),
            ("实际S1进入P3", interpolate(rotated_s1, P3, 25)),
            ("P3到M高悬停", interpolate(P3, high_target, 70)),
            ("M垂直下降到模型20mm间隙", interpolate(high_target, forced_target, 60)),
        ]
    else:
        route = [
            ("纸外展开", interpolate(park_angles, COMPACT, 70)),
            ("紧凑旋转", interpolate(COMPACT, COMPACT_ROTATED, 90)),
            ("进入P3", interpolate(COMPACT_ROTATED, P3, 25)),
            ("P3到M高悬停", interpolate(P3, high_target, 70)),
            ("M垂直下降到模型20mm间隙", interpolate(high_target, forced_target, 60)),
        ]
    minimum_z = math.inf
    minimum_wrist_z = math.inf
    maximum_radius = 0.0
    for segment_name, samples in route:
        for sample in samples:
            radius, tcp_z, wrist_z, _ = fk(sample)
            minimum_z = min(minimum_z, tcp_z)
            minimum_wrist_z = min(minimum_wrist_z, wrist_z)
            maximum_radius = max(maximum_radius, abs(radius))
            minimum_allowed = (
                MIN_FORCED_MODEL_Z_MM
                if segment_name == "M垂直下降到模型20mm间隙"
                else MIN_TRANSIT_MODEL_Z_MM
            )
            if tcp_z < minimum_allowed:
                raise RuntimeError(f"{segment_name} 模型 TCP 高度过低: {tcp_z:.2f}mm")
            if wrist_z < MIN_TRANSIT_MODEL_Z_MM:
                raise RuntimeError(f"{segment_name} 模型腕轴高度过低: {wrist_z:.2f}mm")
            if abs(radius) > MAX_MODEL_RADIUS_MM:
                raise RuntimeError(f"{segment_name} 模型半径过大: {radius:.2f}mm")
    print(f"M 世界坐标: ({wx:.2f}, {wy:.2f}) mm")
    print(f"M 机械臂坐标: ({ax:.2f}, {ay:.2f}) mm")
    print(f"M 50mm高悬停关节: {[round(value, 3) for value in high_target]}")
    print(f"M 强制20mm间隙关节: {[round(value, 3) for value in forced_target]}")
    print(f"最终俯仰角和: {sum(forced_target[1:4]):.3f}°")
    print(f"全路径模型最低 TCP: {minimum_z:.2f} mm")
    print(f"全路径模型最低腕轴: {minimum_wrist_z:.2f} mm")
    print(f"全路径模型最大半径: {maximum_radius:.2f} mm")
    return route, forced_target


def wait_converged(
    arm: Any, target: dict[int, int], timeout_s: float = 10.0
) -> dict[int, int]:
    deadline = time.monotonic() + timeout_s
    stable = 0
    latest: dict[int, int] = {}
    while time.monotonic() < deadline:
        latest = arm.get_current_position()
        if all(
            sid in latest and abs(latest[sid] - value) <= 40
            for sid, value in target.items()
        ):
            stable += 1
            if stable >= 4:
                return latest
        else:
            stable = 0
        time.sleep(0.15)
    raise RuntimeError(f"位置未收敛: target={target}, actual={latest}")


def readback_only() -> None:
    from arm_controller import ArmController

    arm = ArmController()
    if arm.simulation:
        arm.close()
        raise RuntimeError("串口不可用，拒绝 simulation 读回")
    try:
        arm.open_gripper()
        samples = []
        for _ in range(8):
            samples.append(arm.get_current_position())
            time.sleep(0.2)
        print("只读位置样本:")
        for sample in samples:
            print(sample)
        print("未发送任何舵机位置命令；电磁铁保持断电。")
    finally:
        arm.open_gripper()
        arm.close()


def execute(
    route: list[tuple[str, list[list[float]]]], expected_start: dict[int, int]
) -> None:
    from arm_controller import ArmController

    arm = ArmController()
    if arm.simulation:
        arm.close()
        raise RuntimeError("串口不可用，拒绝 simulation 执行")
    try:
        arm.open_gripper()
        current = arm.get_current_position()
        if any(
            sid not in current or abs(current[sid] - expected_start[sid]) > 8
            for sid in SERVO_IDS
        ):
            raise RuntimeError(
                f"起点不是本次已确认稳定姿态: expected={expected_start}, actual={current}"
            )
        for segment_name, samples in route:
            if not samples:
                raise RuntimeError(f"{segment_name} 没有路径采样")
            print(f"执行: {segment_name}")
            positions: dict[int, int] = {}
            for joints in samples:
                positions = {
                    sid: int(deg_to_servo(angle, sid))
                    for sid, angle in zip(SERVO_IDS, joints, strict=True)
                }
                if not arm.write_positions_sync(positions, speed=PARK_SERVO_SPEED):
                    raise RuntimeError(f"{segment_name} 写入失败")
                time.sleep(0.035)
            actual = wait_converged(arm, positions)
            print(f"  已收敛: {actual}")
        print("已停在 M 模型20mm间隙点；电磁铁保持断电。")
    finally:
        arm.open_gripper()
        arm.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="显式允许实机运动")
    parser.add_argument("--readback", action="store_true", help="只读当前位置，不运动")
    parser.add_argument("--resume-s1", action="store_true", help="从已确认稳定S1继续")
    parser.add_argument("--session", type=Path, default=SESSION)
    parser.add_argument("--base", type=Path, default=ROOT / "arm_base.json")
    args = parser.parse_args()
    if args.readback:
        readback_only()
        return 0
    route, _ = preflight(
        args.session.resolve(), args.base.resolve(), resume_s1=args.resume_s1
    )
    if not args.execute:
        print("DRY-RUN PASS；未初始化串口或 GPIO。")
        return 0
    expected_start = STABLE_S1_RAW if args.resume_s1 else PARK_RAW
    execute(route, expected_start)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
