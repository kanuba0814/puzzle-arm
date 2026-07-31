#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""用机械端点计算中位的五轴 SO-101 follower 标定向导。

默认仅预检。实际流程只读取人工手掰位置并写入 Homing_Offset/范围；不会发送
Goal_Position。J1–J4 依次记录两个机械端点，中位 raw 由软件计算，不要求刻度或
目测中位；J5 由用户选择电缆安全的零向并保留完整一圈。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import statistics
import sys
import time
from pathlib import Path

from control.motor_calibration import ENCODER_MAX, HALF_TURN, center_motor_range
from lerobot.motors import MotorCalibration
from lerobot_robot_so101_5d import (
    JOINT_NAMES,
    SO101Follower5D,
    SO101Follower5DConfig,
)

CONFIRM_TEXT = "CALIBRATE-SINGLE-FOLLOWER-5D"
WRITE_CONFIRM_TEXT = "WRITE-CENTERED-CALIBRATION"
DEFAULT_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00")
DEFAULT_CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
ENDPOINT_JOINTS = JOINT_NAMES[:4]
JOINT_LABELS = {
    "shoulder_pan": "J1 底座旋转",
    "shoulder_lift": "J2 肩关节",
    "elbow_flex": "J3 肘关节",
    "wrist_flex": "J4 腕俯仰",
    "wrist_roll": "J5 腕旋转",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="由机械端点自动计算中位；单台原 follower 改装五轴"
    )
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument("--robot-id", default="epuzzle-single-arm")
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--minimum-span-counts", type=int, default=500)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际连接、释放力矩并记录端点；默认只预检",
    )
    parser.add_argument(
        "--replace-existing",
        action="store_true",
        help="备份并替换同 robot-id 的已有五轴标定",
    )
    parser.add_argument("--confirm", default="")
    return parser


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as calibration_file:
            for block in iter(lambda: calibration_file.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise RuntimeError(f"无法读取标定文件 {path}: {exc}") from exc
    return digest.hexdigest()


def validate_saved_calibration(path: Path) -> None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取 LeRobot 标定文件 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"LeRobot 标定文件不是有效 JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("LeRobot 标定文件顶层必须是对象")
    actual = set(payload)
    expected = set(JOINT_NAMES)
    if actual != expected:
        raise RuntimeError(
            "LeRobot 标定不是精确五轴；"
            f" missing={sorted(expected - actual)}, extra={sorted(actual - expected)}"
        )


def stable_raw_position(robot: SO101Follower5D, motor: str) -> int:
    samples: list[int] = []
    for _ in range(11):
        value = robot.bus.read("Present_Position", motor, normalize=False)
        try:
            sample = int(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise RuntimeError(
                f"{JOINT_LABELS[motor]} 返回无效 raw: {value!r}"
            ) from exc
        if not 0 <= sample <= ENCODER_MAX:
            raise RuntimeError(f"{JOINT_LABELS[motor]} raw 超出 0..4095: {sample}")
        samples.append(sample)
        time.sleep(0.04)
    spread = max(samples) - min(samples)
    if spread > 8:
        raise RuntimeError(
            f"{JOINT_LABELS[motor]} 读数不稳定，11 次跨度 {spread} counts"
        )
    try:
        return int(statistics.median(samples))
    except (TypeError, ValueError, OverflowError) as exc:
        raise RuntimeError(f"{JOINT_LABELS[motor]} 无法计算稳定中值") from exc


def prompt_endpoint(robot: SO101Follower5D, motor: str, endpoint: str) -> int:
    input(
        f"\n请托住机械臂，把【{JOINT_LABELS[motor]}】缓慢移到机械端点 {endpoint}。"
        "\n不要用蛮力；其他关节保持稳定。到位后按 Enter 记录 >>> "
    )
    value = stable_raw_position(robot, motor)
    print(f"已记录 {JOINT_LABELS[motor]} 端点 {endpoint}: raw={value}")
    return value


def backup_existing(path: Path) -> Path:
    suffix = time.strftime("%Y%m%dT%H%M%S")
    backup_path = path.with_name(f"{path.stem}.pre-centered-{suffix}.json")
    try:
        shutil.copy2(path, backup_path)
    except OSError as exc:
        raise RuntimeError(f"无法备份已有标定 {path}: {exc}") from exc
    return backup_path


def write_evidence(
    path: Path,
    endpoint_records: dict[str, dict[str, int]],
    wrist_roll_neutral_raw: int,
    calibrations: dict[str, MotorCalibration],
    calibration_sha256: str,
) -> Path:
    evidence_path = path.with_suffix(".evidence.json")
    evidence = {
        "schema_version": 1,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "robot_type": "so101_follower_5d",
        "source_arm": "so101_follower_modified_single_arm",
        "method": "two mechanical endpoints -> computed raw midpoint",
        "goal_position_commands_sent": 0,
        "joint_order": list(JOINT_NAMES),
        "endpoint_records_raw": endpoint_records,
        "wrist_roll_neutral_raw": wrist_roll_neutral_raw,
        "computed_calibration": {
            motor: {
                "id": calibration.id,
                "drive_mode": calibration.drive_mode,
                "homing_offset": calibration.homing_offset,
                "range_min": calibration.range_min,
                "range_max": calibration.range_max,
            }
            for motor, calibration in calibrations.items()
        },
        "calibration_file": path.name,
        "calibration_sha256": calibration_sha256,
        "note": "Motor calibration only; TCP/kinematic calibration remains required.",
    }
    try:
        evidence_path.write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except OSError as exc:
        raise RuntimeError(f"无法写入标定证据 {evidence_path}: {exc}") from exc
    return evidence_path


def main() -> int:
    args = build_parser().parse_args()
    port = args.port.expanduser()
    if not port.is_absolute():
        port = Path.cwd() / port
    calibration_dir = args.calibration_dir.expanduser().resolve(strict=False)
    calibration_path = calibration_dir / f"{args.robot_id}.json"
    print("机械臂拓扑: 单台 SO-101 follower 改装五轴")
    print(f"固定关节: {', '.join(JOINT_NAMES)}")
    print(f"串口: {port}")
    print(f"LeRobot 标定文件: {calibration_path}")
    print("方法: J1–J4 两端点计算 raw 中位；J5 选择电缆安全零向。")
    print("本工具不会发送 Goal_Position。")
    if not port.exists():
        print(f"拒绝：串口不存在: {port}", file=sys.stderr)
        return 2
    if calibration_path.exists() and not args.replace_existing:
        print(
            "拒绝：已有同 ID 标定；若确需替换，请加 --replace-existing，程序会先备份。",
            file=sys.stderr,
        )
        return 2
    if not args.execute:
        print("预检通过；未连接舵机、未修改力矩、未发送位置目标。")
        return 0
    if args.confirm != CONFIRM_TEXT:
        print(
            f"拒绝：实际标定必须传 --confirm {CONFIRM_TEXT}",
            file=sys.stderr,
        )
        return 2
    if not sys.stdin.isatty():
        print("拒绝：实际标定必须在交互式终端中运行", file=sys.stderr)
        return 2
    if args.minimum_span_counts <= 0:
        print("拒绝：--minimum-span-counts 必须为正整数", file=sys.stderr)
        return 2

    print("\n安全要求：机械臂旁必须有人全程托住各连杆。")
    print("流程会释放五轴力矩；只手掰和读位置，不发送位置目标。")
    final_confirmation = input(f"输入 {CONFIRM_TEXT} 再开始: ").strip()
    if final_confirmation != CONFIRM_TEXT:
        print("已取消；未连接机械臂。")
        return 2

    config = SO101Follower5DConfig(
        port=str(port),
        id=args.robot_id,
        calibration_dir=calibration_dir,
        disable_torque_on_disconnect=True,
        use_degrees=True,
    )
    robot = SO101Follower5D(config)
    endpoint_records: dict[str, dict[str, int]] = {}
    written = False
    backup_path: Path | None = None
    try:
        robot.bus.connect()
        robot.bus.disable_torque()
        if calibration_path.exists():
            backup_path = backup_existing(calibration_path)
            print(f"已有标定已备份: {backup_path}")
        robot.bus.reset_calibration()
        print("力矩已释放，Homing_Offset 已临时归零；请持续托住机械臂。")

        centered = {}
        for motor in ENDPOINT_JOINTS:
            endpoint_a = prompt_endpoint(robot, motor, "A")
            endpoint_b = prompt_endpoint(robot, motor, "B")
            result = center_motor_range(
                endpoint_a,
                endpoint_b,
                minimum_span_counts=args.minimum_span_counts,
            )
            centered[motor] = result
            endpoint_records[motor] = {
                "endpoint_a": endpoint_a,
                "endpoint_b": endpoint_b,
                "endpoint_min": result.endpoint_min_raw,
                "endpoint_max": result.endpoint_max_raw,
                "long_arc_span": result.span_counts,
                "wraps_encoder_zero": result.wraps_encoder_zero,
                "midpoint": result.midpoint_raw,
            }
            print(
                f"{JOINT_LABELS[motor]}: long-arc span={result.span_counts}, "
                f"wraps_zero={result.wraps_encoder_zero}, "
                f"computed midpoint={result.midpoint_raw}"
            )

        input(
            "\n把【J5 腕旋转】放到电缆不扭转、希望定义为 0° 的中性朝向，"
            "然后按 Enter 记录 >>> "
        )
        wrist_roll_neutral_raw = stable_raw_position(robot, "wrist_roll")
        print(f"已记录 J5 中性朝向: raw={wrist_roll_neutral_raw}")

        calibrations: dict[str, MotorCalibration] = {}
        for motor_id, motor in enumerate(ENDPOINT_JOINTS, start=1):
            result = centered[motor]
            calibrations[motor] = MotorCalibration(
                id=motor_id,
                drive_mode=0,
                homing_offset=result.homing_offset,
                range_min=result.range_min,
                range_max=result.range_max,
            )
        calibrations["wrist_roll"] = MotorCalibration(
            id=5,
            drive_mode=0,
            homing_offset=wrist_roll_neutral_raw - HALF_TURN,
            range_min=0,
            range_max=ENCODER_MAX,
        )

        print("\n计算结果（尚未写入）：")
        for motor in JOINT_NAMES:
            calibration = calibrations[motor]
            print(
                f"  {JOINT_LABELS[motor]}: homing={calibration.homing_offset:+d}, "
                f"range=[{calibration.range_min}, {calibration.range_max}]"
            )
        write_confirmation = input(
            f"输入 {WRITE_CONFIRM_TEXT} 才写入舵机和文件: "
        ).strip()
        if write_confirmation != WRITE_CONFIRM_TEXT:
            print("已取消写入；未生成新标定文件，舵机保持断力矩。")
            return 2

        robot.calibration = calibrations
        robot.bus.write_calibration(calibrations)
        robot._save_calibration()
        written = True
        validate_saved_calibration(robot.calibration_fpath)
        digest = sha256_file(robot.calibration_fpath)
        checksum_path = robot.calibration_fpath.with_suffix(".json.sha256")
        checksum_path.write_text(
            f"{digest}  {robot.calibration_fpath.name}\n", encoding="utf-8"
        )
        evidence_path = write_evidence(
            robot.calibration_fpath,
            endpoint_records,
            wrist_roll_neutral_raw,
            calibrations,
            digest,
        )
        print(f"五轴 LeRobot 标定已保存: {robot.calibration_fpath}")
        print(f"证据: {evidence_path}")
        print(f"SHA256: {digest}")
        print("下一步仍需 TCP/运动学触点标定；当前文件不能单独解锁运动。")
        return 0
    except KeyboardInterrupt:
        print("\n已中止；舵机保持断力矩，请先把机械臂放稳。", file=sys.stderr)
        return 130
    finally:
        if robot.bus.is_connected:
            robot.bus.disable_torque()
            robot.bus.disconnect(disable_torque=True)
        if not written and backup_path is not None:
            print(f"原标定备份保留在: {backup_path}")


if __name__ == "__main__":
    raise SystemExit(main())
