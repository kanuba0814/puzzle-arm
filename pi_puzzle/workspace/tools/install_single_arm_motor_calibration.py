#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""安全导入同一台 follower 的既有 LeRobot 五轴电机标定。"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

from lerobot.motors import MotorCalibration
from lerobot_robot_so101_5d import (
    JOINT_NAMES,
    SO101Follower5D,
    SO101Follower5DConfig,
)

CONFIRM_TEXT = "IMPORT-SAME-FOLLOWER-5D-CALIBRATION"
DEFAULT_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00")
DEFAULT_CALIBRATION_DIR = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
CALIBRATION_FIELDS = {
    "id",
    "drive_mode",
    "homing_offset",
    "range_min",
    "range_max",
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="导入经同一实物身份确认的精确五轴 LeRobot 标定"
    )
    parser.add_argument("source", type=Path)
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument("--robot-id", default="epuzzle-single-arm")
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--source-bundle-sha256", default="")
    parser.add_argument("--replace-existing", action="store_true")
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


def _strict_int(entry: dict[str, object], field: str, motor: str) -> int:
    value = entry.get(field)
    if isinstance(value, bool) or not isinstance(value, int):
        raise RuntimeError(f"{motor}.{field} 必须是整数")
    return value


def load_exact_five_axis_calibration(
    path: Path,
) -> dict[str, MotorCalibration]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取来源标定 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"来源标定不是有效 JSON: {exc}") from exc
    if not isinstance(payload, dict) or set(payload) != set(JOINT_NAMES):
        actual = set(payload) if isinstance(payload, dict) else set()
        raise RuntimeError(
            "来源必须是精确五轴，禁止 gripper/ID6；"
            f" missing={sorted(set(JOINT_NAMES) - actual)}, "
            f"extra={sorted(actual - set(JOINT_NAMES))}"
        )

    calibrations: dict[str, MotorCalibration] = {}
    for expected_id, motor in enumerate(JOINT_NAMES, start=1):
        raw_entry = payload[motor]
        if not isinstance(raw_entry, dict) or set(raw_entry) != CALIBRATION_FIELDS:
            raise RuntimeError(f"{motor} 字段必须精确为 {sorted(CALIBRATION_FIELDS)}")
        entry: dict[str, object] = raw_entry
        motor_id = _strict_int(entry, "id", motor)
        drive_mode = _strict_int(entry, "drive_mode", motor)
        homing_offset = _strict_int(entry, "homing_offset", motor)
        range_min = _strict_int(entry, "range_min", motor)
        range_max = _strict_int(entry, "range_max", motor)
        if motor_id != expected_id:
            raise RuntimeError(f"{motor}.id={motor_id}，期望固定五轴 ID {expected_id}")
        if drive_mode != 0:
            raise RuntimeError(f"{motor}.drive_mode 必须为 0")
        if not -4095 <= homing_offset <= 4095:
            raise RuntimeError(f"{motor}.homing_offset 超出 -4095..4095")
        if not 0 <= range_min < range_max <= 4095:
            raise RuntimeError(f"{motor} 范围必须满足 0 <= min < max <= 4095")
        calibrations[motor] = MotorCalibration(
            id=motor_id,
            drive_mode=drive_mode,
            homing_offset=homing_offset,
            range_min=range_min,
            range_max=range_max,
        )
    return calibrations


def backup_existing(path: Path) -> Path:
    suffix = time.strftime("%Y%m%dT%H%M%S")
    backup_path = path.with_name(f"{path.stem}.pre-import-{suffix}.json")
    try:
        shutil.copy2(path, backup_path)
    except OSError as exc:
        raise RuntimeError(f"无法备份已有标定 {path}: {exc}") from exc
    return backup_path


def write_evidence(
    target: Path,
    source: Path,
    source_sha256: str,
    source_bundle_sha256: str,
) -> Path:
    evidence_path = target.with_suffix(".import-evidence.json")
    payload = {
        "schema_version": 1,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "robot_type": "so101_follower_5d",
        "source_arm": "so101_follower_modified_single_arm",
        "method": "import exact-five-axis calibration from attested same follower",
        "goal_position_commands_sent": 0,
        "joint_order": list(JOINT_NAMES),
        "source_file_name": source.name,
        "source_file_sha256": source_sha256,
        "source_bundle_sha256": source_bundle_sha256 or None,
        "target_file_name": target.name,
        "target_file_sha256": sha256_file(target),
        "scope": "motor normalization only; not accepted TCP/kinematic calibration",
    }
    try:
        evidence_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    except OSError as exc:
        raise RuntimeError(f"无法写入导入证据 {evidence_path}: {exc}") from exc
    return evidence_path


def main() -> int:
    args = build_parser().parse_args()
    source = args.source.expanduser().resolve(strict=False)
    port = args.port.expanduser().resolve(strict=False)
    calibration_dir = args.calibration_dir.expanduser().resolve(strict=False)
    target = calibration_dir / f"{args.robot_id}.json"
    try:
        calibrations = load_exact_five_axis_calibration(source)
        source_sha256 = sha256_file(source)
    except RuntimeError as exc:
        print(f"拒绝：{exc}", file=sys.stderr)
        return 2

    print(f"来源: {source}")
    print(f"来源 SHA256: {source_sha256}")
    print(f"目标: {target}")
    print(f"固定关节: {', '.join(JOINT_NAMES)}")
    print("操作范围: 只写 Homing_Offset/范围及标定文件，不发送 Goal_Position。")
    if not port.exists():
        print(f"拒绝：串口不存在: {port}", file=sys.stderr)
        return 2
    if target.exists() and not args.replace_existing:
        print("拒绝：目标已存在；替换时必须加 --replace-existing", file=sys.stderr)
        return 2
    if not args.execute:
        print("预检通过；未连接总线、未修改力矩或标定。")
        return 0
    if args.confirm != CONFIRM_TEXT:
        print(f"拒绝：必须传 --confirm {CONFIRM_TEXT}", file=sys.stderr)
        return 2
    if not sys.stdin.isatty():
        print("拒绝：实际导入必须在交互式终端中运行", file=sys.stderr)
        return 2
    confirmation = input(
        "确认来源属于当前同一台 follower，且机械臂已托稳。"
        f"\n输入 {CONFIRM_TEXT} 才写入: "
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
    backup_path: Path | None = None
    try:
        robot.bus.connect()
        robot.bus.disable_torque()
        if target.exists():
            backup_path = backup_existing(target)
            print(f"已有目标已备份: {backup_path}")
        robot.calibration = calibrations
        robot.bus.write_calibration(calibrations)
        readback = robot.bus.read_calibration()
        if readback != calibrations:
            raise RuntimeError(
                f"舵机标定回读不一致: expected={calibrations}, actual={readback}"
            )
        robot._save_calibration()
        saved = load_exact_five_axis_calibration(robot.calibration_fpath)
        if saved != calibrations:
            raise RuntimeError("保存后的五轴标定与来源不一致")
        evidence_path = write_evidence(
            robot.calibration_fpath,
            source,
            source_sha256,
            args.source_bundle_sha256,
        )
        print(f"导入完成: {robot.calibration_fpath}")
        print(f"回读一致；证据: {evidence_path}")
        print(f"目标 SHA256: {sha256_file(robot.calibration_fpath)}")
        print("舵机保持断力矩；TCP/运动学 accepted 门禁仍未解锁。")
        return 0
    finally:
        if robot.bus.is_connected:
            robot.bus.disable_torque()
            robot.bus.disconnect(disable_torque=True)
        if backup_path is not None:
            print(f"原标定备份保留在: {backup_path}")


if __name__ == "__main__":
    raise SystemExit(main())
