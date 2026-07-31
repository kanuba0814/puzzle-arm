#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""单臂运动控制只读 preflight；不打开串口、不发送舵机目标。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from control.calibration import ArmCalibration
from control.kinematics import JOINT_FEATURES

DEFAULT_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00")
DEFAULT_LEROBOT_CALIBRATION = (
    Path.home()
    / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
    / "epuzzle-single-arm.json"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="校验单臂串口、LeRobot 五轴标定和运动学质量门禁"
    )
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument(
        "--lerobot-calibration", type=Path, default=DEFAULT_LEROBOT_CALIBRATION
    )
    parser.add_argument("--kinematic-calibration", type=Path, required=True)
    return parser


def load_lerobot_joint_names(path: Path) -> tuple[str, ...]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取 LeRobot 标定 {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"LeRobot 标定不是有效 JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("LeRobot 标定顶层必须是对象")
    return tuple(payload)


def main() -> int:
    args = build_parser().parse_args()
    port = args.port.expanduser()
    if not port.is_absolute():
        port = Path.cwd() / port
    lerobot_path = args.lerobot_calibration.expanduser().resolve(strict=False)
    kinematic_path = args.kinematic_calibration.expanduser().resolve(strict=False)
    try:
        if not port.exists():
            raise RuntimeError(f"机械臂串口不存在: {port}")
        joint_names = load_lerobot_joint_names(lerobot_path)
        expected_names = tuple(
            feature.removesuffix(".pos") for feature in JOINT_FEATURES
        )
        if set(joint_names) != set(expected_names) or len(joint_names) != 5:
            raise RuntimeError(
                "LeRobot 标定不是精确五轴；"
                f" expected={expected_names}, actual={joint_names}"
            )
        calibration = ArmCalibration.load(kinematic_path)
        geometry = calibration.require_accepted_geometry()
    except (RuntimeError, ValueError) as exc:
        print(f"NOT_READY: {exc}", file=sys.stderr)
        print("未打开串口、未改变力矩、未发送位置目标。", file=sys.stderr)
        return 2

    print("READY: 单臂五轴运动控制静态门禁通过")
    print(f"port={port}")
    print(f"lerobot_calibration={lerobot_path}")
    print(f"kinematic_calibration={kinematic_path}")
    print(f"calibration_id={geometry.calibration_id}")
    print("未打开串口、未改变力矩、未发送位置目标。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
