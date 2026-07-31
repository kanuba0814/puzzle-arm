#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""GPIO17 低电平触发继电器的 60 秒高低循环测试。"""

from __future__ import annotations

import argparse
import time

from control import MagnetConfig, MagnetController


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="GPIO17 高/低各保持固定时间；结束后强制恢复高电平关闭继电器。"
    )
    parser.add_argument("--duration", type=float, default=60.0, help="总测试秒数")
    parser.add_argument("--phase", type=float, default=1.0, help="每个电平保持秒数")
    return parser.parse_args()


def validate_arguments(duration: float, phase: float) -> None:
    if not 0 < duration <= 60:
        raise ValueError("总测试时间必须在 0 到 60 秒之间")
    if not 0 < phase <= 10:
        raise ValueError("单个电平保持时间必须在 0 到 10 秒之间")


def run_cycle(duration: float, phase: float) -> None:
    validate_arguments(duration, phase)
    controller = MagnetController(
        MagnetConfig(
            chip_path="/dev/gpiochip0",
            line_offset=17,
            active_high=False,
            max_on_seconds=10.0,
        )
    )
    deadline = time.monotonic() + duration
    cycle = 0
    try:
        controller.connect()  # Active Low：逻辑关闭对应物理高电平。
        print("GPIO17=HIGH，继电器关闭（安全初始状态）", flush=True)
        while time.monotonic() < deadline:
            cycle += 1
            controller.set_enabled(True)
            print(f"[{cycle}] GPIO17=LOW，继电器吸合", flush=True)
            time.sleep(min(phase, max(0.0, deadline - time.monotonic())))
            controller.set_enabled(False)
            print(f"[{cycle}] GPIO17=HIGH，继电器释放", flush=True)
            time.sleep(min(phase, max(0.0, deadline - time.monotonic())))
    finally:
        controller.close()
        print("测试结束：GPIO17=HIGH，继电器已请求关闭", flush=True)


def main() -> int:
    arguments = parse_arguments()
    run_cycle(arguments.duration, arguments.phase)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
