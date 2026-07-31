#!/usr/bin/env python3
"""将电磁铁 GPIO 置为不吸合状态；配置 UNKNOWN 时安全退出。"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

ENV_PATH = Path("/etc/epuzzle/epuzzle.env")


def load_environment() -> dict[str, str]:
    environment: dict[str, str] = {}
    if not ENV_PATH.is_file():
        return environment
    for raw_line in ENV_PATH.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        environment[key] = value
    return environment


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hold", action="store_true", help="持续占用 GPIO 并保持低电平"
    )
    arguments = parser.parse_args()
    environment = load_environment()
    chip = environment.get("EPUZZLE_MAGNET_GPIO_CHIP", "/dev/gpiochip0")
    line = environment.get("EPUZZLE_MAGNET_GPIO_LINE", "UNKNOWN")
    active_high = environment.get("EPUZZLE_MAGNET_ACTIVE_HIGH", "UNKNOWN")
    if line == "UNKNOWN" or active_high == "UNKNOWN":
        print("磁铁 GPIO 未冻结；未请求任何 GPIO，保持外部硬件下拉的默认关闭状态。")
        return 0
    if not line.isdigit():
        print(f"非法 GPIO line: {line!r}", file=sys.stderr)
        return 2
    if active_high not in {"0", "1"}:
        print(f"非法继电器触发极性: {active_high!r}", file=sys.stderr)
        return 2
    if not Path(chip).exists():
        print(f"GPIO chip 不存在: {chip}", file=sys.stderr)
        return 3
    command = [
        "gpioset",
        "--consumer",
        "epuzzle-safe-off",
        "--chip",
        chip,
    ]
    off_value = "0" if active_high == "1" else "1"
    if arguments.hold:
        # 前台进程持续持有 GPIO；由 systemd 追踪并在关机时终止。
        os.execvpe(
            command[0],
            [*command, f"{line}={off_value}"],
            {**os.environ, "LC_ALL": "C.UTF-8"},
        )
        return 127
    completed = subprocess.run(
        ["timeout", "0.05", *command, f"{line}={off_value}"],
        check=False,
        env={**os.environ, "LC_ALL": "C.UTF-8"},
    )
    # timeout 正常终止长期持有的 gpioset 时返回 124；低电平已保持 50 ms。
    return 0 if completed.returncode == 124 else completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
