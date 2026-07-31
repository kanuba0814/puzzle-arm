#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""页面急停辅助：只做五轴断力矩；磁铁由 systemd fail-safe 单独关闭。"""

from pathlib import Path

from lerobot_robot_so101_5d import SO101Follower5D, SO101Follower5DConfig


def main() -> int:
    robot = SO101Follower5D(
        SO101Follower5DConfig(
            port=("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00"),
            id="epuzzle-single-arm",
            calibration_dir=(
                Path.home()
                / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
            ),
            disable_torque_on_disconnect=True,
            use_degrees=True,
        )
    )
    try:
        robot.bus.connect()
        robot.bus.disable_torque(num_retry=2)
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=True)
    print("五轴力矩已关闭")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
