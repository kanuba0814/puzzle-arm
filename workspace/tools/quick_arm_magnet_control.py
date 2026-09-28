#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""快速标定辅助工具：把机械臂从正式 PARK 抬到 HOME，并提供电磁铁手动控制。

用法（在树莓派上）：
    sudo -E /opt/epuzzle/venv/bin/python \
        /opt/epuzzle/workspace/tools/quick_arm_magnet_control.py

流程：
1. 检查运动锁 /etc/epuzzle/robot-motion.lock.json；被锁定时拒绝任何运动。
2. bumpless 连接机械臂，确认当前处于正式 PARK 容差内。
3. 关节空间经直立位抬到 HOME（不做笛卡尔规划，不下探）。
4. 进入命令行电磁铁控制循环。
5. 退出时先断电磁铁，再回正式 PARK，最后断力矩。

安全约束：
- 电磁铁通电后保持，必须由操作者显式输入 off/0 或退出；异常和急停路径会断电。
- 抬臂期间不断力矩；整个受监控序列由本进程连续持有。
- 若不在 PARK，本工具不猜测姿态，直接拒绝并提示现场处理。
- 使用 --magnet-only 可跳过一切运动，只做电磁铁通断测试。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / "src"
        / "robot_plugin"
        / "lerobot_robot_so101_5d"
    ),
)

from control.magnet_controller import MagnetConfig, MagnetController  # noqa: E402
from control.motion_controller import MotionConfig  # noqa: E402
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402

WORKSPACE = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = (
    WORKSPACE / "calibration" / "arm_kinematics_single_arm_accepted_20260731.json"
)
TRANSFORM_PATH = WORKSPACE / "calibration" / "contact_sheet_to_robot_base_20260731.json"
PARK_PATH = WORKSPACE / "calibration" / "park_pose_single_arm_20260731.json"
DEFAULT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_LEROBOT_CALIBRATION = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
DEFAULT_MOTION_LOCK = Path("/etc/epuzzle/robot-motion.lock.json")

HELP_TEXT = """\
电磁铁命令（输入后回车）：
  on  / 1   通电吸合（保持至 off/0、退出或急停）
  off / 0   断电释放
  st  / s   查询当前状态
  h         显示本帮助
  q         退出：先断电磁铁，再回正式 PARK
"""


def motion_config() -> MotionConfig:
    """与精确标定一致的保守档；本工具不做笛卡尔运动，仅用于构造适配器。"""
    return MotionConfig(
        max_cartesian_step_mm=5.0,
        max_orientation_step_deg=5.0,
        max_joint_step_deg=5.0,
        joint_tolerance_deg=1.0,
        stable_samples=1,
        poll_interval_s=0.02,
        waypoint_timeout_s=3.0,
        safe_min_z_mm=-20.0,
        contact_min_z_mm=-30.0,
        max_z_mm=180.0,
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="抬臂到 HOME 并手动控制电磁铁（快速标定用，无 GUI）"
    )
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT, help="机械臂串口")
    parser.add_argument(
        "--lerobot-calibration",
        type=Path,
        default=DEFAULT_LEROBOT_CALIBRATION,
        help="LeRobot 五轴标定目录",
    )
    parser.add_argument(
        "--motion-lock",
        type=Path,
        default=DEFAULT_MOTION_LOCK,
        help="运动锁文件路径",
    )
    parser.add_argument(
        "--magnet-only",
        action="store_true",
        help="不连接机械臂，只做电磁铁通断",
    )
    return parser.parse_args()


def check_motion_lock(path: Path) -> None:
    """存在有效锁文件时拒绝运动，锁内容原样打印给现场操作者。"""
    if not path.exists():
        return
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"运动锁 {path} 无法读取，按锁定处理: {error}") from error
    if payload.get("locked", True):
        reason = payload.get("reason", "UNKNOWN")
        requires = payload.get("clear_requires", [])
        lines = "\n".join(f"    - {item}" for item in requires)
        raise RuntimeError(
            f"运动被锁定：{reason}\n  解除前必须完成：\n{lines}\n"
            f"  锁文件：{path}\n  仅电磁铁测试可用 --magnet-only。"
        )


def create_magnet() -> MagnetController:
    magnet = MagnetController(
        MagnetConfig(
            chip_path="/dev/gpiochip0",
            line_offset=17,
            active_high=False,
        )
    )
    magnet.connect()
    print("电磁铁：GPIO17 已请求，当前物理高电平（继电器释放）")
    return magnet


def lift_to_home(hardware: InspectionHardwareAdapter) -> None:
    """关节空间经直立位从正式 PARK 抬到 HOME。"""
    if hardware.park_mover is None:
        raise RuntimeError("机械臂未连接，无法抬臂")
    if not hardware.is_at_park():
        raise RuntimeError(
            "机械臂当前不在正式 PARK 容差内。本工具不猜测起点姿态；"
            "请由现场操作者托住机械臂并先恢复到正式 PARK。"
        )
    print("抬臂中：PARK → 直立位 → HOME ...")
    result = hardware.park_mover.park_to_home()
    if not result.success:
        raise RuntimeError(f"抬臂失败（{result.error_code}）：{result.message}")
    pose = hardware.home_pose()
    print(
        "已到 HOME。TCP(robot_base_mm)="
        f"({pose.position_mm[0]:.1f}, {pose.position_mm[1]:.1f}, "
        f"{pose.position_mm[2]:.1f}) mm, pitch={pose.pitch_deg:.2f}°, "
        f"yaw={pose.yaw_deg:.2f}°"
    )


def print_status(magnet: MagnetController) -> None:
    status = magnet.get_status()
    state = "ON 吸合" if status.enabled else "OFF 释放"
    print(
        f"电磁铁 {state} | reason={status.reason} | GPIO{status.line_offset} | "
        f"t={status.monotonic_ns / 1e9:.3f}s(monotonic)"
    )


def control_loop(magnet: MagnetController) -> None:
    """行缓冲命令循环；EOF 与 q 都走正常退出路径。"""
    print(HELP_TEXT)
    while True:
        try:
            line = input("magnet> ").strip().lower()
        except EOFError:
            print()
            return
        if line in ("q", "quit", "exit"):
            return
        if line in ("on", "1"):
            status = magnet.set_enabled(True)
            print(f"ON  已通电，reason={status.reason}，保持中；请用 off/0 断电")
        elif line in ("off", "0"):
            status = magnet.set_enabled(False)
            print(f"OFF 已断电，reason={status.reason}")
        elif line in ("s", "st", "status"):
            print_status(magnet)
        elif line in ("h", "help", "?"):
            print(HELP_TEXT)
        elif line:
            print(f"未知命令 {line!r}，输入 h 查看帮助")


def run_magnet_only() -> int:
    magnet = create_magnet()
    try:
        control_loop(magnet)
        return 0
    except KeyboardInterrupt:
        print("\n中断：立即断电磁铁", file=sys.stderr)
        magnet.emergency_off()
        return 130
    finally:
        magnet.close()
        print("电磁铁已断电并释放 GPIO")


def run_with_arm(args: argparse.Namespace) -> int:
    for label, path in (
        ("运动学标定", CALIBRATION_PATH),
        ("桌面变换", TRANSFORM_PATH),
        ("PARK 标定", PARK_PATH),
        ("LeRobot 标定目录", args.lerobot_calibration),
        ("机械臂串口", args.port),
    ):
        if not path.exists():
            print(f"缺少{label}：{path}", file=sys.stderr)
            return 1

    hardware = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=motion_config(),
    )
    magnet = create_magnet()
    interrupted = False
    try:
        print("bumpless 连接机械臂 ...")
        hardware.connect()
        print("机械臂已连接，力矩使能，目标已对齐当前位置")
        lift_to_home(hardware)
        control_loop(magnet)
    except KeyboardInterrupt:
        interrupted = True
        print("\n中断：立即断电磁铁", file=sys.stderr)
        try:
            magnet.emergency_off()
        except Exception as error:
            print(f"电磁铁急停失败：{error}", file=sys.stderr)
    finally:
        try:
            magnet.close()
            print("电磁铁已断电并释放 GPIO")
        except Exception as error:
            print(f"电磁铁关闭失败：{error}", file=sys.stderr)

    exit_code = 130 if interrupted else 0
    try:
        if hardware.robot is not None:
            print("回正式 PARK ...")
            success, message = hardware.attempt_park()
            print(("PARK 成功：" if success else "PARK 未确认：") + message)
            if not success:
                print(
                    "请现场托住机械臂后再断电，并按 AGENTS.md 处理运动锁。",
                    file=sys.stderr,
                )
                exit_code = 1
    finally:
        hardware.disconnect()
        print("机械臂已断开（断力矩）")
    return exit_code


def main() -> int:
    args = parse_arguments()
    if args.magnet_only:
        return run_magnet_only()
    try:
        check_motion_lock(args.motion_lock)
    except RuntimeError as error:
        print(error, file=sys.stderr)
        return 1
    try:
        return run_with_arm(args)
    except RuntimeError as error:
        print(f"失败：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
