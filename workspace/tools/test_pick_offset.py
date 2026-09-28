#!/usr/bin/env python3
"""单片 PICK 偏移补偿验证工具。

默认只计算补偿点；必须显式传 --execute 才会连接机械臂并执行一次：
PARK -> HOME -> SAFE -> 垂直 approach -> PICK -> 吸合 -> 垂直抬升 -> PARK。

偏移定义：沿 table_mm 中远离机械臂（sheet_origin_table_mm）的方向移动，
只改变本次 PICK 点，不改变 PLACE 或正式拼装配置。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "src" / "robot_plugin" / "lerobot_robot_so101_5d"),
)

from control.magnet_controller import MagnetConfig, MagnetController
from control.motion_controller import MotionConfig, MotionGoal
from integration.runtime_adapter import InspectionHardwareAdapter
from integration.sweep_planner import PieceRef

WORKSPACE = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = WORKSPACE / "calibration" / "arm_kinematics_single_arm_accepted_20260731.json"
TRANSFORM_PATH = WORKSPACE / "calibration" / "contact_sheet_to_robot_base_20260731.json"
PARK_PATH = WORKSPACE / "calibration" / "park_pose_single_arm_20260731.json"
DEFAULT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_CALIBRATION_DIR = Path(
    "/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
LOCK_PATH = Path("/etc/epuzzle/robot-motion.lock.json")


def parse_xy(value: str) -> tuple[float, float]:
    try:
        x_text, y_text = value.split(",", 1)
        x, y = float(x_text), float(y_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("坐标必须为 X,Y") from exc
    if not all(math.isfinite(item) for item in (x, y)):
        raise argparse.ArgumentTypeError("坐标必须是有限数")
    return x, y


def motion_config() -> MotionConfig:
    return MotionConfig(
        max_cartesian_step_mm=10.0,
        max_orientation_step_deg=8.0,
        max_joint_step_deg=8.0,
        joint_tolerance_deg=5.0,
        stable_samples=1,
        poll_interval_s=0.05,
        waypoint_timeout_s=30.0,
        safe_min_z_mm=-20.0,
        contact_min_z_mm=-30.0,
        max_z_mm=180.0,
        follow_tolerance_deg=8.0,
        goal_velocity_by_profile={
            "EMPTY_NORMAL": 150,
            "EMPTY_SLOW": 100,
            "HOLDING_SLOW": 100,
        },
        waypoint_min_dwell_s_by_profile={
            "EMPTY_NORMAL": 0.15,
            "EMPTY_SLOW": 0.20,
            "HOLDING_SLOW": 0.30,
        },
    )


def corrected_point(
    point_table_mm: tuple[float, float],
    arm_origin_table_mm: tuple[float, float],
    away_mm: float,
) -> tuple[float, float]:
    if not math.isfinite(away_mm) or away_mm < 0.0:
        raise ValueError("away_mm 必须是非负有限数")
    dx = point_table_mm[0] - arm_origin_table_mm[0]
    dy = point_table_mm[1] - arm_origin_table_mm[1]
    distance = math.hypot(dx, dy)
    if distance <= 1e-6:
        raise ValueError("测试点不能与机械臂 table 原点重合")
    scale = away_mm / distance
    return point_table_mm[0] + dx * scale, point_table_mm[1] + dy * scale


def move(hardware: InspectionHardwareAdapter, pose, phase: str, speed: str) -> None:
    if hardware.motion is None:
        raise RuntimeError("机械臂运动控制器未连接")
    result = hardware.motion.move(
        MotionGoal(
            goal_id=f"offset-test-{phase.lower()}",
            timestamp_ns=time.monotonic_ns(),
            frame=pose.frame,
            position_mm=pose.position_mm,
            pitch_deg=pose.pitch_deg,
            yaw_deg=pose.yaw_deg,
            phase=phase,
            speed_profile=speed,
            deadline_ns=time.monotonic_ns() + 180_000_000_000,
        )
    )
    if not result.success:
        raise RuntimeError(f"{phase} 失败: {result.message}")


def main() -> int:
    parser = argparse.ArgumentParser(description="单片 PICK 偏移补偿测试")
    parser.add_argument("--table-mm", required=True, type=parse_xy, help="原始目标点 X,Y")
    parser.add_argument("--away-mm", type=float, default=10.0, help="远离机械臂的补偿量，默认 10mm")
    # 现场反馈接触高度可能偏低；默认在已验收 6mm 上抬高 5mm，
    # 仅影响本独立测试，不修改正式抓放配置。
    parser.add_argument("--pick-height-mm", type=float, default=11.0)
    parser.add_argument("--approach-mm", type=float, default=30.0)
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument("--lerobot-calibration", type=Path, default=DEFAULT_CALIBRATION_DIR)
    parser.add_argument("--execute", action="store_true", help="显式执行真实单片测试")
    args = parser.parse_args()

    if args.pick_height_mm < -30.0 or args.pick_height_mm > 180.0:
        parser.error("pick-height-mm 超出安全范围 [-30,180]")
    if args.approach_mm <= 0.0 or args.approach_mm > 60.0:
        parser.error("approach-mm 必须在 (0,60] 内")
    if LOCK_PATH.exists():
        raise SystemExit(f"运动锁存在，拒绝执行: {LOCK_PATH}")

    hardware = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=motion_config(),
    )
    original = args.table_mm
    corrected = corrected_point(original, hardware.transform.sheet_origin_table_mm, args.away_mm)
    print(f"原始 table_mm: ({original[0]:.3f}, {original[1]:.3f})")
    print(f"修正 table_mm: ({corrected[0]:.3f}, {corrected[1]:.3f})")
    print(f"偏移: 远离机械臂 {args.away_mm:.3f} mm；仅影响本次 PICK")
    if not args.execute:
        print("仅计算模式：增加 --execute 才会运动")
        return 0

    magnet = MagnetController(MagnetConfig(chip_path="/dev/gpiochip0", line_offset=17, active_high=False))
    connected = False
    magnet_connected = False
    completed = False
    try:
        hardware.connect()
        connected = True
        if hardware.park_mover is None or not hardware.is_at_park():
            raise RuntimeError("机械臂不在正式 PARK，拒绝偏移测试")
        magnet.connect()
        magnet_connected = True
        magnet.set_enabled(False)
        hardware.park_mover.park_to_home()

        piece = PieceRef("offset-test", corrected)
        pick = hardware.pose_for(piece, args.pick_height_mm)
        approach = type(pick)(
            pick.frame,
            (pick.position_mm[0], pick.position_mm[1], pick.position_mm[2] + args.approach_mm),
            pick.pitch_deg,
            pick.yaw_deg,
        )
        safe = type(pick)(
            pick.frame,
            (pick.position_mm[0], pick.position_mm[1], 60.0),
            pick.pitch_deg,
            pick.yaw_deg,
        )
        print("执行 SAFE -> 垂直 approach -> PICK")
        move(hardware, safe, "SAFE", "EMPTY_NORMAL")
        move(hardware, approach, "PREGRASP", "EMPTY_SLOW")
        move(hardware, pick, "PICK", "EMPTY_SLOW")
        magnet.set_enabled(True)
        time.sleep(2.0)
        # MotionGoal 没有 VERIFY_PICK phase；状态语义在这里用 SAFE，
        # 仍保持同一点的垂直抬升和完整编码器读回。
        move(hardware, approach, "SAFE", "HOLDING_SLOW")
        print("已垂直抬升 30mm；请现场确认是否吸附")
        magnet.set_enabled(False)
        move(hardware, safe, "SAFE", "EMPTY_SLOW")
        result = hardware.park_mover.home_to_park()
        if not result.success:
            raise RuntimeError(f"HOME->PARK 失败: {result.message}")
        completed = True
        return 0
    finally:
        if magnet_connected:
            magnet.emergency_off()
            magnet.close()
        if connected:
            hardware.disconnect()
        if not completed:
            print("测试未完成：未自动猜测或强行回 PARK，请先只读确认机械臂状态")


if __name__ == "__main__":
    raise SystemExit(main())
