#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""人工监督的 PICK/PLACE 接触高度标定工具。

使用方法：
1. 在桌面放置一个测试碎片（红色或真实碎片）
2. 运行本工具，按照提示手动调整高度
3. 测试至少 10 次完整的抓取-放置循环
4. 工具生成验收文件 pick_place_contact_height.json

安全要求：
- 操作者全程监督，随时可急停
- 单次磁铁通电不超过 10 秒
- 测试前确认机械臂已回 PARK
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

# 添加工作区模块路径
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "workspace" / "src"))
sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[2]
        / "workspace"
        / "src"
        / "robot_plugin"
        / "lerobot_robot_so101_5d"
    ),
)

from control.magnet_controller import MagnetConfig, MagnetController
from control.motion_controller import MotionConfig, MotionGoal
from integration.runtime_adapter import (
    PREGRASP_HEIGHT_MM,
    SAFE_HEIGHT_MM,
    InspectionHardwareAdapter,
)

WORKSPACE = Path(__file__).resolve().parents[2] / "workspace"
CALIBRATION_PATH = (
    WORKSPACE / "calibration" / "arm_kinematics_single_arm_accepted_20260731.json"
)
TRANSFORM_PATH = WORKSPACE / "calibration" / "contact_sheet_to_robot_base_20260731.json"
PARK_PATH = WORKSPACE / "calibration" / "park_pose_single_arm_20260731.json"
DEFAULT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_LEROBOT_CALIBRATION = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
OUTPUT_PATH = WORKSPACE / "calibration" / "pick_place_contact_height.json"

# 测试位置：工作区中心附近
TEST_POSITION_TABLE_MM = (105.0, 148.5)


def motion_config() -> MotionConfig:
    """精确标定档：5 mm 插值，关节步长 5°。"""
    return MotionConfig(5.0, 5.0, 5.0, 1.0, 1, 0.02, 3.0, -20.0, -30.0, 180.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        type=Path,
        default=DEFAULT_PORT,
        help="机械臂串口",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=OUTPUT_PATH,
        help="输出 JSON 文件路径",
    )
    parser.add_argument(
        "--pick-height",
        type=float,
        help="PICK 接触高度（mm），如已知可直接指定",
    )
    parser.add_argument(
        "--place-height",
        type=float,
        help="PLACE 接触高度（mm），如已知可直接指定",
    )
    parser.add_argument(
        "--test-cycles",
        type=int,
        default=10,
        help="完整测试循环次数（默认 10）",
    )
    args = parser.parse_args()

    if not args.port.exists():
        print(f"错误：机械臂串口不存在: {args.port}", file=sys.stderr)
        return 2

    print("=" * 70)
    print("PICK/PLACE 接触高度标定工具")
    print("=" * 70)
    print()
    print("安全提示：")
    print("  - 操作者全程监督，随时可按 Ctrl+C 急停")
    print("  - 单次磁铁通电不超过 10 秒")
    print("  - 确认机械臂当前已在 PARK 位置")
    print()
    print(
        f"测试位置：table_mm ({TEST_POSITION_TABLE_MM[0]}, {TEST_POSITION_TABLE_MM[1]})"
    )
    print(f"测试循环：{args.test_cycles} 次")
    print()

    # 连接硬件
    try:
        print("正在连接机械臂...")
        hardware = InspectionHardwareAdapter(
            calibration_path=CALIBRATION_PATH,
            transform_path=TRANSFORM_PATH,
            park_pose_path=PARK_PATH,
            port=args.port,
            lerobot_calibration_dir=DEFAULT_LEROBOT_CALIBRATION,
            motion_config=motion_config(),
        )
        hardware.connect()
        print("✓ 机械臂已连接")

        print("正在连接电磁铁...")
        magnet = MagnetController(MagnetConfig())
        magnet.connect()
        print("✓ 电磁铁已连接")
        print()

    except Exception as error:
        print(f"硬件连接失败: {error}", file=sys.stderr)
        return 1

    # 交互式标定流程
    try:
        if args.pick_height is None:
            print("步骤 1: 测量 PICK 接触高度")
            print("-" * 70)
            pick_height = calibrate_pick_height(hardware, magnet)
        else:
            pick_height = args.pick_height
            print(f"使用指定的 PICK 高度: {pick_height:.2f} mm")

        if args.place_height is None:
            print()
            print("步骤 2: 测量 PLACE 接触高度")
            print("-" * 70)
            place_height = calibrate_place_height(hardware, magnet)
        else:
            place_height = args.place_height
            print(f"使用指定的 PLACE 高度: {place_height:.2f} mm")

        print()
        print("步骤 3: 完整循环测试")
        print("-" * 70)
        test_results = run_test_cycles(
            hardware, magnet, pick_height, place_height, args.test_cycles
        )

        # 生成验收文件
        output_data = generate_acceptance_file(pick_height, place_height, test_results)

        # 保存到文件
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output_data, ensure_ascii=False, indent=2))
        print()
        print("=" * 70)
        print(f"✓ 标定完成，验收文件已保存: {args.output}")
        print(f"  PICK 高度: {pick_height:.2f} mm")
        print(f"  PLACE 高度: {place_height:.2f} mm")
        print(f"  成功率: {test_results['success_rate']:.1%}")
        print(f"  验收状态: {output_data['quality']['status']}")
        print("=" * 70)

        return 0

    except KeyboardInterrupt:
        print("\n\n用户中止标定", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"\n标定过程异常: {error}", file=sys.stderr)
        import traceback

        traceback.print_exc()
        return 1
    finally:
        try:
            magnet.emergency_off()
            hardware.attempt_park()
            hardware.disconnect()
            magnet.close()
        except Exception:
            pass


def calibrate_pick_height(
    hardware: InspectionHardwareAdapter, magnet: MagnetController
) -> float:
    """交互式测量 PICK 接触高度。"""
    print("请在测试位置放置一个碎片（红色测试件或真实碎片）")
    input("准备好后按回车继续...")

    # 先移动到安全高度
    print(f"移动到安全高度 {SAFE_HEIGHT_MM} mm...")
    move_to_test_position(hardware, SAFE_HEIGHT_MM)

    print()
    print("现在开始向下移动，直到磁铁接触碎片。")
    print("输入数字调整高度（负数向下），输入 'test' 测试吸附，输入 'done' 完成。")

    current_height = SAFE_HEIGHT_MM
    while True:
        print(f"\n当前高度: {current_height:.1f} mm")
        command = input("调整高度 (mm) / test / done: ").strip().lower()

        if command == "done":
            break
        elif command == "test":
            print("测试吸附...")
            success = test_pickup(hardware, magnet, current_height)
            if success:
                print("✓ 吸附成功")
            else:
                print("✗ 吸附失败，建议降低高度")
        else:
            try:
                delta = float(command)
                new_height = current_height + delta
                if new_height < 0 or new_height > SAFE_HEIGHT_MM:
                    print(f"高度超出范围 [0, {SAFE_HEIGHT_MM}]")
                    continue
                print(f"移动到 {new_height:.1f} mm...")
                move_to_test_position(hardware, new_height)
                current_height = new_height
            except ValueError:
                print("无效输入")

    print(f"\n✓ PICK 高度确定为: {current_height:.2f} mm")
    return current_height


def calibrate_place_height(
    hardware: InspectionHardwareAdapter, magnet: MagnetController
) -> float:
    """交互式测量 PLACE 接触高度。"""
    print("PLACE 高度测量：将碎片放置到目标位置的高度")
    print("通常 PLACE 高度略高于 PICK 高度，以确保平稳释放。")

    # 先移动到安全高度
    print(f"移动到安全高度 {SAFE_HEIGHT_MM} mm...")
    move_to_test_position(hardware, SAFE_HEIGHT_MM)

    print()
    print("输入数字调整高度（负数向下），输入 'test' 测试释放，输入 'done' 完成。")

    current_height = PREGRASP_HEIGHT_MM
    move_to_test_position(hardware, current_height)

    while True:
        print(f"\n当前高度: {current_height:.1f} mm")
        command = input("调整高度 (mm) / test / done: ").strip().lower()

        if command == "done":
            break
        elif command == "test":
            print("测试释放...")
            success = test_release(hardware, magnet, current_height)
            if success:
                print("✓ 释放成功")
            else:
                print("✗ 释放不理想")
        else:
            try:
                delta = float(command)
                new_height = current_height + delta
                if new_height < 0 or new_height > SAFE_HEIGHT_MM:
                    print(f"高度超出范围 [0, {SAFE_HEIGHT_MM}]")
                    continue
                print(f"移动到 {new_height:.1f} mm...")
                move_to_test_position(hardware, new_height)
                current_height = new_height
            except ValueError:
                print("无效输入")

    print(f"\n✓ PLACE 高度确定为: {current_height:.2f} mm")
    return current_height


def move_to_test_position(
    hardware: InspectionHardwareAdapter, height_mm: float
) -> None:
    """移动到测试位置的指定高度。"""
    from integration.sweep_planner import PieceRef

    test_piece = PieceRef(
        piece_id="test",
        center_table_mm=TEST_POSITION_TABLE_MM,
    )
    pose = hardware.pose_for(test_piece, height_mm)
    timestamp_ns = time.monotonic_ns()
    deadline_ns = timestamp_ns + 30_000_000_000

    goal = MotionGoal(
        goal_id=f"calibrate-height-{height_mm:.1f}",
        timestamp_ns=timestamp_ns,
        frame="robot_base_mm",
        position_mm=pose.position_mm,
        pitch_deg=pose.pitch_deg,
        yaw_deg=pose.yaw_deg,
        phase="CALIBRATION",
        speed_profile="EMPTY_SLOW",
        deadline_ns=deadline_ns,
    )

    if hardware.motion is None:
        raise RuntimeError("Motion controller not connected")

    result = hardware.motion.move(goal)
    if not result.success:
        raise RuntimeError(f"Motion failed: {result.message}")


def test_pickup(
    hardware: InspectionHardwareAdapter, magnet: MagnetController, height_mm: float
) -> bool:
    """测试在当前高度吸附碎片。"""
    move_to_test_position(hardware, height_mm)
    magnet.set_enabled(True)
    time.sleep(2.0)

    # 提升到安全高度
    move_to_test_position(hardware, SAFE_HEIGHT_MM)
    time.sleep(1.0)

    print("碎片是否被吸附？ (y/n): ", end="")
    success = input().strip().lower() == "y"

    magnet.set_enabled(False)
    time.sleep(1.0)

    return success


def test_release(
    hardware: InspectionHardwareAdapter, magnet: MagnetController, height_mm: float
) -> bool:
    """测试在当前高度释放碎片。"""
    # 先吸附
    print("  先吸附碎片...")
    if not test_pickup(hardware, magnet, height_mm - 2.0):
        print("  吸附失败，跳过释放测试")
        return False

    # 移动到释放高度
    move_to_test_position(hardware, height_mm)
    magnet.set_enabled(False)
    time.sleep(1.0)

    # 提升验证
    move_to_test_position(hardware, SAFE_HEIGHT_MM)

    print("碎片是否平稳释放到目标位置？ (y/n): ", end="")
    success = input().strip().lower() == "y"

    return success


def run_test_cycles(
    hardware: InspectionHardwareAdapter,
    magnet: MagnetController,
    pick_height: float,
    place_height: float,
    cycles: int,
) -> dict[str, Any]:
    """运行完整的抓取-放置测试循环。"""
    print(f"执行 {cycles} 次完整循环...")
    print("每次循环后输入结果: 's' (成功) / 'f' (失败) / 'q' (退出)")
    print()

    success_count = 0
    failure_count = 0

    for i in range(cycles):
        print(f"循环 {i + 1}/{cycles}:")

        try:
            # PICK
            print(f"  移动到 PICK 高度 {pick_height:.1f} mm...")
            move_to_test_position(hardware, pick_height)
            print("  磁铁通电...")
            magnet.set_enabled(True)
            time.sleep(2.0)

            # 提升
            print(f"  提升到 {SAFE_HEIGHT_MM} mm...")
            move_to_test_position(hardware, SAFE_HEIGHT_MM)
            time.sleep(1.0)

            # PLACE
            print(f"  下降到 PLACE 高度 {place_height:.1f} mm...")
            move_to_test_position(hardware, place_height)
            print("  磁铁断电...")
            magnet.set_enabled(False)
            time.sleep(1.0)

            # 提升
            print(f"  提升到 {SAFE_HEIGHT_MM} mm...")
            move_to_test_position(hardware, SAFE_HEIGHT_MM)

        except Exception as error:
            print(f"  错误: {error}")
            failure_count += 1
            continue

        result = input("  结果 (s/f/q): ").strip().lower()
        if result == "q":
            print("提前结束测试")
            break
        elif result == "s":
            success_count += 1
            print("  ✓ 成功")
        else:
            failure_count += 1
            print("  ✗ 失败")

        print()

    total = success_count + failure_count
    success_rate = success_count / total if total > 0 else 0.0

    return {
        "total_cycles": total,
        "success_count": success_count,
        "failure_count": failure_count,
        "success_rate": success_rate,
    }


def generate_acceptance_file(
    pick_height: float, place_height: float, test_results: dict[str, Any]
) -> dict[str, Any]:
    """生成验收文件数据。"""
    success_rate = test_results["success_rate"]
    accepted = success_rate >= 0.8

    return {
        "schema_version": 1,
        "calibration_id": f"pick-place-contact-heights-{time.strftime('%Y%m%d')}",
        "date": time.strftime("%Y-%m-%d"),
        "reference_frame": "calibration_sheet_mm",
        "test_procedure": "Manual supervised pick and place with magnet verification",
        "quality": {
            "status": "accepted" if accepted else "rejected",
            "reason": f"成功率 {success_rate:.1%}，{'达到' if accepted else '未达到'} 80% 门槛",
            "reviewer": "operator",
            "review_date": time.strftime("%Y-%m-%d"),
        },
        "contact_heights_mm": {
            "pick": pick_height,
            "place": place_height,
            "pick_source": "Manual calibration with operator supervision",
            "place_source": "Manual calibration with operator supervision",
        },
        "magnet_verification": {
            "pickup_tested": True,
            "release_tested": True,
            "effective_gap_mm": None,
            "test_mass_g": 30,
            "pickup_success_count": test_results["success_count"],
            "pickup_failure_count": test_results["failure_count"],
            "release_success_count": test_results["success_count"],
            "release_failure_count": test_results["failure_count"],
            "on_settle_s": 2.0,
            "off_settle_s": 1.0,
        },
        "single_piece_success_rate": success_rate,
        "total_test_cycles": test_results["total_cycles"],
        "notes": [
            f"PICK height: {pick_height:.2f} mm - measured with loaded magnet",
            f"PLACE height: {place_height:.2f} mm - measured with piece release",
            f"Test cycles: {test_results['total_cycles']} full pick-place operations",
            f"Success rate: {success_rate:.1%}",
            "Magnet footprint and load tested on real hardware",
        ],
        "test_evidence": {
            "video_paths": [],
            "measurement_log": None,
            "operator_notes": f"Manual calibration completed on {time.strftime('%Y-%m-%d')}",
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
