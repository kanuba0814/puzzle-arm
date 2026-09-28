#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""USB 掉线自动恢复实机验证工具（人工监督，拔插由现场操作者执行）。

对应 docs/26_USB断连故障研究.md 第 7 节待办：验证轻量重连（reconnect_lite）
在真实 USB 拔插下按预期工作。

mode=recovery（默认）：
    每轮在运动间隙提示操作者拔下机械臂 USB 线 2–3 秒后插回，随后执行一次
    安全高度 PREGRASP 往返。若本轮内核日志确认发生 USB 断开，则断言
    reconnect_epoch 增加且运动成功（自动恢复）；未发生断开则本轮标记 skipped。

mode=contact-fail-closed：
    PICK 相位运动前拔线（保持拔出）→ 断言禁止自动重连（ROBOT_COMM_TIMEOUT、
    reconnect_epoch==0，且不发送任何运动命令）；插回后断言清理路径
    attempt_park 通过轻量重连完成正式 PARK。

安全约束：
- 全程人工监督；Ctrl+C 随时急停（finally 保证回 PARK 并断力矩）。
- 不启磁铁；PICK 模式目标高度取 50 mm（远离桌面，且运动前线已拔，
  第一步读取即失败，不会真的下探）。
- 结束总是回正式 PARK 并断力矩，输出内核事件统计。

用法（在 /opt/epuzzle 下，epuzzle 用户）：
    bash tools/run_verify_usb_reconnect.sh --mode recovery --rounds 3
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
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

from control.calibration import ArmCalibration  # noqa: E402
from control.motion_controller import (  # noqa: E402
    MotionConfig,
    MotionController,
    MotionErrorCode,
    MotionGoal,
)
from integration.runtime_adapter import InspectionHardwareAdapter  # noqa: E402

WORKSPACE = Path(__file__).resolve().parents[1]
CALIBRATION_PATH = (
    WORKSPACE / "calibration" / "arm_kinematics_single_arm_accepted_20260731.json"
)
TRANSFORM_PATH = WORKSPACE / "calibration" / "contact_sheet_to_robot_base_20260731.json"
PARK_PATH = WORKSPACE / "calibration" / "park_pose_single_arm_20260731.json"
ROBOT_MOTION_LOCK_PATH = Path("/etc/epuzzle/robot-motion.lock.json")
DEFAULT_PORT = Path("/dev/epuzzle-arm")
DEFAULT_LEROBOT_CALIBRATION = Path(
    "/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)
PREGRASP_HEIGHT_MM = 60.0
CONTACT_VERIFY_HEIGHT_MM = 50.0


def motion_config() -> MotionConfig:
    """与精确标定一致的保守档；与 pick_place_motion_config 同参数。"""
    return MotionConfig(
        max_cartesian_step_mm=15.0,
        max_orientation_step_deg=10.0,
        max_joint_step_deg=10.0,
        joint_tolerance_deg=3.0,
        stable_samples=1,
        poll_interval_s=0.02,
        waypoint_timeout_s=3.0,
        safe_min_z_mm=-20.0,
        contact_min_z_mm=-30.0,
        max_z_mm=180.0,
    )


def motion_lock_reason() -> str | None:
    if not ROBOT_MOTION_LOCK_PATH.exists():
        return None
    payload = json.loads(ROBOT_MOTION_LOCK_PATH.read_text(encoding="utf-8"))
    return payload.get("reason") if isinstance(payload, dict) else str(payload)


def kernel_usb_events() -> tuple[int, int]:
    """本次启动内核日志中的 (usb 3-x disconnect 数, EMI 端口禁用数)。"""
    try:
        logs = subprocess.run(
            ["journalctl", "-k", "-b", "0", "--no-pager"],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        logs = ""
    disconnects = len(re.findall(r"usb 3-[12]: USB disconnect", logs))
    emi = len(re.findall(r"disabled by hub \(EMI\?\)", logs))
    return disconnects, emi


def make_goal(
    adapter: InspectionHardwareAdapter,
    goal_id: str,
    position_mm: tuple[float, float, float],
    *,
    phase: str,
) -> MotionGoal:
    home = adapter.home_pose()
    return MotionGoal(
        goal_id=goal_id,
        timestamp_ns=time.monotonic_ns(),
        frame="robot_base_mm",
        position_mm=position_mm,
        pitch_deg=home.pitch_deg,
        yaw_deg=home.yaw_deg,
        phase=phase,
        speed_profile="EMPTY_SLOW",
    )


def run_recovery(
    adapter: InspectionHardwareAdapter, rounds: int
) -> list[dict[str, object]]:
    motion = adapter.motion
    assert motion is not None
    home = adapter.home_pose()
    base_x, base_y, _ = home.position_mm
    goal_a = make_goal(
        adapter,
        "usb-verify-a",
        (base_x + 30.0, base_y, PREGRASP_HEIGHT_MM),
        phase="PREGRASP",
    )
    goal_b = make_goal(
        adapter,
        "usb-verify-b",
        (base_x, base_y, PREGRASP_HEIGHT_MM),
        phase="PREGRASP",
    )
    results: list[dict[str, object]] = []
    for index in range(1, rounds + 1):
        disconnect_before, _ = kernel_usb_events()
        input(f"\n[第 {index}/{rounds} 轮] 请拔出机械臂 USB 线 2–3 秒后插回，按回车开始运动：")
        epoch_before = adapter.robot.reconnect_epoch()  # type: ignore[union-attr]
        result_a = motion.move(goal_a)
        result_b = motion.move(goal_b)
        epoch_after = adapter.robot.reconnect_epoch()  # type: ignore[union-attr]
        disconnect_after, emi_after = kernel_usb_events()
        round_result = {
            "round": index,
            "dropout_detected": disconnect_after > disconnect_before,
            "emi_events_since_boot": emi_after,
            "reconnect_epoch_delta": epoch_after - epoch_before,
            "move_a": {
                "success": result_a.success,
                "error_code": result_a.error_code,
                "executed": result_a.executed_waypoints,
                "planned": result_a.planned_waypoints,
            },
            "move_b": {
                "success": result_b.success,
                "error_code": result_b.error_code,
                "executed": result_b.executed_waypoints,
                "planned": result_b.planned_waypoints,
            },
        }
        if not round_result["dropout_detected"]:
            round_result["status"] = "skipped(no kernel dropout event)"
            print("  ⚠ 内核未记录 USB 断开事件，本轮视为无效（请确认拔的是机械臂 USB 线）")
        elif (
            round_result["reconnect_epoch_delta"] >= 1
            and result_a.success
            and result_b.success
        ):
            round_result["status"] = "PASS"
            print("  ✓ 掉线已检测，轻量重连成功，PREGRASP 往返完成")
        else:
            round_result["status"] = "FAIL"
            print("  ✗ 掉线后未自动恢复，详见结果")
        results.append(round_result)
    return results


def run_contact_fail_closed(
    adapter: InspectionHardwareAdapter,
) -> dict[str, object]:
    motion = adapter.motion
    assert motion is not None
    home = adapter.home_pose()
    base_x, base_y, _ = home.position_mm
    pick_goal = make_goal(
        adapter,
        "usb-verify-pick",
        (base_x, base_y, CONTACT_VERIFY_HEIGHT_MM),
        phase="PICK",
    )
    input("\n请拔出机械臂 USB 线（保持拔出），按回车执行 PICK 相位验证：")
    epoch_before = adapter.robot.reconnect_epoch()  # type: ignore[union-attr]
    result = motion.move(pick_goal)
    epoch_after = adapter.robot.reconnect_epoch()  # type: ignore[union-attr]
    reconnect_denied = (
        result.error_code == MotionErrorCode.ROBOT_COMM_TIMEOUT
        and epoch_after == epoch_before
        and result.executed_waypoints == 0
    )
    print(
        "  PICK 相位结果: "
        f"{result.error_code} executed={result.executed_waypoints} "
        f"reconnect_epoch={epoch_before}->{epoch_after}"
    )
    if not reconnect_denied:
        return {
            "status": "FAIL",
            "reason": "PICK 相位未按预期 fail-closed（应 ROBOT_COMM_TIMEOUT 且不重连不运动）",
            "error_code": result.error_code,
            "reconnect_epoch_delta": epoch_after - epoch_before,
        }
    print("  ✓ PICK 接触段禁止自动重连：ROBOT_COMM_TIMEOUT，未发送运动命令")

    input("\n请插回机械臂 USB 线，按回车执行清理路径 PARK 验证：")
    parked, message = adapter.attempt_park()
    epoch_parked = adapter.robot.reconnect_epoch()  # type: ignore[union-attr]
    cleanup_recovered = parked and epoch_parked > epoch_after
    print(f"  attempt_park: parked={parked} reconnect_epoch={epoch_after}->{epoch_parked}")
    print(f"  {message}")
    if not cleanup_recovered:
        return {
            "status": "FAIL",
            "reason": "清理路径未通过轻量重连完成 PARK",
            "parked": parked,
            "message": message,
        }
    print("  ✓ 清理路径经轻量重连完成正式 PARK")
    return {
        "status": "PASS",
        "error_code": result.error_code,
        "parked": parked,
        "message": message,
        "reconnect_epoch_delta": epoch_parked - epoch_before,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="USB 掉线自动恢复实机验证")
    parser.add_argument("--mode", choices=("recovery", "contact-fail-closed"), default="recovery")
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--port", type=Path, default=DEFAULT_PORT)
    parser.add_argument(
        "--lerobot-calibration",
        type=Path,
        default=DEFAULT_LEROBOT_CALIBRATION,
    )
    args = parser.parse_args()

    lock_reason = motion_lock_reason()
    if lock_reason:
        raise SystemExit(f"运动安全锁: {lock_reason}")
    if not args.port.exists():
        raise SystemExit(f"机械臂离线: {args.port}")
    ArmCalibration.load(CALIBRATION_PATH).require_accepted_geometry()

    adapter = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=motion_config(),
    )
    results: list[dict[str, object]] = []
    try:
        adapter.connect()
        adapter.park_mover.park_to_home()  # type: ignore[union-attr]
        if args.mode == "recovery":
            results = run_recovery(adapter, args.rounds)
        else:
            results = [run_contact_fail_closed(adapter)]
    finally:
        try:
            parked, message = adapter.attempt_park()
        except Exception as error:  # noqa: BLE001 - 清理路径任何异常都要报告
            parked, message = False, f"异常: {error}"
        adapter.disconnect()
        disconnect_count, emi_count = kernel_usb_events()
        print(f"\n清理: PARK={parked} {message}")
        print(
            f"本次启动内核 USB 事件: disconnect={disconnect_count} "
            f"emi={emi_count}（含历史累计）"
        )
        print(json.dumps({"mode": args.mode, "results": results}, ensure_ascii=False, indent=2))
        if not parked:
            raise SystemExit("未能在正式 PARK 断力矩，请按现场事故流程处理")


if __name__ == "__main__":
    main()
