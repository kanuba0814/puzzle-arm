#!/usr/bin/env python3
"""四片连续拼装执行器（带 yaw 补偿）。

基于 123 求解结果，逐片执行 PICK → PLACE，传入正确的末端 yaw。
IK 自动计算 roll = yaw_target - pan，补偿机械臂旋转。

用法（Pi 上，/opt/epuzzle 下）：
    sudo PYTHONPATH=/opt/epuzzle/workspace/src/robot_plugin/lerobot_robot_so101_5d:/opt/epuzzle/workspace/src \
        /opt/epuzzle/venv/bin/python workspace/tools/run_assembly_4pieces.py \
        --plan /home/epuzzle/plan_now/result.json [--dry-run]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path

_WORKSPACE = Path(__file__).resolve().parent.parent
_SRC = _WORKSPACE / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from control.kinematics import ToolPose  # noqa: E402
from control.magnet_controller import MagnetController  # noqa: E402
from control.motion_controller import MotionConfig  # noqa: E402
from integration import (  # noqa: E402
    PlacementPlanRef,
    Supervisor,
    SupervisorConfig,
    SupervisorErrorCode,
    SupervisorFailure,
)
from integration.competition_backend import (  # noqa: E402
    pick_place_lock_reason,
    robot_motion_lock_reason,
)
from integration.runtime_adapter import (  # noqa: E402
    InspectionHardwareAdapter,
    PREGRASP_HEIGHT_MM,
    SAFE_HEIGHT_MM,
)
from integration.sweep_planner import PieceRef, preflight_continuous_sweep  # noqa: E402
from integration.supervisor import VisionBatch  # noqa: E402

CALIBRATION_PATH = _WORKSPACE / "calibration/arm_kinematics_single_arm_accepted_20260731.json"
TRANSFORM_PATH = _WORKSPACE / "calibration/contact_sheet_to_robot_base_20260731.json"
PARK_POSE_PATH = _WORKSPACE / "calibration/park_pose_single_arm_20260731.json"
PICK_PLACE_HEIGHT_PATH = _WORKSPACE / "calibration/pick_place_contact_height.json"
ROBOT_MOTION_LOCK_PATH = Path("/etc/epuzzle/robot-motion.lock.json")
LOG_PATH = _WORKSPACE / "logs/supervisor/assembly-4pieces.jsonl"


@dataclass(frozen=True)
class PickPlaceContactHeights:
    pick_mm: float
    place_mm: float
    magnet_on_settle_s: float
    magnet_off_settle_s: float


def load_pick_place_heights() -> PickPlaceContactHeights:
    lock_reason = pick_place_lock_reason(PICK_PLACE_HEIGHT_PATH)
    if lock_reason:
        raise SupervisorFailure(SupervisorErrorCode.SUPERVISOR_LOCKED, lock_reason)
    payload = json.loads(PICK_PLACE_HEIGHT_PATH.read_text(encoding="utf-8"))
    h = payload["contact_heights_mm"]
    mv = payload["magnet_verification"]
    return PickPlaceContactHeights(
        h["pick"], h["place"], mv["on_settle_s"], mv["off_settle_s"]
    )


def pick_place_motion_config() -> MotionConfig:
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


def make_config(
    shadow: bool, heights: PickPlaceContactHeights | None
) -> SupervisorConfig:
    return SupervisorConfig(
        calibration_path=CALIBRATION_PATH,
        park_pose_path=PARK_POSE_PATH,
        transform_path=TRANSFORM_PATH,
        transform_is_official=True,
        motion_config=(
            pick_place_motion_config() if heights is not None else None
        ),
        z_safe_mm=SAFE_HEIGHT_MM,
        z_pregrasp_mm=PREGRASP_HEIGHT_MM,
        z_pick_mm=None if heights is None else heights.pick_mm,
        z_place_mm=None if heights is None else heights.place_mm,
        park_tolerance_deg=4,
        park_follow_tolerance_deg=8,
        max_vision_age_ms=3_600_000,  # 计划预计算后四片连续执行，单次 grasp_fsm ~20s，放宽到 1h
        global_timeout_s=180,
        shadow=shadow,
        automatic_pick_place_enabled=heights is not None,
        precision_radial_compensation_mm=3,
        precision_pitch_command_deg=-87.5,
        event_log_path=LOG_PATH,
        magnet_footprint_clearance_frozen=heights is not None,
        magnet_on_settle_s=None if heights is None else heights.magnet_on_settle_s,
        magnet_off_settle_s=None if heights is None else heights.magnet_off_settle_s,
    )


class _PlanVision:
    """固定返回给定批次的 vision 后端（不重新检测）。"""

    def __init__(self, batch: VisionBatch) -> None:
        self._batch = batch

    def scan(self) -> VisionBatch:
        return self._batch


def main() -> None:
    parser = argparse.ArgumentParser(description="四片连续拼装（带 yaw 补偿）")
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--port", type=Path, default=Path("/dev/epuzzle-arm"))
    parser.add_argument(
        "--lerobot-calibration",
        type=Path,
        default=Path("/home/epuzzle/.cache/huggingface/lerobot/calibration/robots/so101_follower_5d"),
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    heights = load_pick_place_heights()
    lock_reason = robot_motion_lock_reason(ROBOT_MOTION_LOCK_PATH)
    if lock_reason:
        raise SupervisorFailure(SupervisorErrorCode.SUPERVISOR_LOCKED, f"运动安全锁: {lock_reason}")
    if not args.dry_run and not args.port.exists():
        raise SupervisorFailure(SupervisorErrorCode.ROBOT_COMM_TIMEOUT, f"机械臂离线: {args.port}")

    plan_data = json.loads(args.plan.read_text(encoding="utf-8"))
    placements = plan_data.get("placements") or plan_data.get("solve", {}).get("placements", [])
    if not 1 <= len(placements) <= 4:
        raise ValueError(f"计划必须包含 1--4 片 placements，实际 {len(placements)}")

    pieces = []
    for p in placements:
        src = p["source"]
        tgt = p["target"]
        pieces.append(
            {
                "piece_id": p["piece_id"],
                "src_mm": tuple(src["center_mm"]),
                "src_yaw": float(src["yaw_deg"]),
                "tgt_mm": tuple(tgt["center_mm"]),
                "tgt_yaw": float(tgt["yaw_deg"]),
            }
        )

    print(f"加载 {len(pieces)} 片计划:")
    for p in pieces:
        delta = p["tgt_yaw"] - p["src_yaw"]
        print(
            f"  {p['piece_id']}: 源({p['src_mm'][0]:.1f},{p['src_mm'][1]:.1f}) "
            f"yaw={p['src_yaw']:.1f} → 目标({p['tgt_mm'][0]:.1f},{p['tgt_mm'][1]:.1f}) "
            f"yaw={p['tgt_yaw']:.1f}  Δyaw={delta:.1f}"
        )

    # 硬件初始化（dry-run 也建 kinematics，只不连串口）
    adapter = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_POSE_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=pick_place_motion_config(),
    )

    def yaw_pose(ref: PieceRef, height: float, yaw: float) -> ToolPose:
        base = adapter.pose_for(ref, height)
        return ToolPose(base.frame, base.position_mm, base.pitch_deg, yaw)

    # 全路径 IK 预检：逐点可达性（不检查相邻点步长——
    # MotionController.move 有翻倍加密重试，AGENTS.md 记录过 preflight
    # 的相邻点步长检查对非线性 IK 过于严格，会误报不可达）。
    waypoints = [adapter.home_pose()]
    for p in pieces:
        src_ref = PieceRef(p["piece_id"], p["src_mm"])
        tgt_ref = PieceRef(f"{p['piece_id']}-place", p["tgt_mm"])
        waypoints += [
            yaw_pose(src_ref, SAFE_HEIGHT_MM, p["src_yaw"]),
            yaw_pose(src_ref, PREGRASP_HEIGHT_MM, p["src_yaw"]),
            yaw_pose(src_ref, heights.pick_mm, p["src_yaw"]),
            yaw_pose(src_ref, SAFE_HEIGHT_MM, p["src_yaw"]),
            yaw_pose(tgt_ref, SAFE_HEIGHT_MM, p["tgt_yaw"]),
            yaw_pose(tgt_ref, PREGRASP_HEIGHT_MM, p["tgt_yaw"]),
            yaw_pose(tgt_ref, heights.place_mm, p["tgt_yaw"]),
            yaw_pose(tgt_ref, SAFE_HEIGHT_MM, p["tgt_yaw"]),
        ]
    waypoints.append(adapter.home_pose())
    print("\n全路径 IK 预检（逐点可达性）...")
    for index, waypoint in enumerate(waypoints):
        try:
            adapter.kinematics.inverse(waypoint)
        except ValueError as error:
            raise ValueError(f"路径点 {index} IK 不可达: {error}") from error
    print("✓ 全部关键点 IK 可达（相邻点步长交由 MotionController 加密插值处理）\n")

    if args.dry_run:
        print("--dry-run：仅 IK 预检，不下发运动。")
        return

    # 连接硬件
    adapter.connect()
    magnet = MagnetController()
    magnet.connect()
    magnet.set_enabled(False)

    # 用计划源位构造视觉批次（grasp_fsm 校验 plan 必须绑定当前批次）
    batch = VisionBatch(
        f"plan-{time.monotonic_ns()}",
        hashlib.sha256(args.plan.read_bytes()).hexdigest(),
        time.monotonic_ns(),
        tuple(PieceRef(p["piece_id"], p["src_mm"]) for p in pieces),
    )
    active = Supervisor(
        make_config(shadow=False, heights=heights),
        adapter.motion,
        adapter.park_mover,
        magnet,
        _PlanVision(batch),
        None,
        sweep_kinematics=adapter.kinematics,
    )

    try:
        # 绑定视觉批次：Supervisor.scan 会校验并设置 self._batch，
        # grasp_fsm 的 validate_plan 依赖它
        batch = active.scan()
        print(f"视觉批次: {batch.batch_id} ({len(batch.pieces)} 片)\n")

        # PARK → HOME
        print("PARK → HOME...")
        res = adapter.park_mover.park_to_home()
        if not res.success:
            raise SupervisorFailure(
                SupervisorErrorCode.PARK_TRANSITION_FAILED, f"PARK→HOME 失败: {res.message}"
            )

        for idx, p in enumerate(pieces, 1):
            print(f"\n=== 片 {idx}/4: {p['piece_id']} ===")
            src_ref = PieceRef(p["piece_id"], p["src_mm"])
            tgt_ref = PieceRef(f"{p['piece_id']}-place", p["tgt_mm"])
            pick_pose = yaw_pose(src_ref, heights.pick_mm, p["src_yaw"])
            place_pose = yaw_pose(tgt_ref, heights.place_mm, p["tgt_yaw"])
            plan_ref = PlacementPlanRef(batch.source_image_sha256, batch.monotonic_ns)
            active.grasp_fsm(plan_ref, piece=src_ref, pick_pose=pick_pose, place_pose=place_pose)
            print(f"✓ {p['piece_id']} 完成")
            time.sleep(1.0)

        # HOME → PARK
        print("\nHOME → PARK...")
        res = adapter.park_mover.home_to_park()
        if not res.success:
            raise SupervisorFailure(
                SupervisorErrorCode.PARK_TRANSITION_FAILED, f"HOME→PARK 失败: {res.message}"
            )
        print("\n✓ 四片拼装完成，机械臂已回 PARK")
    finally:
        magnet.emergency_off()
        adapter.disconnect()


if __name__ == "__main__":
    main()
