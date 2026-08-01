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

import numpy as np
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
        poll_interval_s=0.05,
        waypoint_timeout_s=8.0,
        safe_min_z_mm=-20.0,
        contact_min_z_mm=-30.0,
        max_z_mm=180.0,
        transit_cartesian_step_mm=25.0,
        follow_tolerance_deg=8.0,
        # 第一阶段仅提速空载/接近；持物速度保持保守档。
        goal_velocity_by_profile={
            "EMPTY_NORMAL": 250,
            "EMPTY_SLOW": 150,
            "HOLDING_SLOW": 100,
        },
        waypoint_min_dwell_s_by_profile={
            "EMPTY_NORMAL": 0.15,
            "EMPTY_SLOW": 0.20,
            "HOLDING_SLOW": 0.30,
            "LOCAL_POLICY": 0.15,
        },
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
            f"yaw=0.0(约定) → 目标({p['tgt_mm'][0]:.1f},{p['tgt_mm'][1]:.1f}) "
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
        # 详细运动日志：目标位姿/规划/逐 waypoint 读回/收敛误差
        motion_detail_log_dir=Path("/opt/epuzzle/workspace/logs/motion_detail"),
    )
    print(f"详细运动日志: {adapter.motion_detail_log.path if adapter.motion_detail_log else 'disabled'}")

    # table_mm -> robot_base_mm 的坐标系旋转（contact_sheet_to_robot_base 标定）
    TABLE_TO_ROBOT_YAW_DEG = 4.43177

    def yaw_pose(ref: PieceRef, height: float, yaw_table_deg: float) -> ToolPose:
        """构造带 yaw 的 ToolPose。

        yaw_table_deg 是 0° 基准下的碎片朝向（求解器相对角或 0），
        ToolPose.yaw_deg 必须是 robot_base_mm 系（kinematics 里 yaw = pan + roll），
        因此加 TABLE_TO_ROBOT_YAW_DEG 做坐标系旋转。
        """
        base = adapter.pose_for(ref, height)
        return ToolPose(
            base.frame,
            base.position_mm,
            base.pitch_deg,
            yaw_table_deg + TABLE_TO_ROBOT_YAW_DEG,
        )

    def place_yaw_with_roll_compensation(
        src_ref: PieceRef,
        tgt_ref: PieceRef,
        src_yaw_table: float,
        tgt_yaw_table: float,
    ) -> tuple[float, float]:
        """计算 PICK/PLACE 的 robot 系 yaw，保证碎片旋转 ≡ Δyaw (mod 360)。

        0° 基准约定：src_yaw_table 恒为 0（不测量碎片绝对朝向），
        tgt_yaw_table 即求解器相对旋转角。

        物理约束：wrist_roll 单圈（STS3215 0..4095 ticks），逻辑限位 [-180,180]
        → 物理限位 [logical*direction+offset] = [-262.15, +97.85]（direction=-1,
        offset=-82.15）。碎片旋转 = 末端 yaw 变化 = Δpan + Δroll_physical，
        必须等于求解器 Δyaw（模 360）。Δyaw 等价角 {Δyaw, Δyaw±360} 中选
        使物理 roll 落入限位的那个。
        """
        geom = adapter.kinematics.geometry
        roll_dir = geom.joint_directions[4]
        roll_off = geom.joint_zero_offsets_deg[4]
        roll_limits_logical = geom.joint_limits_deg[4]
        # 物理限位 = 逻辑*direction + offset
        roll_lo = min(roll_limits_logical[0] * roll_dir + roll_off,
                      roll_limits_logical[1] * roll_dir + roll_off)
        roll_hi = max(roll_limits_logical[0] * roll_dir + roll_off,
                      roll_limits_logical[1] * roll_dir + roll_off)

        # PICK yaw：用 pose_for 径向默认 yaw（robot 系），保证 HOME→PICK
        # 路径平滑。实机发现：用视觉 angle_deg 作为 PICK yaw 时，某些位置
        # （如右上角 piece_03 yaw=73°）IK 出现多解突变，lift 单步跳 47°。
        # 碎片被磁铁吸住后跟随磁头转，最终朝向由 PLACE yaw = PICK yaw + Δyaw
        # 决定，PICK 的绝对 yaw 不影响拼接结果。
        # （0° 基准：碎片初始朝向视为 0，Δyaw = tgt_yaw − 0 即求解器相对角，
        #   碎片相对桌面旋转 = 磁头 yaw 变化 = Δpan + Δroll。）
        pick_base = adapter.pose_for(src_ref, 60.0)
        pick_yaw = pick_base.yaw_deg  # 径向默认（robot 系）
        pick_pose = ToolPose(
            pick_base.frame,
            pick_base.position_mm,
            pick_base.pitch_deg,
            pick_yaw,
        )
        pick_jv = adapter.kinematics.inverse(pick_pose)
        pm = pick_jv.as_mapping()
        pan_pick_log = pm["shoulder_pan.pos"]
        roll_pick_log = pm["wrist_roll.pos"]
        # 物理角
        pan_pick_phys = pan_pick_log * geom.joint_directions[0] + geom.joint_zero_offsets_deg[0]
        roll_pick_phys = roll_pick_log * roll_dir + roll_off

        # PLACE pan（位置决定，与 yaw 无关）
        place_base = adapter.pose_for(tgt_ref, 60.0)
        pan_place_phys = float(
            np.degrees(
                np.arctan2(place_base.position_mm[1], place_base.position_mm[0])
            )
        )
        delta_yaw = (tgt_yaw_table - src_yaw_table + 180.0) % 360.0 - 180.0
        candidates = [delta_yaw, delta_yaw + 360.0, delta_yaw - 360.0]
        chosen: float | None = None
        chosen_roll: float = 0.0
        for cand in candidates:
            roll_place_phys = roll_pick_phys + cand - (pan_place_phys - pan_pick_phys)
            if roll_lo - 1e-6 <= roll_place_phys <= roll_hi + 1e-6:
                chosen = cand
                chosen_roll = roll_place_phys
                print(
                    f"    补偿: Δyaw={delta_yaw:.1f}° 取等价角 {cand:+.1f}° | "
                    f"pan_phys {pan_pick_phys:+.1f}→{pan_place_phys:+.1f} | "
                    f"roll_phys {roll_pick_phys:+.1f}→{roll_place_phys:+.1f} ✓"
                )
                break
        if chosen is None:
            # 兜底：取最接近限位的等价角
            best_cand = None
            best_dist = 1e9
            for cand in candidates:
                rp = roll_pick_phys + cand - (pan_place_phys - pan_pick_phys)
                dist = max(0.0, roll_lo - rp, rp - roll_hi)
                if dist < best_dist:
                    best_dist = dist
                    best_cand = cand
                    chosen_roll = rp
            chosen = best_cand
            print(
                f"    ⚠ 补偿: 无限位内等价角，取 {chosen:+.1f}° | "
                f"roll_phys {roll_pick_phys:+.1f}→{chosen_roll:+.1f} (越限 {best_dist:.1f}°)"
            )
        return pick_yaw, pick_yaw + chosen

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
            # 完成状态持久化：重跑时跳过已完成片，避免 USB 掉线崩溃后
            # 重复抓放已放置的碎片（实机踩坑：piece_01 被二次抓取放错）。
            state_path = Path("/home/epuzzle/assembly_state.json")
            done = set()
            if state_path.exists():
                try:
                    done = set(json.loads(state_path.read_text()).get("done", []))
                except (OSError, json.JSONDecodeError):
                    done = set()
            if p["piece_id"] in done:
                print(f"  跳过（已完成）")
                continue
            src_ref = PieceRef(p["piece_id"], p["src_mm"])
            tgt_ref = PieceRef(f"{p['piece_id']}-place", p["tgt_mm"])
            # 旋转补偿：选 Δyaw 等价角使 roll 在限位内，保证碎片旋转 ≡ Δyaw
            pick_yaw_robot, place_yaw_robot = place_yaw_with_roll_compensation(
                src_ref, tgt_ref, p["src_yaw"], p["tgt_yaw"]
            )
            pick_pose = yaw_pose(src_ref, heights.pick_mm, p["src_yaw"])
            # 用补偿后的 robot 系 yaw 覆盖 pick/place 姿态
            pick_pose = ToolPose(
                pick_pose.frame, pick_pose.position_mm, pick_pose.pitch_deg, pick_yaw_robot
            )
            place_pose = yaw_pose(tgt_ref, heights.place_mm, p["tgt_yaw"])
            place_pose = ToolPose(
                place_pose.frame, place_pose.position_mm, place_pose.pitch_deg, place_yaw_robot
            )
            # 验证实际碎片旋转 = Δyaw（模 360）
            pick_jv = adapter.kinematics.inverse(pick_pose)
            place_jv = adapter.kinematics.inverse(place_pose)
            pm = pick_jv.as_mapping()
            lm = place_jv.as_mapping()
            actual_rot = (lm["shoulder_pan.pos"] + lm["wrist_roll.pos"]) - (
                pm["shoulder_pan.pos"] + pm["wrist_roll.pos"]
            )
            delta_yaw = (p["tgt_yaw"] - p["src_yaw"] + 180.0) % 360.0 - 180.0
            ok = abs((actual_rot - delta_yaw + 180.0) % 360.0 - 180.0) < 0.5
            print(
                f"  旋转验证: 实际 {actual_rot:+.1f}° vs Δyaw {delta_yaw:+.1f}° "
                f"({'✓' if ok else '✗ 偏差!'})"
            )
            plan_ref = PlacementPlanRef(batch.source_image_sha256, batch.monotonic_ns)
            active.grasp_fsm(plan_ref, piece=src_ref, pick_pose=pick_pose, place_pose=place_pose)
            done.add(p["piece_id"])
            state_path.write_text(json.dumps({"done": sorted(done)}))
            print(f"✓ {p['piece_id']} 完成（进度 {len(done)}/4）")
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
