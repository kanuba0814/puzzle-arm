#!/usr/bin/env python3
# pyright: reportMissingImports=false

"""Supervisor CLI for shadow planning and the frozen magnet-off inspection modes."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from control.magnet_controller import MagnetConfig, MagnetController
from control.motion_controller import MotionConfig, MotionGoal
from integration import (
    PlacementPlanRef,
    ShadowPlanReport,
    Supervisor,
    SupervisorConfig,
    SupervisorErrorCode,
    SupervisorFailure,
    VisionBatch,
)
from integration.competition_backend import pick_place_lock_reason
from integration.runtime_adapter import (
    PREGRASP_HEIGHT_MM,
    SAFE_HEIGHT_MM,
    InspectionHardwareAdapter,
)
from integration.sweep_planner import PieceRef, preflight_continuous_sweep
from puzzle import (
    PuzzleOptions,
    PuzzleSolveStatus,
    TargetRegion,
    WorkspaceState,
    solve_puzzle,
)

WORKSPACE = Path("workspace")
CALIBRATION_PATH = (
    WORKSPACE / "calibration/arm_kinematics_single_arm_accepted_20260731.json"
)
TRANSFORM_PATH = WORKSPACE / "calibration/contact_sheet_to_robot_base_20260731.json"
PARK_PATH = WORKSPACE / "calibration/park_pose_single_arm_20260731.json"
HOMOGRAPHY_PATH = WORKSPACE / "calibration/table_homography_gui_20260731.npy"
DEFAULT_PIECES_PATH = WORKSPACE / "runtime/red_piece_detection/pieces.json"
DEFAULT_PORT = Path("/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB9067408-if00")
DEFAULT_LEROBOT_CALIBRATION = (
    Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower_5d"
)

COMMAND_ALIASES = {
    "detect": "scan",
    "scan": "scan",
    "red-baseline": "sweep",
    "sweep-shortest": "sweep",
    "sweep": "sweep",
    "repeat": "repeat",
    "pregrasp": "pregrasp",
    "park": "park",
    "puzzle-shadow": "shadow-plan",
    "shadow-plan": "shadow-plan",
    "pick-place": "pick-place",
    "competition-self-color": "competition-self-color",
    "competition-white": "competition-white",
    "competition-card": "competition-card",
    "manual-read-state": "manual-read-state",
    "manual-jog-joint": "manual-jog-joint",
    "manual-jog-cartesian": "manual-jog-cartesian",
    "manual-magnet-pulse": "manual-magnet-pulse",
    "manual-magnet-off": "manual-magnet-off",
    "commission-center": "commission-center",
}
CANONICAL_COMMAND = {
    "detect": "detect",
    "scan": "detect",
    "red-baseline": "red-baseline",
    "sweep-shortest": "sweep-shortest",
    "sweep": "sweep-shortest",
    "repeat": "repeat",
    "pregrasp": "pregrasp",
    "park": "park",
    "puzzle-shadow": "puzzle-shadow",
    "shadow-plan": "puzzle-shadow",
    "pick-place": "pick-place",
    "competition-self-color": "competition-self-color",
    "competition-white": "competition-white",
    "competition-card": "competition-card",
    "manual-read-state": "manual-read-state",
    "manual-jog-joint": "manual-jog-joint",
    "manual-jog-cartesian": "manual-jog-cartesian",
    "manual-magnet-pulse": "manual-magnet-pulse",
    "manual-magnet-off": "manual-magnet-off",
    "commission-center": "commission-center",
}
COMMISSION_CENTER_TABLE_MM = (105.0, 148.5)
COMMISSION_CENTER_HEIGHT_MM = 60.0
MAX_COMMISSION_HOLD_SECONDS = 300.0
PICK_PLACE_HEIGHT_PATH = WORKSPACE / "calibration/pick_place_contact_height.json"
VERTICAL_REFERENCE_FRAME = "calibration_sheet_mm"
MAX_MAGNET_SETTLE_S = 5.0


def motion_config() -> MotionConfig:
    """快速巡检档：15 mm 插值，关节步长仍严格不超过 10°。

    梯形速度：SAFE 空载 200 LSB ≈ 35°/s 峰值，PICK/PLACE 60 LSB ≈ 10.6°/s。
    """
    return MotionConfig(
        15.0,
        10.0,
        10.0,
        3.0,
        1,
        0.02,
        3.0,
        -20.0,
        -30.0,
        180.0,
        goal_velocity_by_profile={
            "EMPTY_NORMAL": 200,
            "EMPTY_SLOW": 120,
            "HOLDING_SLOW": 80,
        },
    )


def pick_place_motion_config() -> MotionConfig:
    """抓放档：非接触转移用 40mm 大步长 + 8° 跟随，减少路点数；
    接触目标（PICK/PLACE）仍用 15mm 精细分段 + 3° 严格容差。

    跟随容差只用于非接触转移；MotionController 对 PICK/PLACE phase 强制
    忽略跟随容差，因为负载下宽门槛会使 TCP 提前 10-20 mm 判到位。

    梯形速度：EMPTY_NORMAL 200 LSB（35°/s）空载快速转移，EMPTY_SLOW 120
    （21°/s）接近，HOLDING_SLOW 80（14°/s）负载慢速，减少冲击与滑移。
    """
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
        transit_cartesian_step_mm=25.0,  # 转移步长；40mm 曾使单步关节跳 10.14° 超限
        follow_tolerance_deg=8.0,
        goal_velocity_by_profile={
            "EMPTY_NORMAL": 200,
            "EMPTY_SLOW": 120,
            "HOLDING_SLOW": 80,
        },
    )


class JsonVision:
    def __init__(self, pieces_path: Path) -> None:
        self._pieces_path = pieces_path

    def scan(self) -> VisionBatch:
        try:
            payload = json.loads(self._pieces_path.read_text(encoding="utf-8"))
        except OSError as error:
            raise RuntimeError(f"无法读取碎片观察: {self._pieces_path}") from error
        except json.JSONDecodeError as error:
            raise ValueError(f"碎片观察不是有效 JSON: {self._pieces_path}") from error
        if not isinstance(payload, dict):
            raise ValueError("pieces.json 顶层必须是对象")
        raw_pieces = payload.get("pieces")
        if not isinstance(raw_pieces, list):
            raise ValueError("pieces.json.pieces 必须是数组")
        pieces = tuple(
            PieceRef(
                str(item["piece_id"]),
                _point(item["center_table_mm"], "center_table_mm"),
                _point(item["pick_point_table_mm"], "pick_point_table_mm")
                if "pick_point_table_mm" in item
                else None,
            )
            for item in raw_pieces
            if isinstance(item, dict)
        )
        if len(pieces) != len(raw_pieces):
            raise ValueError("pieces 必须全部为对象")
        source_image = self._pieces_path.parent / "frame.jpg"
        image_ref = (
            hashlib.sha256(source_image.read_bytes()).hexdigest()
            if source_image.exists()
            else hashlib.sha256(self._pieces_path.read_bytes()).hexdigest()
        )
        return VisionBatch(
            f"cli-{time.monotonic_ns()}", image_ref, time.monotonic_ns(), pieces
        )


def _point(value: object, field: str) -> tuple[float, float]:
    if not isinstance(value, list | tuple) or len(value) != 2:
        raise ValueError(f"{field} 必须为两个 table_mm 数值")
    try:
        point = float(value[0]), float(value[1])
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须为两个 table_mm 数值") from error
    if not all(math.isfinite(coordinate) for coordinate in point):
        raise ValueError(f"{field} 必须为两个有限 table_mm 数值")
    return point


@dataclass
class JsonPuzzlePiece:
    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float
    timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]


class GeometryShadowSolver:
    """把当前序列化观察送入纯 puzzle 模块；永不发送动作。"""

    def __init__(self, pieces_path: Path) -> None:
        self._pieces_path = pieces_path

    def solve(self, batch: VisionBatch) -> ShadowPlanReport:
        try:
            payload = json.loads(self._pieces_path.read_text(encoding="utf-8"))
        except OSError as error:
            raise RuntimeError(
                f"无法读取 shadow 碎片观察: {self._pieces_path}"
            ) from error
        except json.JSONDecodeError as error:
            raise ValueError("shadow 碎片观察不是有效 JSON") from error
        if not isinstance(payload, dict) or not isinstance(payload.get("pieces"), list):
            raise ValueError("shadow 碎片观察缺少 pieces 数组")
        pieces = tuple(
            self._piece(item, index) for index, item in enumerate(payload["pieces"])
        )
        result = solve_puzzle(
            WorkspaceState(pieces),
            TargetRegion("table_mm", 0.0, 0.0, 210.0, 148.5),
            PuzzleOptions(
                time_budget_ms=10_000,
                max_expansions=20_000,
                # 白色纯色碎片（题面 2.(1)）只有几何判据：所有通过评分的装配都
                # 满足“位置正确、不重叠”，因此取确定性最优解（gap=0.0 从不判
                # 歧义）。两片相同矩形等对称材料存在多个同样合法的装配，
                # 保守 None 会拒绝可解任务；扑克牌纹理任务接入时必须恢复保守值。
                ambiguity_score_gap=0.0,
                partial_offset_step_mm=1.0,
                abs_tolerance_mm=2.0,
                rel_tolerance=0.03,
                vertex_tolerance_mm=2.0,
            ),
        )
        error_code = self._error_code(result.status, result.error_code)
        return ShadowPlanReport(
            result.status.value,
            tuple(plan.as_mapping() for plan in result.placements),
            {
                "best_score": result.best_score,
                "second_best_score": result.second_best_score,
                "score_gap": result.score_gap,
                "candidate_count": result.candidates_after_dedup,
                "inferred_target_mm": result.inferred_target,
                "score_breakdown": result.score_breakdown,
                "expansions": result.expansions,
                "pruned": result.pruned,
            },
            result.solver_version,
            batch.source_image_sha256,
            error_code,
        )

    @staticmethod
    def _piece(item: object, index: int) -> JsonPuzzlePiece:
        if not isinstance(item, dict):
            raise ValueError(f"pieces[{index}] 必须是对象")
        polygon_value = item.get("polygon_table_mm")
        if not isinstance(polygon_value, list) or not 3 <= len(polygon_value) <= 5:
            raise ValueError(f"pieces[{index}].polygon_table_mm 必须有 3--5 点")
        polygon = tuple(
            _point(point, f"pieces[{index}].polygon_table_mm")
            for point in polygon_value
        )
        yaw = _finite_number(item.get("yaw_deg", 0.0), f"pieces[{index}].yaw_deg")
        timestamp_value = item.get("timestamp_ns", 0)
        if isinstance(timestamp_value, bool) or not isinstance(timestamp_value, int):
            raise ValueError(f"pieces[{index}].timestamp_ns 必须是整数")
        equivalent_value = item.get("equivalent_yaws_deg", [])
        if not isinstance(equivalent_value, list):
            raise ValueError(f"pieces[{index}].equivalent_yaws_deg 必须是数组")
        return JsonPuzzlePiece(
            piece_id=str(item.get("piece_id", f"piece-{index}")),
            polygon_table_mm=polygon,
            center_table_mm=_point(
                item.get("center_table_mm"), f"pieces[{index}].center_table_mm"
            ),
            yaw_deg=yaw,
            timestamp_ns=timestamp_value,
            equivalent_yaws_deg=tuple(
                _finite_number(value, f"pieces[{index}].equivalent_yaws_deg")
                for value in equivalent_value
            ),
        )

    @staticmethod
    def _error_code(
        status: PuzzleSolveStatus, detail: str | None
    ) -> SupervisorErrorCode | None:
        if status is PuzzleSolveStatus.SOLVED:
            return None
        if status is PuzzleSolveStatus.AMBIGUOUS:
            return SupervisorErrorCode.SOLVER_AMBIGUOUS
        if detail == "SOLVER_TIMEOUT":
            return SupervisorErrorCode.SOLVER_TIMEOUT
        return SupervisorErrorCode.SOLVER_NO_SOLUTION


def _finite_number(value: object, field: str) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{field} 必须是有限数值") from error
    if not math.isfinite(number):
        raise ValueError(f"{field} 必须是有限数值")
    return number


def build_parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser()
    result.add_argument("mode", choices=tuple(COMMAND_ALIASES))
    result.add_argument("--pieces", type=Path, default=DEFAULT_PIECES_PATH)
    result.add_argument("--piece-index", type=int, default=0)
    result.add_argument(
        "--place-table-mm",
        default=None,
        help="pick-place 落位点 table_mm，形式 X,Y；本模式不从求解器推断",
    )
    result.add_argument(
        "--camera",
        default=os.environ.get("EPUZZLE_CAMERA_DEVICE", "/dev/epuzzle-camera"),
    )
    result.add_argument("--port", type=Path, default=DEFAULT_PORT)
    result.add_argument(
        "--material",
        default="red",
        help="碎片材料 profile：red（legacy 确定性路径）、white（竞赛）或 green/blue（开发验证）",
    )
    result.add_argument(
        "--lerobot-calibration", type=Path, default=DEFAULT_LEROBOT_CALIBRATION
    )
    result.add_argument("--output", type=Path)
    result.add_argument("--result-file", type=Path)
    result.add_argument("--status-file", type=Path)
    result.add_argument("--target-angle", type=float, default=0.0)
    result.add_argument("--hold-seconds", type=float, default=300.0)
    # 手动页参数。
    result.add_argument("--joint-feature", type=str, default="shoulder_pan")
    result.add_argument("--delta-deg", type=float, default=0.0)
    result.add_argument("--axis", type=str, default="x")
    result.add_argument("--delta", type=float, default=0.0)
    result.add_argument("--frame", type=str, default="robot_base_mm")
    result.add_argument("--duration-s", type=float, default=3.0)
    return result


def refresh_detection(args: argparse.Namespace, material: str = "red") -> None:
    """Capture a new frame before hardware connection; never consume stale JSON."""
    script = "detect_generic_pieces.py" if material != "red" else "detect_red_pieces.py"
    command = [
        sys.executable,
        str(WORKSPACE / "tools" / script),
        "--homography",
        str(HOMOGRAPHY_PATH),
        "--camera",
        str(args.camera),
        "--output-dir",
        str(args.pieces.parent),
    ]
    if material != "red":
        command.extend(["--material", material])
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        detail = (
            completed.stderr.strip() or completed.stdout.strip() or "检测工具无输出"
        )
        raise RuntimeError(f"相机同步检测失败: {detail}")
    if not args.pieces.exists():
        raise RuntimeError("相机检测未产生 pieces.json")


@dataclass(frozen=True)
class PickPlaceContactHeights:
    """已验收的 PICK/PLACE 接触高度（calibration_sheet_mm）。"""

    pick_mm: float
    place_mm: float
    magnet_on_settle_s: float
    magnet_off_settle_s: float
    calibration_id: str
    single_piece_success_rate: float


def load_pick_place_heights(
    path: Path | None = None,
) -> PickPlaceContactHeights:
    """加载并校验 PICK/PLACE 接触高度验收文件。

    fail-closed：文件缺失、未验收、成功率不足、高度或安定时间缺失时均抛
    ``SupervisorFailure``，绝不回退到估计值。
    """
    target = path or PICK_PLACE_HEIGHT_PATH
    lock_reason = pick_place_lock_reason(target)
    if lock_reason is not None:
        raise SupervisorFailure(SupervisorErrorCode.SUPERVISOR_LOCKED, lock_reason)
    try:
        payload = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"接触高度验收文件无法读取: {error}",
        ) from error
    if not isinstance(payload, dict):
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED, "接触高度验收文件顶层必须是对象"
        )
    if payload.get("reference_frame") != VERTICAL_REFERENCE_FRAME:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"接触高度参考系必须是 {VERTICAL_REFERENCE_FRAME}",
        )
    heights = payload.get("contact_heights_mm")
    if not isinstance(heights, dict):
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            "接触高度验收文件缺少 contact_heights_mm",
        )
    try:
        pick_mm = _finite_number(heights.get("pick"), "contact_heights_mm.pick")
        place_mm = _finite_number(heights.get("place"), "contact_heights_mm.place")
    except ValueError as error:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED, f"接触高度无效: {error}"
        ) from error
    for name, value in (("pick", pick_mm), ("place", place_mm)):
        if not 0.0 <= value <= PREGRASP_HEIGHT_MM:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED,
                f"{name} 接触高度 {value} mm 超出 [0, {PREGRASP_HEIGHT_MM}] 安全区间",
            )

    verification = payload.get("magnet_verification")
    if not isinstance(verification, dict):
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            "接触高度验收文件缺少 magnet_verification",
        )
    if not verification.get("pickup_tested") or not verification.get("release_tested"):
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            "磁铁吸附/释放尚未完成实机验证",
        )
    try:
        on_settle_s = _finite_number(
            verification.get("on_settle_s"), "magnet_verification.on_settle_s"
        )
        off_settle_s = _finite_number(
            verification.get("off_settle_s"), "magnet_verification.off_settle_s"
        )
    except ValueError as error:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"磁铁安定时间必须实测并写入验收文件: {error}",
        ) from error
    if not 0.0 <= on_settle_s <= MAX_MAGNET_SETTLE_S:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"on_settle_s 超出 (0, {MAX_MAGNET_SETTLE_S}] 范围",
        )
    if not 0.0 <= off_settle_s <= MAX_MAGNET_SETTLE_S:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"off_settle_s 超出 (0, {MAX_MAGNET_SETTLE_S}] 范围",
        )

    return PickPlaceContactHeights(
        pick_mm=pick_mm,
        place_mm=place_mm,
        magnet_on_settle_s=on_settle_s,
        magnet_off_settle_s=off_settle_s,
        calibration_id=str(payload.get("calibration_id", "UNKNOWN")),
        single_piece_success_rate=_finite_number(
            payload.get("single_piece_success_rate", 0.0),
            "single_piece_success_rate",
        ),
    )


def config(
    *,
    shadow: bool,
    log_path: Path,
    heights: PickPlaceContactHeights | None = None,
) -> SupervisorConfig:
    return SupervisorConfig(
        calibration_path=CALIBRATION_PATH,
        park_pose_path=PARK_PATH,
        transform_path=TRANSFORM_PATH,
        # 接触纸→robot_base 变换已在 20260731 完成实机标定并冻结
        transform_is_official=True,
        # 只在抓放（heights 已验收）启用跟随容差；巡检/PARK 保持原行为
        motion_config=motion_config()
        if heights is None
        else pick_place_motion_config(),
        z_safe_mm=SAFE_HEIGHT_MM,
        z_pregrasp_mm=PREGRASP_HEIGHT_MM,
        z_pick_mm=None if heights is None else heights.pick_mm,
        z_place_mm=None if heights is None else heights.place_mm,
        park_tolerance_deg=4,
        park_follow_tolerance_deg=8,
        max_vision_age_ms=30_000,
        global_timeout_s=120,
        shadow=shadow,
        automatic_pick_place_enabled=heights is not None,
        precision_radial_compensation_mm=3,
        precision_pitch_command_deg=-87.5,
        event_log_path=log_path,
        # 磁头净空随接触高度验收一同冻结；未验收时保持 fail-closed
        magnet_footprint_clearance_frozen=heights is not None,
        magnet_on_settle_s=None if heights is None else heights.magnet_on_settle_s,
        magnet_off_settle_s=None if heights is None else heights.magnet_off_settle_s,
    )


def _atomic_write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def write_status(args: argparse.Namespace, phase: str, progress: float) -> None:
    if args.status_file is None:
        return
    _atomic_write_json(
        args.status_file,
        {
            "schema_version": 1,
            "command": args.command,
            "phase": phase,
            "progress": max(0.0, min(1.0, progress)),
            "updated_at_ns": time.monotonic_ns(),
        },
    )


def outcome(
    *,
    success: bool,
    error_code: str,
    message: str,
    mode: str,
    route: list[str],
    duration_ms: int,
    final_state: str,
    events_path: str,
) -> dict[str, object]:
    return {
        "success": success,
        "error_code": error_code,
        "message": message,
        "mode": mode,
        "route": route,
        "duration_ms": duration_ms,
        "final_state": final_state,
        "events_path": events_path,
    }


def robot_motion_lock_reason() -> str | None:
    lock_path = Path(
        os.environ.get(
            "EPUZZLE_ROBOT_MOTION_LOCK",
            "/etc/epuzzle/robot-motion.lock.json",
        )
    )
    if not lock_path.exists():
        return None
    try:
        payload = json.loads(lock_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        return f"运动安全锁无法解析: {lock_path}: {error}"
    if not isinstance(payload, dict):
        return f"运动安全锁必须是 JSON 对象: {lock_path}"
    reason = payload.get("reason")
    return str(reason) if reason else f"存在未解除的运动安全锁: {lock_path}"


def classify_unhandled_error(
    error: BaseException, *, mode: str, robot_port: Path
) -> SupervisorErrorCode:
    """只把明确的机械臂设备丢失/通信 I/O 故障映射为通信错误。"""
    if mode not in {"sweep", "repeat", "pregrasp", "park", "commission-center"}:
        return SupervisorErrorCode.SAFETY_REJECTED
    message = str(error)
    if message.startswith("相机同步检测失败:"):
        return SupervisorErrorCode.SAFETY_REJECTED
    current: BaseException | None = error
    for _ in range(8):
        if isinstance(current, OSError) and current.errno in {5, 19, 110}:
            return SupervisorErrorCode.ROBOT_COMM_TIMEOUT
        next_error = current.__cause__ or current.__context__
        if next_error is None or next_error is current:
            break
        current = next_error
    if any(
        marker in message
        for marker in (
            "[Errno 19]",
            "No such device",
            "device reports readiness to read but returned no data",
        )
    ):
        return SupervisorErrorCode.ROBOT_COMM_TIMEOUT
    if not robot_port.exists() and any(
        marker in message.lower()
        for marker in ("serial", "port", "robot", "机械臂", "串口")
    ):
        return SupervisorErrorCode.ROBOT_COMM_TIMEOUT
    return SupervisorErrorCode.SAFETY_REJECTED


def gui_result(
    payload: dict[str, object], *, command: str, started_at_ns: int
) -> dict[str, object]:
    success_value = payload.get("success")
    success = isinstance(success_value, bool) and success_value
    duration_value = payload.get("duration_ms", 0)
    try:
        duration_ms = (
            float(duration_value)
            if isinstance(duration_value, int | float)
            and not isinstance(duration_value, bool)
            else 0.0
        )
    except (TypeError, ValueError, OverflowError):
        duration_ms = 0.0
    if not math.isfinite(duration_ms):
        duration_ms = 0.0
    return {
        "schema_version": 1,
        "command": command,
        "result": "PASS" if success else "FAIL",
        "magnet_enabled": False,
        "message": str(payload.get("message", "")),
        "started_at_ns": started_at_ns,
        "duration_s": duration_ms / 1000.0,
        "events": [
            {
                "error_code": payload.get("error_code"),
                "route": payload.get("route", []),
                "final_state": payload.get("final_state"),
                "events_path": payload.get("events_path"),
            }
        ],
        **payload,
    }


def _magnet_config_from_environment() -> MagnetConfig:
    try:
        line_offset = int(os.environ.get("EPUZZLE_MAGNET_GPIO_LINE", "17"))
        max_on_seconds = float(os.environ.get("EPUZZLE_MAGNET_MAX_ON_SECONDS", "10.0"))
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("电磁铁 GPIO line/通电上限配置无效") from error
    active_high = os.environ.get("EPUZZLE_MAGNET_ACTIVE_HIGH") == "1"
    return MagnetConfig(
        chip_path=os.environ.get("EPUZZLE_MAGNET_GPIO_CHIP", "/dev/gpiochip0"),
        line_offset=line_offset,
        active_high=active_high,
        max_on_seconds=max_on_seconds,
    )


def run_commission_center(
    args: argparse.Namespace, started_at: float
) -> dict[str, object]:
    """Move to the A4 center at 60 mm and hold a signal-controlled session."""
    if not 0.0 < args.hold_seconds <= MAX_COMMISSION_HOLD_SECONDS:
        raise ValueError(
            f"hold_seconds 必须在 (0, {MAX_COMMISSION_HOLD_SECONDS:.0f}] 范围内"
        )
    motion_lock_reason = robot_motion_lock_reason()
    if motion_lock_reason is not None:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"机械臂运动安全锁: {motion_lock_reason}",
        )
    if not args.port.exists():
        raise SupervisorFailure(
            SupervisorErrorCode.ROBOT_COMM_TIMEOUT,
            f"机械臂设备离线: {args.port}",
        )

    log_path = WORKSPACE / "logs/supervisor" / "commission-center.jsonl"
    hardware = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=motion_config(),
    )
    magnet: MagnetController | None = None
    stop_requested = False
    emergency_requested = False
    magnet_on_requested = False
    magnet_off_requested = False
    magnet_was_enabled = False
    previous_handlers: dict[int, Any] = {}

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    def request_emergency_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested, emergency_requested
        emergency_requested = True
        stop_requested = True

    def request_magnet_on(_signum: int, _frame: object) -> None:
        nonlocal magnet_on_requested
        magnet_on_requested = True

    def request_magnet_off(_signum: int, _frame: object) -> None:
        nonlocal magnet_off_requested
        magnet_off_requested = True

    try:
        for signum, handler in (
            (signal.SIGUSR1, request_magnet_on),
            (signal.SIGUSR2, request_magnet_off),
            (signal.SIGHUP, request_stop),
            (signal.SIGTERM, request_emergency_stop),
            (signal.SIGINT, request_emergency_stop),
        ):
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, handler)

        write_status(args, "bumpless 连接五轴", 0.10)
        hardware.connect()
        if hardware.motion is None or hardware.park_mover is None:
            raise RuntimeError("机械臂适配器未完成运动初始化")
        center_piece = PieceRef("commission-center", COMMISSION_CENTER_TABLE_MM)
        center_pose = hardware.pose_for(center_piece, COMMISSION_CENTER_HEIGHT_MM)
        preflight_continuous_sweep(
            hardware.kinematics,
            motion_config(),
            (hardware.home_pose(), center_pose, hardware.home_pose()),
        )

        write_status(args, "PARK→HOME", 0.25)
        parked_to_home = hardware.park_mover.park_to_home()
        if not parked_to_home.success:
            raise RuntimeError(f"PARK→HOME 失败: {parked_to_home.message}")
        write_status(args, "移动到中心 60mm", 0.45)
        deadline_ns = time.monotonic_ns() + int(args.hold_seconds * 1_000_000_000)
        motion_result = hardware.motion.move(
            MotionGoal(
                goal_id="commission-center-60mm",
                timestamp_ns=time.monotonic_ns(),
                frame="robot_base_mm",
                position_mm=center_pose.position_mm,
                pitch_deg=center_pose.pitch_deg,
                yaw_deg=center_pose.yaw_deg,
                phase="SAFE",
                speed_profile="EMPTY_SLOW",
                deadline_ns=deadline_ns,
            )
        )
        if not motion_result.success:
            raise RuntimeError(f"中心 60mm 未稳定到位: {motion_result.message}")

        connected_magnet = MagnetController(_magnet_config_from_environment())
        connected_magnet.connect()
        magnet = connected_magnet
        write_status(args, "中心60mm保持；磁铁关闭", 0.60)
        hold_deadline = time.monotonic() + args.hold_seconds
        last_enabled = False
        while not stop_requested and time.monotonic() < hold_deadline:
            if magnet_on_requested:
                magnet_on_requested = False
                magnet_off_requested = False
                connected_magnet.set_enabled(True)
                magnet_was_enabled = True
                write_status(args, "中心60mm保持；磁铁开启（10秒自动关闭）", 0.70)
            if magnet_off_requested:
                magnet_off_requested = False
                connected_magnet.set_enabled(False)
                write_status(args, "中心60mm保持；磁铁关闭", 0.70)
            enabled = connected_magnet.get_status().enabled
            if last_enabled and not enabled:
                write_status(args, "中心60mm保持；磁铁已自动关闭", 0.70)
            last_enabled = enabled
            time.sleep(0.05)
        if emergency_requested:
            raise SupervisorFailure(
                SupervisorErrorCode.ESTOP_ACTIVE,
                "中心60mm标定保持收到急停，已关闭磁铁并请求断力矩",
            )
        timed_out = time.monotonic() >= hold_deadline and not stop_requested
        write_status(args, "结束保持，准备回 PARK", 0.85)
        message = (
            "中心 60mm 保持到达时限，已自动关闭磁铁并回 PARK"
            if timed_out
            else "中心 60mm 标定保持已结束，磁铁已关闭并回 PARK"
        )
        return outcome(
            success=True,
            error_code=SupervisorErrorCode.NONE.value,
            message=message,
            mode=args.mode,
            route=[],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state="PARKED",
            events_path=str(log_path),
        ) | {"magnet_enabled": magnet_was_enabled}
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if magnet is not None:
            with contextlib.suppress(Exception):
                magnet.close()
        if hardware.robot is not None:
            if emergency_requested:
                hardware.disconnect()
            else:
                try:
                    parked, message = hardware.attempt_park()
                    if not parked:
                        raise RuntimeError(f"回正式 PARK 失败: {message}")
                finally:
                    hardware.disconnect()


def run_magnet_off(args: argparse.Namespace, started_at: float) -> dict[str, object]:
    """立即释放电磁铁；不依赖机械臂，但需要在目标 rootfs 上才能实际写入 GPIO。"""
    config = _magnet_config_from_environment()
    controller = MagnetController(config)
    try:
        controller.connect()
        write_status(args, "请求电磁铁断电", 0.2)
        status = controller.set_enabled(False)
        write_status(args, "电磁铁已释放", 0.95)
        return outcome(
            success=True,
            error_code=SupervisorErrorCode.NONE.value,
            message=f"电磁铁已释放（原因：{status.reason}）",
            mode=args.mode,
            route=[],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state="MAGNET_OFF",
            events_path=str(WORKSPACE / "logs/supervisor" / f"{args.command}.jsonl"),
        )
    finally:
        controller.close()


def run_magnet_pulse(args: argparse.Namespace, started_at: float) -> dict[str, object]:
    """有界电磁铁脉冲；duration 由 GUI 控制在 (0, 10] 秒内。"""
    duration = args.duration_s
    if not 0.0 < duration <= 10.0:
        raise ValueError("duration_s 必须在 (0, 10] 范围内")
    config = _magnet_config_from_environment()
    controller = MagnetController(config)
    try:
        controller.connect()
        write_status(args, "电磁铁吸合", 0.1)
        controller.set_enabled(True)
        write_status(args, "定时吸合中", 0.5)
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if not controller.get_status().enabled:
                break
            time.sleep(0.05)
        controller.set_enabled(False)
        write_status(args, "电磁铁已释放", 0.95)
        return outcome(
            success=True,
            error_code=SupervisorErrorCode.NONE.value,
            message=f"电磁铁 {duration:.1f}s 脉冲完成",
            mode=args.mode,
            route=[],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state="MAGNET_OFF",
            events_path=str(WORKSPACE / "logs/supervisor" / f"{args.command}.jsonl"),
        )
    finally:
        controller.close()


def run_manual_command(
    args: argparse.Namespace, started_at: float
) -> dict[str, object]:
    """手动运动命令占位：后端未接入时拒绝执行。"""
    raise SupervisorFailure(
        SupervisorErrorCode.SUPERVISOR_LOCKED,
        f"{args.command} 后端尚未接入，保持锁定",
    )


def run_hardware(args: argparse.Namespace, started_at: float) -> dict[str, object]:
    # This must happen before constructing or connecting a robot adapter.
    if args.mode in {"sweep", "repeat", "pregrasp"}:
        write_status(args, "刷新相机检测", 0.08)
        refresh_detection(args)
        write_status(args, "检测完成，准备连接机械臂", 0.18)
    motion_lock_reason = robot_motion_lock_reason()
    if motion_lock_reason is not None:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"机械臂运动安全锁: {motion_lock_reason}",
        )
    if not args.port.exists():
        raise SupervisorFailure(
            SupervisorErrorCode.ROBOT_COMM_TIMEOUT,
            f"机械臂设备离线: {args.port}",
        )
    vision = JsonVision(args.pieces)
    log_path = WORKSPACE / "logs/supervisor" / f"{args.mode}.jsonl"
    hardware = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=motion_config(),
    )
    route: list[str] = []
    error: BaseException | None = None
    park_failure = ""
    try:
        write_status(args, "bumpless 连接五轴", 0.25)
        hardware.connect()
        if hardware.motion is None or hardware.park_mover is None:
            raise RuntimeError("机械臂适配器未完成运动初始化")
        active_supervisor = Supervisor(
            config(shadow=False, log_path=log_path),
            hardware.motion,
            hardware.park_mover,
            None,
            vision,
            None,
            sweep_kinematics=hardware.kinematics,
        )
        if args.mode == "park":
            write_status(args, "回正式 PARK", 0.55)
            parked, message = hardware.attempt_park()
            if not parked:
                raise RuntimeError(f"回正式 PARK 失败: {message}")
            return outcome(
                success=True,
                error_code=SupervisorErrorCode.NONE.value,
                message=message,
                mode=args.mode,
                route=[],
                duration_ms=int((time.monotonic() - started_at) * 1000),
                final_state="PARKED",
                events_path=str(log_path),
            )
        write_status(args, "校验当前视觉批次", 0.32)
        batch = active_supervisor.scan()
        selected = batch.pieces
        if args.mode != "sweep":
            if not 0 <= args.piece_index < len(batch.pieces):
                raise ValueError("piece-index 超出新鲜检测结果范围")
            selected = (batch.pieces[args.piece_index],)
            batch = VisionBatch(
                batch.batch_id, batch.source_image_sha256, batch.monotonic_ns, selected
            )
        # repeat uses a second low center visit; sweep/pregrasp retain one visit.
        write_status(args, "全路径预检并执行", 0.42)
        if args.mode == "repeat":
            route_result = active_supervisor.sweep(
                batch,
                start_table_mm=hardware.home_projection_table_mm(),
                pose_for=hardware.pose_for,
                home_pose=hardware.home_pose(),
                low_repetitions=2,
            )
        else:
            route_result = active_supervisor.sweep(
                batch,
                start_table_mm=hardware.home_projection_table_mm(),
                pose_for=hardware.pose_for,
                home_pose=hardware.home_pose(),
            )
        route = [piece.piece_id for piece in route_result]
        write_status(args, "已回正式 PARK", 0.95)
        result = active_supervisor.result(
            True, SupervisorErrorCode.NONE, "磁铁关闭的巡检完成"
        )
        return outcome(
            success=True,
            error_code=result.error_code.value,
            message=result.message,
            mode=args.mode,
            route=route,
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state=result.final_state.value,
            events_path=result.events_path,
        )
    except BaseException as caught:
        error = caught
        raise
    finally:
        if hardware.robot is not None:
            try:
                parked, message = hardware.attempt_park()
                if not parked:
                    park_failure = message
            except BaseException as park_error:
                park_failure = f"回正式 PARK 时异常: {park_error}"
            finally:
                hardware.disconnect()
        if park_failure:
            detail = f"异常清理回正式 PARK 失败: {park_failure}"
            if error is not None:
                raise RuntimeError(f"{error}; {detail}") from error
            raise RuntimeError(detail)


def run_pick_place(args: argparse.Namespace, started_at: float) -> dict[str, object]:
    """已验收接触高度下的单片磁吸 PICK/PLACE。

    门禁顺序严格 fail-closed：先验收文件，再运动安全锁，再设备节点，
    最后才连接机械臂和继电器。任何一步失败都不会通电磁铁。
    """
    write_status(args, "校验 PICK/PLACE 接触高度验收", 0.05)
    heights = load_pick_place_heights()

    # 廉价参数校验：在连硬件之前完成
    place_table_mm = _place_target_table_mm(args)

    motion_lock_reason_value = robot_motion_lock_reason()
    if motion_lock_reason_value is not None:
        raise SupervisorFailure(
            SupervisorErrorCode.SUPERVISOR_LOCKED,
            f"机械臂运动安全锁: {motion_lock_reason_value}",
        )
    if not args.port.exists():
        raise SupervisorFailure(
            SupervisorErrorCode.ROBOT_COMM_TIMEOUT,
            f"机械臂设备离线: {args.port}",
        )

    write_status(args, "刷新相机检测", 0.12)
    refresh_detection(args, material=getattr(args, "material", "red"))

    log_path = WORKSPACE / "logs/supervisor" / "pick-place.jsonl"
    hardware = InspectionHardwareAdapter(
        calibration_path=CALIBRATION_PATH,
        transform_path=TRANSFORM_PATH,
        park_pose_path=PARK_PATH,
        port=args.port,
        lerobot_calibration_dir=args.lerobot_calibration,
        motion_config=pick_place_motion_config(),
    )
    magnet: MagnetController | None = None
    error: BaseException | None = None
    park_failure = ""
    try:
        write_status(args, "连接继电器", 0.2)
        connected_magnet = MagnetController(MagnetConfig())
        magnet = connected_magnet
        connected_magnet.connect()
        connected_magnet.set_enabled(False)

        write_status(args, "bumpless 连接五轴", 0.28)
        hardware.connect()
        if hardware.motion is None or hardware.park_mover is None:
            raise RuntimeError("机械臂适配器未完成运动初始化")

        active_supervisor = Supervisor(
            config(shadow=False, log_path=log_path, heights=heights),
            hardware.motion,
            hardware.park_mover,
            magnet,
            JsonVision(args.pieces),
            None,
            sweep_kinematics=hardware.kinematics,
        )

        write_status(args, "校验当前视觉批次", 0.35)
        batch = active_supervisor.scan()
        if not 0 <= args.piece_index < len(batch.pieces):
            raise ValueError("piece-index 超出新鲜检测结果范围")
        piece = batch.pieces[args.piece_index]

        place_table_mm = _place_target_table_mm(args)
        pick_pose = hardware.pose_for(piece, heights.pick_mm)
        place_pose = hardware.pose_for(
            PieceRef(f"{piece.piece_id}-place", place_table_mm),
            heights.place_mm,
        )

        # 全路径 IK 预检：任何不可达点都必须在第一条运动命令之前拒绝。
        write_status(args, "全路径 IK 预检", 0.42)
        preflight_continuous_sweep(
            hardware.kinematics,
            pick_place_motion_config(),
            (
                hardware.home_pose(),
                hardware.pose_for(piece, SAFE_HEIGHT_MM),
                hardware.pose_for(piece, PREGRASP_HEIGHT_MM),
                pick_pose,
                hardware.pose_for(piece, SAFE_HEIGHT_MM),
                hardware.pose_for(
                    PieceRef(f"{piece.piece_id}-place", place_table_mm), SAFE_HEIGHT_MM
                ),
                hardware.pose_for(
                    PieceRef(f"{piece.piece_id}-place", place_table_mm),
                    PREGRASP_HEIGHT_MM,
                ),
                place_pose,
                hardware.pose_for(
                    PieceRef(f"{piece.piece_id}-place", place_table_mm), SAFE_HEIGHT_MM
                ),
                hardware.home_pose(),
            ),
        )

        park_to_home = hardware.park_mover.park_to_home()
        if not park_to_home.success:
            raise SupervisorFailure(
                SupervisorErrorCode.PARK_TRANSITION_FAILED,
                f"PARK→HOME 失败: {park_to_home.message}",
            )

        write_status(args, f"执行 {piece.piece_id} 磁吸抓放", 0.55)
        active_supervisor.grasp_fsm(
            PlacementPlanRef(batch.source_image_sha256, batch.monotonic_ns),
            piece=piece,
            pick_pose=pick_pose,
            place_pose=place_pose,
        )

        write_status(args, "回正式 PARK", 0.9)
        result = active_supervisor.result(
            True,
            SupervisorErrorCode.NONE,
            f"{piece.piece_id} 抓放完成（无落位传感确认）",
        )
        return outcome(
            success=True,
            error_code=result.error_code.value,
            message=result.message,
            mode=args.mode,
            route=[piece.piece_id],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state=result.final_state.value,
            events_path=result.events_path,
        )
    except BaseException as caught:
        error = caught
        raise
    finally:
        if magnet is not None:
            with contextlib.suppress(BaseException):
                magnet.emergency_off()
        if hardware.robot is not None:
            try:
                parked, message = hardware.attempt_park()
                if not parked:
                    park_failure = message
            except BaseException as park_error:
                park_failure = f"回正式 PARK 时异常: {park_error}"
            finally:
                hardware.disconnect()
        if magnet is not None:
            with contextlib.suppress(BaseException):
                magnet.close()
        if park_failure:
            detail = f"异常清理回正式 PARK 失败: {park_failure}"
            if error is not None:
                raise RuntimeError(f"{error}; {detail}") from error
            raise RuntimeError(detail)


def _place_target_table_mm(args: argparse.Namespace) -> tuple[float, float]:
    """PLACE 目标必须显式给出；本模式不从求解器推断落位。"""
    if args.place_table_mm is None:
        raise ValueError("pick-place 需要 --place-table-mm X,Y 明确指定落位点")
    parts = str(args.place_table_mm).split(",")
    if len(parts) != 2:
        raise ValueError("--place-table-mm 必须是 X,Y 形式")
    x = _finite_number(parts[0].strip(), "--place-table-mm X")
    y = _finite_number(parts[1].strip(), "--place-table-mm Y")
    # A4 下半区（y < 148.5）是官方要求的目标拼合区。
    if not 0.0 <= x <= 210.0 or not 0.0 <= y <= 297.0:
        raise ValueError("--place-table-mm 超出 A4 table_mm 范围")
    return (x, y)


def main() -> int:
    args = build_parser().parse_args()
    requested_mode = args.mode
    args.command = CANONICAL_COMMAND[requested_mode]
    args.mode = COMMAND_ALIASES[requested_mode]
    started_at = time.monotonic()
    started_at_ns = time.monotonic_ns()
    log_path = WORKSPACE / "logs/supervisor" / f"{args.command}.jsonl"
    write_status(args, "准备", 0.0)
    try:
        if args.mode in {
            "competition-self-color",
            "competition-white",
            "competition-card",
        }:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED,
                "比赛全流程后端尚未接入，不能一键启动",
            )
        if args.mode == "pick-place":
            payload = run_pick_place(args, started_at)
        elif args.mode in {
            "manual-read-state",
            "manual-jog-joint",
            "manual-jog-cartesian",
            "commission-center",
        }:
            payload = run_manual_command(args, started_at)
        elif args.mode == "commission-center":
            payload = run_commission_center(args, started_at)
        elif args.mode == "manual-magnet-pulse":
            payload = run_magnet_pulse(args, started_at)
        elif args.mode == "manual-magnet-off":
            payload = run_magnet_off(args, started_at)
        elif args.mode == "shadow-plan":
            write_status(args, "读取当前观察并执行 shadow 求解", 0.35)
            report = Supervisor(
                config(shadow=True, log_path=log_path),
                None,
                None,
                None,
                JsonVision(args.pieces),
                GeometryShadowSolver(args.pieces),
            ).shadow_plan()
            shadow_error = report.error_code
            shadow_success = shadow_error is None
            shadow_error_code = (
                SupervisorErrorCode.NONE.value
                if shadow_error is None
                else shadow_error.value
            )
            payload = outcome(
                success=shadow_success,
                error_code=shadow_error_code,
                message=f"Shadow 分析完成：{report.solve_status}；结果不会下发机械臂",
                mode=args.mode,
                route=[],
                duration_ms=int((time.monotonic() - started_at) * 1000),
                final_state="COMPLETE" if shadow_success else "FAILED",
                events_path=str(log_path),
            )
            payload["report"] = report.as_mapping()
        elif args.mode == "scan":
            write_status(args, "刷新相机检测", 0.25)
            refresh_detection(args)
            batch = JsonVision(args.pieces).scan()
            payload = outcome(
                success=True,
                error_code=SupervisorErrorCode.NONE.value,
                message=f"新鲜相机检测完成：{len(batch.pieces)} 个碎片",
                mode=args.mode,
                route=[piece.piece_id for piece in batch.pieces],
                duration_ms=int((time.monotonic() - started_at) * 1000),
                final_state="SCAN",
                events_path=str(log_path),
            )
        else:
            payload = run_hardware(args, started_at)
        exit_code = 0 if payload["success"] else 2
    except SupervisorFailure as error:
        payload = outcome(
            success=False,
            error_code=error.code.value,
            message=str(error),
            mode=args.mode,
            route=[],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state="FAILED",
            events_path=str(log_path),
        )
        exit_code = 2
    except BaseException as error:
        payload = outcome(
            success=False,
            error_code=classify_unhandled_error(
                error, mode=args.mode, robot_port=args.port
            ).value,
            message=str(error),
            mode=args.mode,
            route=[],
            duration_ms=int((time.monotonic() - started_at) * 1000),
            final_state="FAILED",
            events_path=str(log_path),
        )
        exit_code = 1
    if args.output:
        _atomic_write_json(args.output, payload)
    result_payload = gui_result(
        payload, command=args.command, started_at_ns=started_at_ns
    )
    if args.result_file:
        _atomic_write_json(args.result_file, result_payload)
    write_status(
        args,
        "完成" if exit_code == 0 else "失败",
        1.0,
    )
    print(json.dumps(payload, ensure_ascii=False))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
