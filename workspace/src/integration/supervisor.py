# pyright: reportMissingImports=false

"""Fail-closed production integration Supervisor.

This module owns runtime state changes.  It composes the frozen Cartesian control
layer but does not replace its IK, trajectory, GPIO, or five-axis contracts.
"""

from __future__ import annotations

import hashlib
import json
import time
import uuid
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol

from control.kinematics import ToolPose
from control.motion_controller import MotionConfig, MotionController, MotionGoal

from .event_log import JsonlEventLog
from .park_sequence import JointSpaceParkMover, ParkMoveErrorCode
from .sweep_planner import (
    PieceRef,
    SweepKinematics,
    plan_sweep_order,
    preflight_continuous_sweep,
)


class SupervisorState(str, Enum):
    PARKED = "PARKED"
    INIT = "INIT"
    PARK_TO_HOME = "PARK_TO_HOME"
    HOME_TO_PARK = "HOME_TO_PARK"
    WAIT_START = "WAIT_START"
    SCAN = "SCAN"
    SHADOW_PLAN = "SHADOW_PLAN"
    SOLVE = "SOLVE"
    PLAN_PREGRASP = "PLAN_PREGRASP"
    MOVE_PREGRASP = "MOVE_PREGRASP"
    LOCAL_GRASP_ALIGN = "LOCAL_GRASP_ALIGN"
    MAGNET_ON = "MAGNET_ON"
    VERIFY_PICK = "VERIFY_PICK"
    MOVE_PREPLACE = "MOVE_PREPLACE"
    LOCAL_PLACE_ALIGN = "LOCAL_PLACE_ALIGN"
    RELEASE = "RELEASE"
    VERIFY_PLACE = "VERIFY_PLACE"
    NEXT_PIECE = "NEXT_PIECE"
    COMPLETE = "COMPLETE"
    ESTOP = "ESTOP"
    RECOVERY = "RECOVERY"
    FAILED = "FAILED"


class SupervisorErrorCode(str, Enum):
    NONE = "NONE"
    SUPERVISOR_LOCKED = "SUPERVISOR_LOCKED"
    PARK_MISMATCH = "PARK_MISMATCH"
    PARK_TRANSITION_FAILED = "PARK_TRANSITION_FAILED"
    VISION_STALE = "VISION_STALE"
    VISION_NO_PIECE = "VISION_NO_PIECE"
    VISION_LOW_CONFIDENCE = "VISION_LOW_CONFIDENCE"
    PLAN_STALE = "PLAN_STALE"
    SOLVER_NO_SOLUTION = "SOLVER_NO_SOLUTION"
    SOLVER_AMBIGUOUS = "SOLVER_AMBIGUOUS"
    SOLVER_TIMEOUT = "SOLVER_TIMEOUT"
    TRANSFORM_UNOFFICIAL = "TRANSFORM_UNOFFICIAL"
    CALIBRATION_REJECTED = "CALIBRATION_REJECTED"
    ESTOP_ACTIVE = "ESTOP_ACTIVE"
    GLOBAL_TIMEOUT = "GLOBAL_TIMEOUT"
    SAFETY_REJECTED = "SAFETY_REJECTED"
    IK_NO_SOLUTION = "IK_NO_SOLUTION"
    ROBOT_COMM_TIMEOUT = "ROBOT_COMM_TIMEOUT"
    SHADOW_MODE = "SHADOW_MODE"
    PICK_VERIFY_FAILED = "PICK_VERIFY_FAILED"
    RELEASE_VERIFY_FAILED = "RELEASE_VERIFY_FAILED"


@dataclass(frozen=True)
class SupervisorConfig:
    calibration_path: Path
    park_pose_path: Path
    transform_path: Path
    transform_is_official: bool
    motion_config: MotionConfig
    z_safe_mm: float
    z_pregrasp_mm: float
    z_pick_mm: float | None
    z_place_mm: float | None
    park_tolerance_deg: float
    park_follow_tolerance_deg: float
    max_vision_age_ms: int | None
    global_timeout_s: float
    shadow: bool
    automatic_pick_place_enabled: bool
    precision_radial_compensation_mm: float
    precision_pitch_command_deg: float
    event_log_path: Path
    magnet_footprint_clearance_frozen: bool = False
    magnet_on_settle_s: float | None = None
    magnet_off_settle_s: float | None = None


@dataclass(frozen=True)
class VisionBatch:
    batch_id: str
    source_image_sha256: str
    monotonic_ns: int
    pieces: tuple[PieceRef, ...]
    homography_sha256: str = ""


@dataclass(frozen=True)
class PlacementPlanRef:
    source_image_sha256: str
    monotonic_ns: int


@dataclass(frozen=True)
class ShadowPlanReport:
    solve_status: str
    placements: tuple[Mapping[str, object], ...]
    scores: Mapping[str, object]
    solver_version: str
    vision_batch_ref: str
    error_code: SupervisorErrorCode | None = None

    def as_mapping(self) -> dict[str, object]:
        return {
            **asdict(self),
            "error_code": self.error_code.value if self.error_code else None,
        }


@dataclass(frozen=True)
class SupervisorResult:
    schema_version: int
    run_id: str
    final_state: SupervisorState
    success: bool
    error_code: SupervisorErrorCode
    message: str
    events_path: str
    vision_batch_ref: str | None
    plan_ref: str | None
    duration_ms: int

    def as_mapping(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "final_state": self.final_state.value,
            "success": self.success,
            "error_code": self.error_code.value,
            "message": self.message,
            "events_path": self.events_path,
            "vision_batch_ref": self.vision_batch_ref,
            "plan_ref": self.plan_ref,
            "duration_ms": self.duration_ms,
        }


class VisionProvider(Protocol):
    def scan(self) -> VisionBatch: ...


class PuzzleSolver(Protocol):
    def solve(self, batch: VisionBatch) -> ShadowPlanReport: ...


class Magnet(Protocol):
    def set_enabled(self, enabled: bool) -> object: ...

    def emergency_off(self) -> object: ...


class SupervisorFailure(RuntimeError):
    def __init__(self, code: SupervisorErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code


class Supervisor:
    """The only integration component allowed to change global run state."""

    _GRASP_STATES = {
        SupervisorState.LOCAL_GRASP_ALIGN,
        SupervisorState.MAGNET_ON,
        SupervisorState.VERIFY_PICK,
        SupervisorState.MOVE_PREPLACE,
        SupervisorState.LOCAL_PLACE_ALIGN,
        SupervisorState.RELEASE,
        SupervisorState.VERIFY_PLACE,
    }

    def __init__(
        self,
        config: SupervisorConfig,
        motion: MotionController | None,
        park_mover: JointSpaceParkMover | None,
        magnet: Magnet | None,
        vision: VisionProvider,
        solver: PuzzleSolver | None,
        event_log: JsonlEventLog | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
        sweep_kinematics: SweepKinematics | None = None,
    ) -> None:
        if config.shadow and (
            motion is not None or magnet is not None or park_mover is not None
        ):
            raise ValueError("shadow Supervisor 必须使用无硬件依赖的 Stub/None")
        self.config = config
        self._motion = motion
        self._park_mover = park_mover
        self._magnet = magnet
        self._vision = vision
        self._solver = solver
        self._event_log = event_log or JsonlEventLog(
            config.event_log_path, monotonic_ns
        )
        self._monotonic = monotonic
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._sweep_kinematics = sweep_kinematics
        self._state = SupervisorState.PARKED
        self._estop_latched = False
        self._started_at = monotonic()
        self._run_id = uuid.uuid4().hex
        self._batch: VisionBatch | None = None

    @property
    def state(self) -> SupervisorState:
        return self._state

    def _transition(self, target: SupervisorState) -> None:
        if self._estop_latched and target is not SupervisorState.ESTOP:
            raise SupervisorFailure(SupervisorErrorCode.ESTOP_ACTIVE, "急停锁定中")
        if target in self._GRASP_STATES:
            self._grasp_gates()
        old = self._state
        self._state = target
        self._event_log.record(
            state=target.value, event=f"transition:{old.value}->{target.value}"
        )

    def emergency_stop(self) -> None:
        self._estop_latched = True
        self._state = SupervisorState.ESTOP
        self._event_log.record(
            state=self._state.value,
            event="emergency_stop",
            severity="FATAL",
            error_code=SupervisorErrorCode.ESTOP_ACTIVE.value,
        )
        if self._magnet is not None:
            self._magnet.emergency_off()

    def reset_to_parked(self, *, operator_confirmed: bool) -> bool:
        if not self._estop_latched or not operator_confirmed:
            return False
        if self._park_mover is None or not self._park_mover.is_at_park():
            return False
        self._estop_latched = False
        self._transition(SupervisorState.PARKED)
        return True

    def scan(self) -> VisionBatch:
        self._ensure_deadline()
        self._transition(SupervisorState.SCAN)
        batch = self._vision.scan()
        if not batch.pieces:
            raise SupervisorFailure(
                SupervisorErrorCode.VISION_NO_PIECE, "视觉未观察到碎片"
            )
        self._validate_batch(batch)
        self._batch = batch
        self._event_log.record(
            state=self.state.value,
            event="vision_batch",
            payload={
                "batch_id": batch.batch_id,
                "source_image_sha256": batch.source_image_sha256,
                "piece_count": len(batch.pieces),
            },
        )
        return batch

    def _validate_batch(self, batch: VisionBatch) -> None:
        if self.config.max_vision_age_ms is None:
            if self.config.shadow:
                return
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "视觉时效阈值 UNKNOWN"
            )
        age_ns = self._monotonic_ns() - batch.monotonic_ns
        if age_ns < 0 or age_ns > self.config.max_vision_age_ms * 1_000_000:
            raise SupervisorFailure(SupervisorErrorCode.VISION_STALE, "视觉结果已过期")

    def validate_plan(self, plan: PlacementPlanRef) -> None:
        if self._batch is None:
            raise SupervisorFailure(SupervisorErrorCode.PLAN_STALE, "没有绑定视觉批次")
        self._validate_batch(self._batch)
        if plan.source_image_sha256 != self._batch.source_image_sha256:
            raise SupervisorFailure(
                SupervisorErrorCode.PLAN_STALE, "计划未绑定当前图像"
            )
        if plan.monotonic_ns != self._batch.monotonic_ns:
            raise SupervisorFailure(
                SupervisorErrorCode.PLAN_STALE, "计划时间戳未绑定当前视觉"
            )

    def _grasp_gates(self) -> None:
        if self.config.shadow:
            raise SupervisorFailure(
                SupervisorErrorCode.SHADOW_MODE, "shadow 模式禁止硬件动作"
            )
        if self.config.z_pick_mm is None or self.config.z_place_mm is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "PICK/PLACE 接触高度 UNKNOWN"
            )
        if not self.config.automatic_pick_place_enabled:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "自动抓放未冻结启用"
            )
        if not self.config.transform_is_official:
            raise SupervisorFailure(
                SupervisorErrorCode.TRANSFORM_UNOFFICIAL, "table_mm 变换未正式冻结"
            )
        if not self.config.magnet_footprint_clearance_frozen:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "磁头净空 UNKNOWN"
            )
        if self._batch is None:
            raise SupervisorFailure(
                SupervisorErrorCode.VISION_STALE, "没有当前视觉批次"
            )
        self._validate_batch(self._batch)

    def grasp_fsm(
        self,
        plan: PlacementPlanRef,
        *,
        piece: PieceRef,
        pick_pose: ToolPose,
        place_pose: ToolPose,
    ) -> None:
        """磁吸 PICK/PLACE 状态机。

        入口先跑公共 fail-closed 门禁：接触高度 UNKNOWN、自动抓放未启用、
        变换未冻结或磁头净空未冻结时，都不可能到达磁铁命令。

        当前无抱起/落下传感器：VERIFY_PICK / VERIFY_PLACE 只做提升运动和
        读回记录，不伪造吸附或释放成功的传感确认。该局限必须写入事件日志。
        """
        self.validate_plan(plan)
        z_pick = self.config.z_pick_mm
        z_place = self.config.z_place_mm
        if z_pick is None or z_place is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "PICK/PLACE 接触高度 UNKNOWN"
            )
        if self._magnet is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SAFETY_REJECTED, "自动抓放缺少磁铁控制器"
            )
        if self._motion is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SAFETY_REJECTED, "自动抓放缺少运动控制器"
            )
        # pick_pose/place_pose 在 robot_base_mm；z_pick_mm/z_place_mm 在
        # calibration_sheet_mm。两者跨坐标系，不得直接比较。高度正确性由
        # 调用方 pose_for(piece, heights.pick_mm) 保证，并由全路径 IK 预检兑现。

        pick_pregrasp = self._at_height(pick_pose, self.config.z_pregrasp_mm)
        pick_safe = self._at_height(pick_pose, self.config.z_safe_mm)
        place_preplace = self._at_height(place_pose, self.config.z_pregrasp_mm)
        place_safe = self._at_height(place_pose, self.config.z_safe_mm)

        magnet_engaged = False
        try:
            self._transition(SupervisorState.LOCAL_GRASP_ALIGN)
            self._move(pick_safe, "SAFE", "EMPTY_NORMAL", piece.piece_id)
            self._move(pick_pregrasp, "PREGRASP", "EMPTY_SLOW", piece.piece_id)
            self._move(pick_pose, "PICK", "EMPTY_SLOW", piece.piece_id)

            self._transition(SupervisorState.MAGNET_ON)
            self._magnet.set_enabled(True)
            magnet_engaged = True
            self._event_log.record(
                state=self.state.value, event="magnet:on", piece_id=piece.piece_id
            )
            self._settle(self.config.magnet_on_settle_s)

            self._transition(SupervisorState.VERIFY_PICK)
            self._move(pick_safe, "SAFE", "HOLDING_SLOW", piece.piece_id)
            self._event_log.record(
                state=self.state.value,
                event="verify_pick_unsensed",
                piece_id=piece.piece_id,
                payload={"sensor_verified": False},
            )

            self._transition(SupervisorState.MOVE_PREPLACE)
            # 两步转移，避免飞行途中旋转撞到机械臂架子：
            # 1) 保持源 yaw 平移到目标正上方（z_safe）；
            # 2) 在目标正上方原地旋转到目标 yaw，再下降接触。
            # 大 Δyaw_cmd（如 -227°）若在直线飞行中线性旋转，中途可能
            # 扫过架子/障碍物；拆开后旋转只发生在安全高度目标上方。
            # 注意：pick_pose/place_pose 的 yaw 已由调用方（run_assembly）
            # 按标准视角（俯视 A4）修正为 Δyaw_cmd = 2*Δpan − Δyaw，
            # 此处只做插值执行，不再重算方向。
            place_safe_src_yaw = ToolPose(
                place_safe.frame,
                place_safe.position_mm,
                place_safe.pitch_deg,
                pick_pose.yaw_deg,
            )
            self._move(place_safe_src_yaw, "SAFE", "HOLDING_SLOW", piece.piece_id)
            self._move(place_safe, "SAFE", "HOLDING_SLOW", piece.piece_id)

            self._transition(SupervisorState.LOCAL_PLACE_ALIGN)
            self._move(place_preplace, "PREPLACE", "HOLDING_SLOW", piece.piece_id)
            self._move(place_pose, "PLACE", "HOLDING_SLOW", piece.piece_id)

            self._transition(SupervisorState.RELEASE)
            self._magnet.set_enabled(False)
            magnet_engaged = False
            self._event_log.record(
                state=self.state.value, event="magnet:off", piece_id=piece.piece_id
            )
            self._settle(self.config.magnet_off_settle_s)

            self._transition(SupervisorState.VERIFY_PLACE)
            self._move(place_safe, "SAFE", "EMPTY_SLOW", piece.piece_id)
            self._event_log.record(
                state=self.state.value,
                event="verify_place_unsensed",
                piece_id=piece.piece_id,
                payload={"sensor_verified": False},
            )
        finally:
            if magnet_engaged:
                self._magnet.emergency_off()
                self._event_log.record(
                    state=self.state.value,
                    event="magnet:emergency_off",
                    piece_id=piece.piece_id,
                    severity="ERROR",
                )

    @staticmethod
    def _at_height(pose: ToolPose, height_mm: float) -> ToolPose:
        x, y, _ = pose.position_mm
        return ToolPose(pose.frame, (x, y, height_mm), pose.pitch_deg, pose.yaw_deg)

    def _settle(self, seconds: float | None) -> None:
        if seconds is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SUPERVISOR_LOCKED, "磁铁安定时间 UNKNOWN"
            )
        if seconds > 0.0:
            self._sleep(seconds)

    def _move(
        self, pose: ToolPose, phase: str, speed_profile: str, piece_id: str
    ) -> None:
        if self._motion is None:
            raise SupervisorFailure(
                SupervisorErrorCode.SAFETY_REJECTED, "运动控制器未连接"
            )
        self._ensure_deadline()
        result = self._motion.move(
            MotionGoal(
                goal_id=f"{phase.lower()}-{piece_id}-{uuid.uuid4().hex[:8]}",
                timestamp_ns=self._monotonic_ns(),
                frame="robot_base_mm",
                position_mm=pose.position_mm,
                pitch_deg=pose.pitch_deg,
                yaw_deg=pose.yaw_deg,
                phase=phase,
                speed_profile=speed_profile,
                deadline_ns=int(
                    (self._started_at + self.config.global_timeout_s) * 1_000_000_000
                ),
            )
        )
        self._event_log.record(
            state=self.state.value,
            event="motion_actual_readback",
            piece_id=piece_id,
            payload=result.as_mapping(),
        )
        if not result.success:
            raise SupervisorFailure(
                SupervisorErrorCode(result.error_code.value), result.message
            )

    def shadow_plan(self) -> ShadowPlanReport:
        if not self.config.shadow:
            raise SupervisorFailure(
                SupervisorErrorCode.SAFETY_REJECTED,
                "非 shadow 配置拒绝 shadow-only API",
            )
        batch = self.scan()
        self._transition(SupervisorState.SHADOW_PLAN)
        report = (
            self._solver.solve(batch)
            if self._solver
            else ShadowPlanReport(
                "UNAVAILABLE",
                (),
                {},
                "none",
                batch.source_image_sha256,
                SupervisorErrorCode.SOLVER_NO_SOLUTION,
            )
        )
        self._transition(SupervisorState.COMPLETE)
        self._event_log.record(
            state=self.state.value, event="shadow_plan", payload=report.as_mapping()
        )
        return report

    def sweep(
        self,
        batch: VisionBatch,
        *,
        start_table_mm: tuple[float, float],
        pose_for: Callable[[PieceRef, float], ToolPose],
        home_pose: ToolPose | None = None,
        low_repetitions: int = 1,
    ) -> tuple[PieceRef, ...]:
        """Run a magnet-off continuous low-height inspection sweep.

        The segment is entirely IK-preflighted before the first motion command;
        only the first point descends and only the final point ascends.
        """
        if self.config.shadow:
            raise SupervisorFailure(
                SupervisorErrorCode.SHADOW_MODE, "shadow 模式禁止 sweep 动作"
            )
        if (
            self._motion is None
            or self._park_mover is None
            or self._sweep_kinematics is None
        ):
            raise SupervisorFailure(
                SupervisorErrorCode.SAFETY_REJECTED, "sweep 缺少已标定的运动依赖"
            )
        if low_repetitions < 1:
            raise ValueError("low_repetitions 必须至少为 1")
        self._validate_batch(batch)
        route = plan_sweep_order(batch.pieces, start_table_mm)
        if not route:
            raise SupervisorFailure(
                SupervisorErrorCode.VISION_NO_PIECE, "视觉未观察到碎片"
            )
        high_poses = [pose_for(piece, self.config.z_safe_mm) for piece in route]
        low_poses = [pose_for(piece, self.config.z_pregrasp_mm) for piece in route]
        repeated_low_poses = tuple(
            pose for pose in low_poses for _ in range(low_repetitions)
        )
        segment = (high_poses[0], *repeated_low_poses, high_poses[-1])
        full_path = segment if home_pose is None else (home_pose, *segment, home_pose)
        preflight_continuous_sweep(
            self._sweep_kinematics, self.config.motion_config, full_path
        )
        if self._magnet is not None:
            self._magnet.set_enabled(False)
            self._event_log.record(state=self.state.value, event="magnet:off")
        self._transition(SupervisorState.PARK_TO_HOME)
        moved = self._park_mover.park_to_home()
        if not moved.success:
            self._raise_park(moved.error_code, moved.message)
        try:
            self._transition(SupervisorState.PLAN_PREGRASP)
            execution_poses = (
                (*segment, home_pose) if home_pose is not None else segment
            )
            for index, pose in enumerate(execution_poses):
                self._ensure_deadline()
                self._transition(SupervisorState.MOVE_PREGRASP)
                result = self._motion.move(
                    MotionGoal(
                        goal_id=f"sweep-{index}",
                        timestamp_ns=self._monotonic_ns(),
                        frame="robot_base_mm",
                        position_mm=pose.position_mm,
                        pitch_deg=pose.pitch_deg,
                        yaw_deg=pose.yaw_deg,
                        phase="PREGRASP",
                        speed_profile="EMPTY_NORMAL",
                        deadline_ns=int(
                            (self._started_at + self.config.global_timeout_s)
                            * 1_000_000_000
                        ),
                    )
                )
                self._event_log.record(
                    state=self.state.value,
                    event="motion_actual_readback",
                    payload=result.as_mapping(),
                )
                if 1 <= index <= len(repeated_low_poses):
                    self._event_log.record(
                        state=self.state.value,
                        event="sweep_center_actual_readback",
                        piece_id=route[(index - 1) // low_repetitions].piece_id,
                        payload=result.as_mapping(),
                    )
                if not result.success:
                    raise SupervisorFailure(
                        SupervisorErrorCode(result.error_code.value), result.message
                    )
            self._transition(SupervisorState.COMPLETE)
            return route
        finally:
            self._transition(SupervisorState.HOME_TO_PARK)
            returned = self._park_mover.home_to_park()
            if not returned.success:
                self._raise_park(returned.error_code, returned.message)
            self._transition(SupervisorState.PARKED)

    def result(
        self, success: bool, error_code: SupervisorErrorCode, message: str
    ) -> SupervisorResult:
        duration_ms = int((self._monotonic() - self._started_at) * 1000)
        return SupervisorResult(
            1,
            self._run_id,
            self.state,
            success,
            error_code,
            message,
            str(self._event_log.path),
            self._batch.source_image_sha256 if self._batch else None,
            None,
            duration_ms,
        )

    def _raise_park(self, code: ParkMoveErrorCode | None, message: str) -> None:
        mapped = (
            SupervisorErrorCode.PARK_MISMATCH
            if code is ParkMoveErrorCode.PARK_MISMATCH
            else SupervisorErrorCode.PARK_TRANSITION_FAILED
        )
        raise SupervisorFailure(mapped, message)

    def _ensure_deadline(self) -> None:
        if self._monotonic() - self._started_at > self.config.global_timeout_s:
            self._transition(SupervisorState.RECOVERY)
            raise SupervisorFailure(
                SupervisorErrorCode.GLOBAL_TIMEOUT, "Supervisor 全局超时"
            )


def hash_json(value: Mapping[str, object]) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
