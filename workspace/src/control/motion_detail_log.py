"""详细运动日志：记录从 MotionGoal 到每个 waypoint 收敛的完整链路。

用于分析运动细节与尺寸偏差来源：
- 目标位姿（robot_base_mm 系）
- 规划结果（步数、关节跨度）
- 每个插值 waypoint 的目标关节
- 每次读回的实际关节 + 与目标误差（逐关节）
- 收敛过程（几次读回、误差收敛轨迹）
- 最终结果（误差、耗时）

写入 JSONL：{motion_detail_dir}/motion-YYYYMMDD-HHMMSS.jsonl
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from .kinematics import JointVector, ToolPose


class MotionDetailLogger:
    """MotionController 的详细运动日志。"""

    def __init__(self, log_dir: Path | None = None) -> None:
        self._enabled = log_dir is not None
        if not self._enabled:
            return
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._path = log_dir / f"motion-{stamp}.jsonl"
        self._fd = open(self._path, "w", encoding="utf-8")

    def close(self) -> None:
        if self._enabled and self._fd is not None:
            self._fd.close()
            self._fd = None

    @property
    def path(self) -> Path | None:
        return self._path if self._enabled else None

    def _write(self, event: str, **payload: object) -> None:
        if not self._enabled or self._fd is None:
            return
        rec = {
            "t_mono_s": round(time.monotonic(), 6),
            "event": event,
            **payload,
        }
        self._fd.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self._fd.flush()

    # ---------- 阶段记录 ----------

    def goal(self, goal_id: str, pose: ToolPose, phase: str, speed_profile: str) -> None:
        self._write(
            "goal",
            goal_id=goal_id,
            phase=phase,
            speed_profile=speed_profile,
            position_mm=[round(v, 3) for v in pose.position_mm],
            pitch_deg=round(pose.pitch_deg, 3),
            yaw_deg=round(pose.yaw_deg, 3),
        )

    def plan(
        self,
        goal_id: str,
        steps: int,
        joint_span_deg: float,
        start_joints: dict[str, float],
        target_joints: dict[str, float],
    ) -> None:
        self._write(
            "plan",
            goal_id=goal_id,
            steps=steps,
            joint_span_deg=round(float(joint_span_deg), 3),
            start_joints={k: round(v, 3) for k, v in start_joints.items()},
            target_joints={k: round(v, 3) for k, v in target_joints.items()},
        )

    def waypoint_sent(
        self, goal_id: str, index: int, total: int, target: dict[str, float]
    ) -> None:
        self._write(
            "waypoint_sent",
            goal_id=goal_id,
            index=index,
            total=total,
            target_joints={k: round(v, 3) for k, v in target.items()},
        )

    def readback(
        self,
        goal_id: str,
        index: int,
        actual: dict[str, float],
        error_deg: dict[str, float],
        max_error: float,
        tolerance_deg: float,
    ) -> None:
        self._write(
            "readback",
            goal_id=goal_id,
            index=index,
            actual_joints={k: round(v, 3) for k, v in actual.items()},
            error_deg={k: round(v, 3) for k, v in error_deg.items()},
            max_error=round(max_error, 3),
            tolerance_deg=tolerance_deg,
        )

    def waypoint_converged(
        self, goal_id: str, index: int, reads: int, final_error: float
    ) -> None:
        self._write(
            "waypoint_converged",
            goal_id=goal_id,
            index=index,
            reads=reads,
            final_error=round(final_error, 3),
        )

    def waypoint_timeout(self, goal_id: str, index: int, error: float) -> None:
        self._write("waypoint_timeout", goal_id=goal_id, index=index, error=round(error, 3))

    def result(
        self,
        goal_id: str,
        success: bool,
        error_code: str,
        executed: int,
        planned: int,
        duration_s: float,
        final_joints: dict[str, float],
    ) -> None:
        self._write(
            "result",
            goal_id=goal_id,
            success=success,
            error_code=error_code,
            executed=executed,
            planned=planned,
            duration_s=round(duration_s, 3),
            final_joints={k: round(v, 3) for k, v in final_joints.items()},
        )
