#!/usr/bin/env python3
"""123 求解器 shim：让 supervisor_run 使用 solve_puzzle_123。

用法：
    supervisor_run.py shadow-plan --solver-backend 123 ...
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from integration.supervisor import ShadowPlanReport, VisionBatch

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from integration._adapter_123 import solve_puzzle_123  # noqa: E402
from integration.supervisor import SupervisorErrorCode  # noqa: E402
from puzzle.solver import (  # noqa: E402
    PuzzleOptions,
    PuzzleSolveStatus,
    TargetRegion,
    WorkspaceState,
)


class Legacy123Solver:
    """调用 123 拼装求解器（AssemblySolver）的 shadow solver shim。"""

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
            _JsonPuzzlePiece(item, index)
            for index, item in enumerate(payload["pieces"])
        )
        # 目标区：题目要求 0--210mm, 0--148.5mm（A4 下半区）
        target = TargetRegion("table_mm", 0.0, 0.0, 210.0, 148.5)
        # 注意：123 solver 不用 workspace 的 time_budget/max_expansions，
        # 它有自己的内部超时和近似解兜底。
        options = PuzzleOptions(
            time_budget_ms=10_000,
            max_expansions=20_000,
            ambiguity_score_gap=None,
            partial_offset_step_mm=1.0,
            abs_tolerance_mm=2.0,
            rel_tolerance=0.03,
            vertex_tolerance_mm=2.0,
        )

        result = solve_puzzle_123(WorkspaceState(pieces), target, options)

        error_code = _error_code(result.status, result.error_code)
        return {
            "solve_status": result.status.value,
            "placements": [plan.as_mapping() for plan in result.placements],
            "scores": {
                "best_score": result.best_score,
                "second_best_score": result.second_best_score,
                "score_gap": result.score_gap,
                "candidate_count": result.candidates_after_dedup,
                "inferred_target_mm": result.inferred_target,
                "score_breakdown": result.score_breakdown,
                "expansions": result.expansions,
                "pruned": result.pruned,
            },
            "solver_version": result.solver_version,
            "vision_batch_ref": batch.source_image_sha256,
            "error_code": error_code,
        }


class _JsonPuzzlePiece:
    """满足 solver._PieceLike 协议的 JSON 碎片。"""

    def __init__(self, item: object, index: int) -> None:
        if not isinstance(item, dict):
            raise ValueError(f"pieces[{index}] 必须是对象")

        polygon_value = item.get("polygon_table_mm")
        if not isinstance(polygon_value, list) or not 3 <= len(polygon_value) <= 6:
            raise ValueError(
                f"pieces[{index}].polygon_table_mm 必须是 3--6 点列表（实际 {polygon_value}）"
            )

        self.piece_id: str = str(item.get("piece_id", f"piece_{index:02d}"))
        self.polygon_table_mm: tuple[tuple[float, float], ...] = tuple(
            (float(pt[0]), float(pt[1])) for pt in polygon_value
        )
        center_value = item.get("center_table_mm")
        if center_value is None or len(center_value) != 2:
            raise ValueError(f"pieces[{index}].center_table_mm 必须是 [x, y]")
        self.center_table_mm: tuple[float, float] = (
            float(center_value[0]),
            float(center_value[1]),
        )
        self.yaw_deg: float = float(item.get("yaw_deg", 0.0))
        self.timestamp_ns: int = item.get("timestamp_ns", 0)
        self.equivalent_yaws_deg: tuple[float, ...] = (self.yaw_deg,)


def _error_code(
    status: PuzzleSolveStatus, error: str | None
) -> SupervisorErrorCode | None:
    if status is PuzzleSolveStatus.SOLVED:
        return None
    if status is PuzzleSolveStatus.NO_SOLUTION:
        return SupervisorErrorCode.SOLVER_NO_SOLUTION
    if status is PuzzleSolveStatus.TIMEOUT:
        return SupervisorErrorCode.SOLVER_TIMEOUT
    if status is PuzzleSolveStatus.AMBIGUOUS:
        return None  # AMBIGUOUS 对 123 来说是近似解，仍可用
    return SupervisorErrorCode.SOLVER_FAILED
