"""确定性拼图几何求解公共接口。"""

from .solver import (
    PlacementPlan,
    Pose2,
    PuzzleOptions,
    PuzzleSolveResult,
    PuzzleSolveStatus,
    TargetRegion,
    TextureEvidence,
    WorkspaceState,
    solve_puzzle,
)

__all__ = [
    "PlacementPlan",
    "Pose2",
    "PuzzleOptions",
    "PuzzleSolveResult",
    "PuzzleSolveStatus",
    "TargetRegion",
    "TextureEvidence",
    "WorkspaceState",
    "solve_puzzle",
]
