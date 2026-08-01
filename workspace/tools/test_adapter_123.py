#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""123 适配器端到端验证：合成 guillotine 碎片 → solve_puzzle_123。

从 workspace/src 运行：
    python3 ../tools/test_adapter_123.py
或从项目根：
    PYTHONPATH=workspace/src python3 workspace/tools/test_adapter_123.py
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from integration._adapter_123 import solve_puzzle_123  # noqa: E402
from puzzle._legacy_123_assembly import (  # noqa: E402
    _gen_guillotine_pieces,
    _scatter,
)
from puzzle.discrete_layout import polygon_distance_mm  # noqa: E402
from puzzle.solver import (  # noqa: E402
    PuzzleOptions,
    PuzzleSolveStatus,
    TargetRegion,
    WorkspaceState,
)


@dataclass(frozen=True)
class FakePiece:
    """满足 solver._PieceLike 协议的最小测试替身。"""

    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float]
    yaw_deg: float
    timestamp_ns: int
    equivalent_yaws_deg: tuple[float, ...]
    pick_point_table_mm: tuple[float, float] | None = None


def make_pieces(scattered: list[np.ndarray]) -> tuple[FakePiece, ...]:
    pieces = []
    for index, verts in enumerate(scattered):
        polygon = tuple((float(v[0]), float(v[1])) for v in verts)
        vertex_mean = verts.mean(axis=0)
        # 数据契约不保证 center 等于顶点算术均值；故意制造差异，防止适配器
        # 用 placed_vertex_mean + R@(pick-center) 近似正式 R@pick+t。
        center = vertex_mean + np.asarray((3.0, -2.0))
        pieces.append(
            FakePiece(
                piece_id=f"piece_{index:02d}",
                polygon_table_mm=polygon,
                center_table_mm=(float(center[0]), float(center[1])),
                yaw_deg=0.0,
                timestamp_ns=index,
                equivalent_yaws_deg=(0.0,),
                # 非质心吸点回归：验证 source 保留实测 pick_point，target
                # 对同一局部点应用求解旋转与离散平移。
                pick_point_table_mm=(
                    float(vertex_mean[0] + 1.25),
                    float(vertex_mean[1] - 0.75),
                ),
            )
        )
    return tuple(pieces)


def main() -> int:
    # 与正式 A4 下半区一致；旧 297x105 横条会拒绝竖向 100mm 拼图加间距。
    target = TargetRegion(
        frame="table_mm",
        min_x_mm=0.0,
        min_y_mm=0.0,
        max_x_mm=210.0,
        max_y_mm=148.5,
    )
    options = PuzzleOptions(placement_gap_mm=5.0)

    passed = 0
    total = 6
    for seed in range(total):
        truth_pieces, true_size = _gen_guillotine_pieces(
            W=100, H=60, n_pieces=4, seed=seed
        )
        scattered = _scatter(truth_pieces, seed=seed + 100)
        state = WorkspaceState(pieces=make_pieces(scattered), locked_placements=())

        result = solve_puzzle_123(state, target, options)

        if result.status is PuzzleSolveStatus.NO_SOLUTION:
            print(
                f"seed={seed}: NO_SOLUTION ({result.error_code}) "
                f"diagnostics={result.diagnostics}"
            )
            continue

        assert result.inferred_target is not None
        width, height = result.inferred_target
        match = (
            abs(width - true_size[0]) < 2 and abs(height - true_size[1]) < 2
        ) or (abs(width - true_size[1]) < 2 and abs(height - true_size[0]) < 2)

        centers = np.array([plan.target.center_mm for plan in result.placements])
        inside = bool(
            np.all(centers[:, 0] >= target.min_x_mm)
            and np.all(centers[:, 0] <= target.max_x_mm)
            and np.all(centers[:, 1] >= target.min_y_mm)
            and np.all(centers[:, 1] <= target.max_y_mm)
        )
        # 0° 基准约定：source.yaw 恒为 0（不测量绝对朝向），
        # target.yaw 即求解器相对旋转角。
        zero_baseline = all(
            plan.source.yaw_deg == 0.0 and -180.0 <= plan.target.yaw_deg < 180.0
            for plan in result.placements
        )
        source_by_id = {piece.piece_id: piece for piece in state.pieces}
        grip_semantics = all(
            plan.source.center_mm == source_by_id[plan.piece_id].pick_point_table_mm
            for plan in result.placements
        )
        target_polygons = []
        for plan in result.placements:
            source_piece = source_by_id[plan.piece_id]
            angle_rad = np.radians(plan.target.yaw_deg)
            rotation = np.asarray(
                (
                    (np.cos(angle_rad), -np.sin(angle_rad)),
                    (np.sin(angle_rad), np.cos(angle_rad)),
                )
            )
            target_polygons.append(
                (np.asarray(source_piece.polygon_table_mm) - plan.source.center_mm)
                @ rotation.T
                + plan.target.center_mm
            )
        clearances = [
            polygon_distance_mm(target_polygons[first], target_polygons[second])
            for first in range(len(target_polygons))
            for second in range(first + 1, len(target_polygons))
        ]
        clearance_ok = bool(clearances) and min(clearances) >= 5.0 - 1e-5

        print(
            f"seed={seed}: {result.status.value} "
            f"{width:.1f}x{height:.1f} 真值{true_size[0]:.0f}x{true_size[1]:.0f} "
            f"尺寸{'OK' if match else '不符'} 落位{'在区内' if inside else '越界'} "
            f"0基准{'OK' if zero_baseline else '不符'} "
            f"吸点{'OK' if grip_semantics else '不符'} "
            f"最小净距={min(clearances):.3f}mm "
            f"片数={len(result.placements)} 用时={result.timing_ms['total_ms']:.0f}ms"
        )
        if (
            match
            and inside
            and zero_baseline
            and grip_semantics
            and clearance_ok
            and len(result.placements) == 4
        ):
            passed += 1

    print(f"--- {passed}/{total} 通过")
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
