# pyright: reportMissingImports=false

import unittest
from dataclasses import dataclass

from puzzle import PuzzleOptions, TargetRegion, WorkspaceState, solve_puzzle


@dataclass(frozen=True)
class Piece:
    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float] = (10.0, 10.0)
    yaw_deg: float = 0.0
    timestamp_ns: int = 3
    equivalent_yaws_deg: tuple[float, ...] = ()


class PuzzleSolverSyntheticTest(unittest.TestCase):
    def test_solved_plan_reports_verified_five_mm_pair_clearance(self) -> None:
        pieces = (
            Piece("a", ((0, 0), (50, 0), (50, 60), (0, 60)), (25, 30)),
            Piece("b", ((50, 0), (100, 0), (100, 60), (50, 60)), (75, 30)),
        )
        workspace = WorkspaceState(pieces)
        target = TargetRegion("table_mm", 0, 0, 210, 148)
        result = solve_puzzle(
            workspace,
            target,
            PuzzleOptions(ambiguity_score_gap=0.0, max_expansions=1000),
        )
        self.assertEqual(result.status.value, "SOLVED")
        self.assertGreaterEqual(
            float(result.diagnostics["extra"]["minimum_pair_clearance_mm"]),
            5.0 - 1e-5,
        )
        self.assertEqual(result.score_breakdown["placement_gap_mm"], 5.0)
        self.assertEqual(
            tuple(plan.source.center_mm for plan in result.placements),
            tuple(piece.center_table_mm for piece in pieces),
        )
        self.assertTrue(all(plan.source.yaw_deg == 0.0 for plan in result.placements))

    def test_gap_below_five_mm_is_rejected_by_options(self) -> None:
        for invalid_gap in (0.0, 4.999, float("nan")):
            with self.subTest(placement_gap_mm=invalid_gap):
                with self.assertRaisesRegex(ValueError, "至少 5 mm"):
                    PuzzleOptions(placement_gap_mm=invalid_gap)

    def test_single_legal_rectangle_is_solved_deterministically(self) -> None:
        piece = Piece("one", ((0, 0), (100, 0), (100, 60), (0, 60)), (20, 20))
        target = TargetRegion("table_mm", 0, 0, 210, 148)
        first = solve_puzzle(WorkspaceState((piece,)), target, PuzzleOptions())
        second = solve_puzzle(WorkspaceState((piece,)), target, PuzzleOptions())
        self.assertEqual(first.status.value, "SOLVED")
        self.assertEqual(first.placements, second.placements)
        # reference point is 30mm left/10mm below polygon bbox center；平移后
        # 多边形 bbox 中心为目标区中心，reference 仍保持同一局部偏移。
        self.assertEqual(first.placements[0].target.center_mm, (75.0, 64.0))
        self.assertEqual(
            first.diagnostics["extra"]["bounds_table_mm"],
            [55.0, 44.0, 155.0, 104.0],
        )

    def test_single_piece_fails_closed_when_target_region_is_too_small(self) -> None:
        piece = Piece("one", ((0, 0), (100, 0), (100, 60), (0, 60)), (20, 20))
        result = solve_puzzle(
            WorkspaceState((piece,)),
            TargetRegion("table_mm", 0, 0, 50, 50),
            PuzzleOptions(),
        )

        self.assertEqual(result.status.value, "NO_SOLUTION")
        self.assertEqual(result.error_code, "SOLVER_DISCRETE_LAYOUT_FAILED")
        self.assertEqual(result.placements, ())

    def test_budget_exhaustion_returns_no_partial_plan(self) -> None:
        pieces = (
            Piece("a", ((0, 0), (50, 0), (50, 60), (0, 60))),
            Piece("b", ((0, 0), (50, 0), (50, 60), (0, 60))),
        )
        result = solve_puzzle(
            WorkspaceState(pieces),
            TargetRegion("table_mm", 0, 0, 210, 148),
            PuzzleOptions(max_expansions=1),
        )
        self.assertEqual(result.error_code, "SOLVER_TIMEOUT")
        self.assertEqual(result.placements, ())

    def test_t_offset_fixture_solves_with_shared_long_edge_intervals(self) -> None:
        from fixtures.puzzle_fixture_generator import generate_fixture

        fixture = generate_fixture("t-offset-3", 7)
        result = solve_puzzle(
            WorkspaceState(fixture.pieces),
            TargetRegion("table_mm", 0, 0, 210, 148),
            PuzzleOptions(ambiguity_score_gap=0.0, max_expansions=1000),
        )
        self.assertEqual(result.status.value, "SOLVED")
        self.assertEqual(len(result.placements), 3)

    def test_overlapping_candidate_cannot_become_a_plan(self) -> None:
        # Two 100×60 pieces cannot form any allowed 90--120×50--90 target
        # without overlap, so no executable plan may be emitted.
        pieces = (
            Piece("a", ((0, 0), (100, 0), (100, 60), (0, 60))),
            Piece("b", ((0, 0), (100, 0), (100, 60), (0, 60))),
        )
        result = solve_puzzle(
            WorkspaceState(pieces),
            TargetRegion("table_mm", 0, 0, 210, 148),
            PuzzleOptions(ambiguity_score_gap=0.0, max_expansions=1000),
        )
        self.assertEqual(result.status.value, "NO_SOLUTION")
        self.assertEqual(result.placements, ())


if __name__ == "__main__":
    unittest.main()
