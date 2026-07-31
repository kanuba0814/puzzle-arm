# pyright: reportMissingImports=false

from dataclasses import dataclass
import unittest

import numpy as np

from puzzle import PuzzleOptions
from puzzle.solver import describe_edges, edge_candidates, se2_align


@dataclass(frozen=True)
class Piece:
    piece_id: str
    polygon_table_mm: tuple[tuple[float, float], ...]
    center_table_mm: tuple[float, float] = (0.0, 0.0)
    yaw_deg: float = 0.0
    timestamp_ns: int = 1
    equivalent_yaws_deg: tuple[float, ...] = ()


class PuzzleSolverUnitTest(unittest.TestCase):
    def test_se2_is_proper_rotation(self) -> None:
        first = describe_edges(Piece("a", ((0, 0), (90, 0), (90, 50), (0, 50))))[0]
        second = describe_edges(Piece("b", ((0, 0), (90, 0), (90, 50), (0, 50))))[0]
        rotation, _ = se2_align(first, second)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(2), atol=1e-10)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

    def test_internal_partial_edge_includes_non_endpoint_offset(self) -> None:
        long_piece = Piece("long", ((0, 0), (90, 0), (90, 50), (0, 50)))
        short_piece = Piece("short", ((0, 0), (20, 0), (20, 30), (0, 30)))
        candidates = edge_candidates(
            long_piece, short_piece, PuzzleOptions(partial_offset_step_mm=1.0)
        )
        offsets = {
            candidate.offset_mm
            for candidate in candidates
            if candidate.kind == "internal"
        }
        self.assertIn(37.0, offsets)


if __name__ == "__main__":
    unittest.main()
