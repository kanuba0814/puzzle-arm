import inspect
import unittest

from fixtures.puzzle_fixture_generator import FAMILIES, generate_fixture
from puzzle import PuzzleOptions, TargetRegion, WorkspaceState, solve_puzzle


class PuzzleFixtureGeneratorTest(unittest.TestCase):
    def test_all_original_families_are_self_validating_and_seed_deterministic(self):
        for family in FAMILIES:
            self.assertEqual(generate_fixture(family, 17), generate_fixture(family, 17))

    def test_production_solver_signature_has_no_fixture_truth_argument(self):
        self.assertNotIn("ground_truth", inspect.signature(solve_puzzle).parameters)

    def test_representative_rectangle_corpus_solves_deterministically(self):
        options = PuzzleOptions(ambiguity_score_gap=0.0, max_expansions=2000)
        target = TargetRegion("table_mm", 0, 0, 210, 148)
        # Fixed, representative corpus: one each of 1/2/3/4-piece rectangular coverage.
        for family in ("rectangle-1", "stripes-2", "stripes-3", "stripes-4"):
            fixture = generate_fixture(family, 23)
            first = solve_puzzle(WorkspaceState(fixture.pieces), target, options)
            second = solve_puzzle(WorkspaceState(fixture.pieces), target, options)
            self.assertEqual(first.status.value, "SOLVED", family)
            self.assertEqual(first.placements, second.placements, family)


if __name__ == "__main__":
    unittest.main()
