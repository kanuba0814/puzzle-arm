# pyright: reportMissingImports=false

import unittest

import numpy as np

from puzzle.discrete_layout import (
    DiscreteLayoutError,
    plan_discrete_layout,
    polygon_distance_mm,
)


TARGET_BOUNDS = (0.0, 0.0, 210.0, 148.0)


def rectangle(min_x: float, min_y: float, max_x: float, max_y: float) -> np.ndarray:
    return np.asarray(
        (
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ),
        dtype=np.float64,
    )


def placed_polygons(
    polygons: tuple[np.ndarray, ...], translations: tuple[tuple[float, float], ...]
) -> tuple[np.ndarray, ...]:
    return tuple(
        polygon + np.asarray(translation)
        for polygon, translation in zip(polygons, translations, strict=True)
    )


class DiscreteLayoutTest(unittest.TestCase):
    def assert_clearance(
        self, first: np.ndarray, second: np.ndarray, expected_mm: float = 5.0
    ) -> None:
        self.assertAlmostEqual(
            polygon_distance_mm(first, second), expected_mm, delta=1e-6
        )

    def test_two_touching_rectangles_get_exact_five_mm_gap_and_stay_centered(self) -> None:
        polygons = (rectangle(0, 0, 50, 60), rectangle(50, 0, 100, 60))

        layout = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )
        placed = placed_polygons(polygons, layout.translations_mm)

        self.assert_clearance(*placed)
        all_points = np.vstack(placed)
        np.testing.assert_allclose(
            (all_points.min(axis=0) + all_points.max(axis=0)) / 2.0,
            np.array((105.0, 74.0)),
            atol=1e-9,
        )
        np.testing.assert_allclose(
            all_points.max(axis=0) - all_points.min(axis=0),
            np.array((105.0, 60.0)),
            atol=1e-9,
        )

    def test_three_collinear_pieces_keep_five_mm_between_both_neighbors(self) -> None:
        polygons = (
            rectangle(0, 0, 30, 60),
            rectangle(30, 0, 60, 60),
            rectangle(60, 0, 90, 60),
        )

        layout = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )
        placed = placed_polygons(polygons, layout.translations_mm)

        self.assert_clearance(placed[0], placed[1])
        self.assert_clearance(placed[1], placed[2])
        self.assertGreaterEqual(polygon_distance_mm(placed[0], placed[2]), 5.0)

    def test_t_layout_has_five_mm_clearance_for_every_pair(self) -> None:
        polygons = (
            rectangle(0, 0, 50, 60),
            rectangle(50, 30, 100, 60),
            rectangle(50, 0, 100, 30),
        )

        layout = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )
        placed = placed_polygons(polygons, layout.translations_mm)

        for first_index in range(len(placed)):
            for second_index in range(first_index + 1, len(placed)):
                self.assertGreaterEqual(
                    polygon_distance_mm(placed[first_index], placed[second_index]),
                    5.0 - 1e-6,
                )
        self.assertAlmostEqual(layout.minimum_clearance_mm or 0.0, 5.0, delta=1e-6)

    def test_non_neighbor_pair_below_gap_is_also_separated(self) -> None:
        polygons = (
            rectangle(0, 0, 20, 20),
            rectangle(20, 0, 40, 20),
            rectangle(43, 0, 63, 20),
        )

        layout = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )
        placed = placed_polygons(polygons, layout.translations_mm)

        self.assertGreaterEqual(polygon_distance_mm(placed[1], placed[2]), 5.0 - 1e-6)

    def test_clearance_below_five_mm_is_rejected(self) -> None:
        polygons = (rectangle(0, 0, 50, 60), rectangle(50, 0, 100, 60))

        for invalid_clearance in (0.0, 4.999, float("nan")):
            with self.subTest(clearance_mm=invalid_clearance):
                with self.assertRaisesRegex(DiscreteLayoutError, "至少 5 mm"):
                    plan_discrete_layout(
                        polygons,
                        clearance_mm=invalid_clearance,
                        target_bounds_mm=TARGET_BOUNDS,
                    )

    def test_layout_is_deterministic(self) -> None:
        polygons = (
            rectangle(0, 0, 50, 60),
            rectangle(50, 30, 100, 60),
            rectangle(50, 0, 100, 30),
        )

        first = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )
        second = plan_discrete_layout(
            polygons, clearance_mm=5.0, target_bounds_mm=TARGET_BOUNDS
        )

        self.assertEqual(first, second)

    def test_layout_fails_closed_when_target_region_is_too_small(self) -> None:
        polygons = (rectangle(0, 0, 50, 60), rectangle(50, 0, 100, 60))

        with self.assertRaisesRegex(DiscreteLayoutError, "超出目标区域"):
            plan_discrete_layout(
                polygons,
                clearance_mm=5.0,
                target_bounds_mm=(0.0, 0.0, 100.0, 60.0),
            )

    def test_polygon_distance_handles_intersection_and_containment(self) -> None:
        outer = rectangle(0, 0, 20, 20)
        overlapping = rectangle(10, 10, 30, 30)
        inner = rectangle(5, 5, 10, 10)

        self.assertEqual(polygon_distance_mm(outer, overlapping), 0.0)
        self.assertEqual(polygon_distance_mm(outer, inner), 0.0)


if __name__ == "__main__":
    unittest.main()
