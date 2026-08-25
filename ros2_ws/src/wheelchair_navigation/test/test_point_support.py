import unittest

import numpy as np

from wheelchair_navigation.costmap import FrontCostmapConfig
from wheelchair_navigation.point_support import filter_points_by_cell_support


class PointSupportTests(unittest.TestCase):
    def setUp(self):
        self.config = FrontCostmapConfig(
            length_m=1.0, width_m=1.0, resolution_m=0.1
        )

    def test_requires_three_points_in_each_eligible_cell(self):
        points = np.array(
            [
                [0.05, 0.05, 0.20],
                [0.06, 0.06, 0.30],
                [0.15, 0.05, 0.20],
                [0.16, 0.06, 0.30],
                [0.17, 0.07, 0.40],
                [-0.10, 0.00, 0.20],
            ],
            dtype=np.float32,
        )

        result = filter_points_by_cell_support(
            points,
            np.array([True, True, True, True, True, False]),
            self.config,
            min_points_per_cell=3,
        )

        np.testing.assert_array_equal(
            result.keep_mask, [False, False, True, True, True, True]
        )
        self.assertEqual(result.stats.occupied_cells, 2)
        self.assertEqual(result.stats.low_support_cells, 1)
        self.assertEqual(result.stats.low_support_points, 2)

    def test_retains_finite_points_outside_nav2_marking_window(self):
        points = np.array(
            [
                [0.05, 0.05, 0.20],
                [3.0, 0.0, 0.20],
                [np.nan, 0.0, 0.20],
            ],
            dtype=np.float32,
        )

        result = filter_points_by_cell_support(
            points,
            np.array([True, False, False]),
            self.config,
            min_points_per_cell=1,
        )

        np.testing.assert_array_equal(result.keep_mask, [True, True, False])

    def test_rejects_invalid_thresholds_and_shapes(self):
        points = np.zeros((2, 3), dtype=np.float32)
        for minimum in (0, 1.5, True):
            with self.subTest(minimum=minimum), self.assertRaises(ValueError):
                filter_points_by_cell_support(
                    points,
                    np.ones(2, dtype=bool),
                    self.config,
                    min_points_per_cell=minimum,
                )
        with self.assertRaises(ValueError):
            filter_points_by_cell_support(
                points,
                np.ones(3, dtype=bool),
                self.config,
                min_points_per_cell=3,
            )


if __name__ == "__main__":
    unittest.main()
