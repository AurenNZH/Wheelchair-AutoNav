import unittest

import numpy as np

from wheelchair_navigation.costmap import (
    FrontCostmapConfig,
    LocalCostmapConfig,
    front_point_cell_ids,
    minimum_range_rejection_mask,
    obstacle_point_mask,
    validate_mapping_configs,
)


class CostmapGeometryTests(unittest.TestCase):
    def test_obstacle_mask_matches_nav2_height_and_range_window(self):
        points = np.array(
            [
                [0.44, 0.0, 0.4],
                [0.45, 0.0, 0.4],
                [4.0, 0.0, 1.5],
                [4.01, 0.0, 0.4],
                [1.0, 0.0, 0.04],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        accepted, counts = obstacle_point_mask(points, LocalCostmapConfig())

        np.testing.assert_array_equal(
            accepted, [False, True, True, False, False, False]
        )
        self.assertEqual(counts["input_points"], 6)
        self.assertEqual(counts["finite_points"], 5)
        self.assertEqual(counts["height_range_points"], 2)

    def test_front_cell_ids_match_nav2_grid_geometry(self):
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )
        points = np.array(
            [
                [0.0, -1.0, 0.4],
                [0.49, -0.51, 0.4],
                [1.99, 0.99, 0.4],
                [-0.01, 0.0, 0.4],
                [2.0, 0.0, 0.4],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        valid, cell_ids, cell_count = front_point_cell_ids(points, config)

        np.testing.assert_array_equal(
            valid, [True, True, True, False, False, False]
        )
        np.testing.assert_array_equal(cell_ids[:3], [0, 0, 15])
        self.assertEqual(cell_count, 16)

    def test_minimum_range_is_a_hard_exclusion_with_inclusive_boundary(self):
        points = np.array(
            [
                [0.44, 0.0, 0.4],
                [0.45, 0.0, 0.4],
                [0.0, -0.46, 0.4],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        rejected = minimum_range_rejection_mask(points, 0.45)

        np.testing.assert_array_equal(rejected, [True, False, False, False])
        with self.assertRaises(ValueError):
            minimum_range_rejection_mask(points, -0.1)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_mapping_configs(
                LocalCostmapConfig(min_range_m=5.0, max_range_m=4.0),
                FrontCostmapConfig(),
            )
        with self.assertRaises(ValueError):
            validate_mapping_configs(
                LocalCostmapConfig(), FrontCostmapConfig(fov_deg=181.0)
            )


if __name__ == "__main__":
    unittest.main()
