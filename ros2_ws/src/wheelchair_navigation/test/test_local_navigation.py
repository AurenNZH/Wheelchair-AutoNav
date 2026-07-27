import time
import unittest

import numpy as np

from wheelchair_navigation.local_navigation import (
    LocalCostmapConfig,
    SelfFilterBox,
    make_local_costmaps,
    parse_self_filter_boxes,
    world_to_cell,
)
from wheelchair_navigation.local_navigation_node import cloud_timestamp_error


class LocalNavigationTests(unittest.TestCase):
    def test_defaults_publish_raw_obstacle_size_without_inflation(self):
        config = LocalCostmapConfig(size_m=4.0, resolution_m=0.1)
        points = np.array([[1.0, 0.0, 0.4]], dtype=np.float32)

        raw, costmap, stats = make_local_costmaps(points, config)

        self.assertEqual(config.inflation_radius_m, 0.0)
        self.assertEqual(np.count_nonzero(raw == 100), 1)
        np.testing.assert_array_equal(costmap, raw)
        self.assertEqual(stats.occupied_cells, 1)

    def test_range_height_and_nonfinite_returns_are_filtered(self):
        config = LocalCostmapConfig(size_m=12.0, inflation_radius_m=0.0)
        points = np.array(
            [
                [-2.0, 0.0, 0.04],
                [4.1, 0.0, 0.4],
                [1.0, 0.0, 0.4],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        raw, _, stats = make_local_costmaps(points, config)

        obstacle_cell = world_to_cell(1.0, 0.0, config, raw.shape[0])
        self.assertEqual(raw[obstacle_cell[1], obstacle_cell[0]], 100)
        self.assertEqual(np.count_nonzero(raw == 100), 1)
        self.assertEqual(stats.input_points, 4)
        self.assertEqual(stats.finite_points, 3)
        self.assertEqual(stats.height_range_points, 1)

    def test_inflation_is_separate_and_applied_once(self):
        config = LocalCostmapConfig(
            size_m=4.0,
            resolution_m=0.1,
            inflation_radius_m=0.2,
        )
        points = np.array([[1.0, 0.0, 0.4]], dtype=np.float32)

        raw, costmap, _ = make_local_costmaps(points, config)

        self.assertEqual(np.count_nonzero(raw == 100), 1)
        self.assertEqual(np.count_nonzero(costmap == 100), 13)

    def test_measured_self_filter_box_removes_only_points_inside_it(self):
        config = LocalCostmapConfig(size_m=4.0)
        box = SelfFilterBox(0.2, 0.8, -0.4, 0.4, 0.05, 0.8)
        points = np.array(
            [[0.5, 0.0, 0.3], [1.0, 0.0, 0.3]], dtype=np.float32
        )

        raw, _, stats = make_local_costmaps(points, config, (box,))

        masked_cell = world_to_cell(0.5, 0.0, config, raw.shape[0])
        obstacle_cell = world_to_cell(1.0, 0.0, config, raw.shape[0])
        self.assertEqual(raw[masked_cell[1], masked_cell[0]], 0)
        self.assertEqual(raw[obstacle_cell[1], obstacle_cell[0]], 100)
        self.assertEqual(stats.self_filtered_points, 1)

    def test_flat_self_filter_configuration_is_validated_and_padded(self):
        self.assertEqual(parse_self_filter_boxes(None), ())
        boxes = parse_self_filter_boxes(
            [0.2, 0.8, -0.4, 0.4, 0.05, 0.8], padding_m=0.02
        )
        self.assertEqual(len(boxes), 1)
        np.testing.assert_allclose(
            [
                boxes[0].min_x_m,
                boxes[0].max_x_m,
                boxes[0].min_y_m,
                boxes[0].max_y_m,
                boxes[0].min_z_m,
                boxes[0].max_z_m,
            ],
            [0.18, 0.82, -0.42, 0.42, 0.03, 0.82],
        )
        with self.assertRaises(ValueError):
            parse_self_filter_boxes([0.0, 1.0])
        with self.assertRaises(ValueError):
            parse_self_filter_boxes([1.0, 0.0, -1.0, 1.0, 0.0, 1.0])

    def test_vectorized_mapper_handles_representative_dense_cloud_quickly(self):
        rng = np.random.default_rng(7)
        count = 172_000
        points = np.column_stack(
            (
                rng.uniform(-4.0, 4.0, count),
                rng.uniform(-4.0, 4.0, count),
                rng.uniform(0.05, 1.5, count),
            )
        ).astype(np.float32)

        started = time.perf_counter()
        make_local_costmaps(points, LocalCostmapConfig())
        elapsed_s = time.perf_counter() - started

        # A generous unit-test guard; hardware acceptance uses the stricter 100 ms target.
        self.assertLess(elapsed_s, 1.0)

    def test_cloud_timestamp_accepts_recent_data(self):
        self.assertIsNone(
            cloud_timestamp_error(
                now_ns=2_000_000_000,
                stamp_ns=1_500_000_000,
                max_age_s=1.0,
                max_future_offset_s=0.1,
            )
        )

    def test_cloud_timestamp_rejects_stale_future_and_invalid_data(self):
        common = {
            "now_ns": 2_000_000_000,
            "max_age_s": 1.0,
            "max_future_offset_s": 0.1,
        }
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=0, **common),
            "invalid_lidar_timestamp",
        )
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=500_000_000, **common),
            "stale_lidar",
        )
        self.assertEqual(
            cloud_timestamp_error(stamp_ns=2_200_000_000, **common),
            "future_lidar",
        )


if __name__ == "__main__":
    unittest.main()
