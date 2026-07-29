import time
import unittest
from pathlib import Path

import numpy as np
import yaml

from wheelchair_navigation.local_navigation import (
    FrontCostmapConfig,
    GhostFilterConfig,
    LocalCostmapConfig,
    SelfFilterBox,
    TemporalGhostFilter,
    filter_obstacle_points,
    make_local_and_front_costmaps,
    make_local_costmaps,
    parse_self_filter_boxes,
    world_to_cell,
)
from wheelchair_navigation.local_navigation_node import cloud_timestamp_error


class LocalNavigationTests(unittest.TestCase):
    def test_demo_profile_skips_duplicate_derived_costmap(self):
        config_path = (
            Path(__file__).resolve().parents[1]
            / "config"
            / "local_mapping.yaml"
        )
        parameters = yaml.safe_load(config_path.read_text())["local_costmap"][
            "ros__parameters"
        ]

        self.assertTrue(parameters["publish_raw_obstacles"])
        self.assertFalse(parameters["publish_derived_costmap"])
        self.assertTrue(parameters["publish_front_costmap"])
        self.assertTrue(parameters["publish_filtered_front_costmap"])
        self.assertTrue(parameters["publish_rejected_front_costmap"])
        self.assertEqual(
            parameters["filtered_front_costmap_topic"],
            "/front_costmap_filtered",
        )
        self.assertEqual(
            parameters["rejected_front_costmap_topic"],
            "/front_costmap_rejected",
        )
        self.assertEqual(parameters["front_length_m"], 4.0)
        self.assertEqual(
            parameters["self_filter_boxes"],
            [0.0, 0.53, -0.465, 0.2, 0.32, 0.82],
        )
        self.assertEqual(parameters["self_filter_padding_m"], 0.0)
        self.assertEqual(parameters["ghost_filter_min_component_cells"], 2)
        self.assertEqual(parameters["ghost_filter_history_frames"], 3)
        self.assertEqual(parameters["ghost_filter_min_hits"], 2)
        self.assertEqual(parameters["ghost_filter_match_radius_cells"], 1)
        self.assertEqual(parameters["ghost_filter_reset_gap_s"], 0.5)

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

    def test_front_map_contains_only_forward_sector(self):
        points = np.array(
            [
                [1.0, 0.0, 0.4],
                [1.0, 1.0, 0.4],
                [-1.0, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        raw, costmap, front, stats = make_local_and_front_costmaps(
            points,
            LocalCostmapConfig(size_m=4.0, resolution_m=0.1),
            FrontCostmapConfig(length_m=2.0, width_m=4.0, resolution_m=0.1),
        )

        np.testing.assert_array_equal(raw, costmap)
        self.assertEqual(front.shape, (40, 20))
        self.assertEqual(np.count_nonzero(raw == 100), 3)
        self.assertEqual(np.count_nonzero(front == 100), 2)
        self.assertEqual(stats.front_points, 2)
        self.assertEqual(stats.front_occupied_cells, 2)

    def test_front_fov_can_be_narrowed_without_changing_full_map(self):
        points = np.array(
            [[1.0, 0.0, 0.4], [1.0, 1.0, 0.4]], dtype=np.float32
        )

        raw, _, front, _ = make_local_and_front_costmaps(
            points,
            LocalCostmapConfig(size_m=4.0),
            FrontCostmapConfig(length_m=2.0, width_m=4.0, fov_deg=60.0),
        )

        self.assertEqual(np.count_nonzero(raw == 100), 2)
        self.assertEqual(np.count_nonzero(front == 100), 1)

    def test_multi_cell_component_is_kept_immediately(self):
        ghost_filter = TemporalGhostFilter()
        raw = np.zeros((6, 6), dtype=np.int8)
        raw[2, 2:4] = 100

        filtered, rejected, stats = ghost_filter.filter(
            raw, stamp_ns=1_000_000_000
        )

        np.testing.assert_array_equal(filtered, raw)
        self.assertEqual(np.count_nonzero(rejected), 0)
        self.assertEqual(stats.strong_component_cells, 2)
        self.assertEqual(stats.rejected_cells, 0)

    def test_isolated_cell_requires_two_temporally_matched_hits(self):
        ghost_filter = TemporalGhostFilter()
        raw = np.zeros((6, 6), dtype=np.int8)
        raw[2, 2] = 100

        first_filtered, first_rejected, first_stats = ghost_filter.filter(
            raw, stamp_ns=1_000_000_000
        )
        second_filtered, second_rejected, second_stats = ghost_filter.filter(
            raw, stamp_ns=1_100_000_000
        )

        self.assertEqual(np.count_nonzero(first_filtered), 0)
        np.testing.assert_array_equal(first_rejected, raw)
        self.assertEqual(first_stats.rejected_cells, 1)
        np.testing.assert_array_equal(second_filtered, raw)
        self.assertEqual(np.count_nonzero(second_rejected), 0)
        self.assertEqual(second_stats.temporal_rescued_cells, 1)

    def test_isolated_cell_can_move_one_cell_between_frames(self):
        ghost_filter = TemporalGhostFilter()
        first = np.zeros((6, 6), dtype=np.int8)
        first[2, 2] = 100
        second = np.zeros((6, 6), dtype=np.int8)
        second[2, 3] = 100

        ghost_filter.filter(first, stamp_ns=1_000_000_000)
        filtered, _, stats = ghost_filter.filter(
            second, stamp_ns=1_100_000_000
        )

        np.testing.assert_array_equal(filtered, second)
        self.assertEqual(stats.temporal_rescued_cells, 1)

    def test_isolated_cell_outside_match_radius_remains_rejected(self):
        ghost_filter = TemporalGhostFilter()
        first = np.zeros((6, 6), dtype=np.int8)
        first[2, 1] = 100
        second = np.zeros((6, 6), dtype=np.int8)
        second[2, 4] = 100

        ghost_filter.filter(first, stamp_ns=1_000_000_000)
        filtered, rejected, stats = ghost_filter.filter(
            second, stamp_ns=1_100_000_000
        )

        self.assertEqual(np.count_nonzero(filtered), 0)
        np.testing.assert_array_equal(rejected, second)
        self.assertEqual(stats.rejected_cells, 1)

    def test_ghost_history_resets_after_cloud_gap(self):
        ghost_filter = TemporalGhostFilter()
        raw = np.zeros((6, 6), dtype=np.int8)
        raw[2, 2] = 100
        original = np.array(raw, copy=True)

        ghost_filter.filter(raw, stamp_ns=1_000_000_000)
        filtered, rejected, stats = ghost_filter.filter(
            raw, stamp_ns=1_600_000_000
        )

        np.testing.assert_array_equal(raw, original)
        self.assertEqual(np.count_nonzero(filtered), 0)
        np.testing.assert_array_equal(rejected, raw)
        self.assertTrue(stats.history_reset)

    def test_invalid_ghost_filter_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            TemporalGhostFilter(GhostFilterConfig(min_component_cells=0))
        with self.assertRaises(ValueError):
            TemporalGhostFilter(
                GhostFilterConfig(history_frames=2, min_hits=3)
            )
        with self.assertRaises(ValueError):
            TemporalGhostFilter(GhostFilterConfig(match_radius_cells=-1))
        with self.assertRaises(ValueError):
            TemporalGhostFilter(GhostFilterConfig(reset_gap_s=0.0))

    def test_invalid_front_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            make_local_and_front_costmaps(
                np.empty((0, 3), dtype=np.float32),
                LocalCostmapConfig(),
                FrontCostmapConfig(fov_deg=181.0),
            )

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

    def test_provisional_mount_box_keeps_points_beyond_each_open_face(self):
        box_values = [0.0, 0.53, -0.465, 0.2, 0.32, 0.82]
        boxes = parse_self_filter_boxes(box_values, padding_m=0.0)
        points = np.array(
            [
                [0.30, -0.20, 0.50],  # inside
                [0.54, -0.20, 0.50],  # beyond front
                [0.30, -0.475, 0.50],  # beyond right
                [0.30, 0.21, 0.50],  # beyond left
                [0.30, -0.20, 0.31],  # below
                [0.30, -0.20, 0.83],  # above
            ],
            dtype=np.float32,
        )

        accepted, counts = filter_obstacle_points(
            points, LocalCostmapConfig(), boxes
        )

        self.assertEqual(counts["self_filtered_points"], 1)
        self.assertEqual(accepted.shape[0], 5)

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
