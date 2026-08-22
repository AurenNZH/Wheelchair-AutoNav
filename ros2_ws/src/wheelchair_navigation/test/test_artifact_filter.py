import unittest

import numpy as np
from wheelchair_navigation.artifact_filter import (
    ArtifactGridCell,
    ArtifactHaloSpan,
    artifact_configured_halo_cell_ids,
    artifact_grid_membership,
    artifact_halo_cell_ids,
    filter_artifact_points,
    minimum_cell_support_filter,
    parse_artifact_grid_cells,
    parse_artifact_grid_halo_spans,
    validate_artifact_filter_frame,
    validate_artifact_grid_cells,
    validate_artifact_grid_halo_spans,
)
from wheelchair_navigation.costmap import (
    FrontCostmapConfig,
    LocalCostmapConfig,
    SelfFilterBox,
    front_point_cell_ids,
    make_front_grid,
    make_full_raw_grid,
    obstacle_point_mask,
)


class ArtifactFilterTests(unittest.TestCase):
    def test_upstream_keep_mask_retains_nav2_owned_points(self):
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=1.0
        )
        points = np.array(
            [
                [0.5, 0.5, 0.4],
                [0.2, -0.5, 0.4],
                [1.5, -0.5, 0.4],
                [1.6, -0.5, 0.4],
                [5.0, 0.0, 0.4],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )
        eligible = np.array([True, False, True, True, False, False])
        boxes = (SelfFilterBox(0.0, 0.4, -1.0, 0.0, 0.0, 1.0),)
        cells = (ArtifactGridCell(0, 0, 0, 0.3, 0.5),)
        spans = (ArtifactHaloSpan(0, 0, 0, 0),)

        result = filter_artifact_points(
            points,
            eligible,
            boxes,
            cells,
            spans,
            config,
            min_points_per_cell=3,
            global_min_points_per_cell=3,
        )

        np.testing.assert_array_equal(
            result.keep_mask,
            [False, False, False, False, True, False],
        )
        self.assertEqual(result.artifact_stats.unique_rejected_points, 1)
        self.assertEqual(result.support.stats.low_support_points, 2)

    def test_parser_accepts_empty_valid_and_cross_region_overlap(self):
        self.assertEqual(parse_artifact_grid_cells(None), ())
        self.assertEqual(parse_artifact_grid_cells([]), ())

        cells = parse_artifact_grid_cells(
            [
                0.0, 2.0, -1.0, 0.2, 0.3,
                1.0, 2.0, -1.0, 0.25, 0.35,
            ]
        )

        self.assertEqual(len(cells), 2)
        self.assertEqual(cells[0], ArtifactGridCell(0, 2, -1, 0.2, 0.3))

    def test_parser_rejects_invalid_records_and_same_region_duplicates(self):
        invalid = (
            [0.0, 1.0],
            [0.0, 1.0, 0.0, np.nan, 0.3],
            [0.5, 1.0, 0.0, 0.2, 0.3],
            [-1.0, 1.0, 0.0, 0.2, 0.3],
            [0.0, -1.0, 0.0, 0.2, 0.3],
            [0.0, 1.0, 0.0, 0.4, 0.3],
            [
                0.0, 1.0, 0.0, 0.2, 0.3,
                0.0, 1.0, 0.0, 0.1, 0.4,
            ],
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_artifact_grid_cells(values)

    def test_halo_span_parser_accepts_ranges_and_rejects_overlap(self):
        spans = parse_artifact_grid_halo_spans(
            [0.0, 1.0, -2.0, 0.0, 1.0, 3.0, 2.0, 4.0]
        )

        self.assertEqual(spans[0], ArtifactHaloSpan(0, 1, -2, 0))
        invalid = (
            [0.0, 1.0],
            [0.0, 1.0, -1.0, np.nan],
            [0.5, 1.0, -1.0, 1.0],
            [0.0, 1.0, 1.0, -1.0],
            [0.0, 1.0, -1.0, 1.0, 0.0, 1.0, 1.0, 2.0],
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_artifact_grid_halo_spans(values)

    def test_grid_validation_checks_regions_bounds_and_frame(self):
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )
        validate_artifact_grid_cells(
            (ArtifactGridCell(0, 3, 1, 0.2, 0.3),), config
        )
        invalid = (
            (ArtifactGridCell(1, 0, 0, 0.2, 0.3),),
            (ArtifactGridCell(0, 4, 0, 0.2, 0.3),),
            (ArtifactGridCell(0, 0, -3, 0.2, 0.3),),
        )
        for cells in invalid:
            with self.subTest(cells=cells), self.assertRaises(ValueError):
                validate_artifact_grid_cells(cells, config)

        validate_artifact_filter_frame("base_link", "base_link")
        with self.assertRaises(ValueError):
            validate_artifact_filter_frame("rslidar", "base_link")

    def test_cell_membership_uses_half_open_xy_and_inclusive_z(self):
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )
        cell = ArtifactGridCell(0, 1, 0, 0.2, 0.3)
        points = np.array(
            [
                [0.5, 0.0, 0.2],
                [0.999, 0.499, 0.3],
                [1.0, 0.0, 0.25],
                [0.75, -0.001, 0.25],
                [0.75, 0.1, 0.31],
                [np.nan, 0.1, 0.25],
            ],
            dtype=np.float32,
        )
        valid, cell_ids, _ = front_point_cell_ids(points, config)

        membership, stats = artifact_grid_membership(
            points, cell_ids, valid, (cell,), config
        )

        np.testing.assert_array_equal(
            membership, [True, True, False, False, False, False]
        )
        self.assertEqual(stats.region_count, 1)
        self.assertEqual(stats.per_region_rejected_points, (2,))
        self.assertEqual(stats.unique_rejected_points, 2)

    def test_overlapping_regions_report_region_and_unique_counts(self):
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )
        cells = (
            ArtifactGridCell(0, 1, 0, 0.1, 0.3),
            ArtifactGridCell(1, 1, 0, 0.2, 0.4),
        )
        points = np.array(
            [[0.75, 0.1, 0.15], [0.75, 0.1, 0.25], [0.75, 0.1, 0.35]],
            dtype=np.float32,
        )
        valid, cell_ids, _ = front_point_cell_ids(points, config)

        membership, stats = artifact_grid_membership(
            points, cell_ids, valid, cells, config
        )

        np.testing.assert_array_equal(membership, [True, True, True])
        self.assertEqual(stats.per_region_rejected_points, (2, 2))
        self.assertEqual(stats.unique_rejected_points, 3)

    def test_halo_is_eight_connected_and_clipped_to_grid(self):
        config = FrontCostmapConfig(
            length_m=4.0, width_m=4.0, resolution_m=1.0
        )
        centre = (ArtifactGridCell(0, 1, 0, 0.2, 0.3),)
        edge = (ArtifactGridCell(0, 0, -2, 0.2, 0.3),)

        np.testing.assert_array_equal(
            artifact_halo_cell_ids(centre, config, 1),
            [4, 5, 6, 8, 9, 10, 12, 13, 14],
        )
        np.testing.assert_array_equal(
            artifact_halo_cell_ids(edge, config, 1), [0, 1, 4, 5]
        )

    def test_configured_halo_is_independent_but_must_cover_mask(self):
        config = FrontCostmapConfig(
            length_m=4.0, width_m=4.0, resolution_m=1.0
        )
        cells = (ArtifactGridCell(0, 1, 0, 0.2, 0.3),)
        spans = (
            ArtifactHaloSpan(0, 0, -1, 1),
            ArtifactHaloSpan(0, 1, -1, 1),
            ArtifactHaloSpan(0, 2, -1, 1),
        )

        np.testing.assert_array_equal(
            artifact_configured_halo_cell_ids(spans, cells, config),
            [4, 5, 6, 8, 9, 10, 12, 13, 14],
        )
        with self.assertRaises(ValueError):
            validate_artifact_grid_halo_spans(
                (ArtifactHaloSpan(0, 0, -1, 1),), cells, config
            )

    def test_minimum_support_uses_fixed_candidates_and_is_disjoint(self):
        cell_ids = np.array([0, 0, 1, 2, 2, 3, 3], dtype=np.int64)
        valid = np.ones(7, dtype=bool)
        eligible = np.ones(7, dtype=bool)
        rejected = np.array(
            [True, False, False, False, False, False, False]
        )

        result = minimum_cell_support_filter(
            cell_ids,
            valid,
            eligible,
            rejected,
            np.array([0, 1, 2]),
            cell_count=4,
            min_points_per_cell=2,
        )

        np.testing.assert_array_equal(
            result.low_support_mask,
            [False, True, True, False, False, False, False],
        )
        np.testing.assert_array_equal(
            result.shadow_mask,
            [False, False, False, True, True, True, True],
        )
        self.assertFalse(np.any(result.low_support_mask & rejected))
        self.assertEqual(result.stats.mask_touched_cells, 1)
        self.assertEqual(result.stats.mask_removed_cells, 0)
        self.assertEqual(result.stats.mask_mixed_cells, 1)
        self.assertEqual(result.stats.threshold_candidate_cells, 3)
        self.assertEqual(result.stats.low_support_cells, 2)
        self.assertEqual(result.stats.low_support_points, 2)
        self.assertEqual(result.stats.configured_halo_cells, 3)
        np.testing.assert_array_equal(result.candidate_cell_ids, [0, 1, 2])
        np.testing.assert_array_equal(result.low_support_cell_ids, [0, 1])

    def test_minimum_one_and_empty_scope_leave_points_unchanged(self):
        common = {
            "cell_ids": np.array([0, 1], dtype=np.int64),
            "valid_front_mask": np.ones(2, dtype=bool),
            "eligible_mask": np.ones(2, dtype=bool),
            "mask_rejected_mask": np.zeros(2, dtype=bool),
            "threshold_candidate_cell_ids": np.array([], dtype=np.int64),
            "cell_count": 2,
        }

        result = minimum_cell_support_filter(
            min_points_per_cell=1, **common
        )
        stricter = minimum_cell_support_filter(
            min_points_per_cell=3, **common
        )

        np.testing.assert_array_equal(result.shadow_mask, [True, True])
        np.testing.assert_array_equal(stricter.shadow_mask, [True, True])
        self.assertEqual(stricter.stats.threshold_candidate_cells, 0)

    def test_global_minimum_applies_outside_configured_halo(self):
        result = minimum_cell_support_filter(
            cell_ids=np.array([0, 1, 1, 1, 2, 2], dtype=np.int64),
            valid_front_mask=np.ones(6, dtype=bool),
            eligible_mask=np.ones(6, dtype=bool),
            mask_rejected_mask=np.zeros(6, dtype=bool),
            threshold_candidate_cell_ids=np.array([], dtype=np.int64),
            cell_count=3,
            min_points_per_cell=15,
            global_min_points_per_cell=3,
        )

        np.testing.assert_array_equal(
            result.shadow_mask, [False, True, True, True, False, False]
        )
        np.testing.assert_array_equal(result.low_support_cell_ids, [0, 2])
        self.assertEqual(result.stats.global_min_points_per_cell, 3)
        self.assertEqual(result.stats.low_support_cells, 2)
        self.assertEqual(result.stats.low_support_points, 3)

    def test_invalid_support_configuration_is_rejected(self):
        common = {
            "cell_ids": np.array([0], dtype=np.int64),
            "valid_front_mask": np.array([True]),
            "eligible_mask": np.array([True]),
            "mask_rejected_mask": np.array([False]),
            "threshold_candidate_cell_ids": np.array([0]),
            "cell_count": 1,
        }
        for value in (0, -1, 1.5, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                minimum_cell_support_filter(
                    min_points_per_cell=value, **common
                )
            with self.subTest(global_value=value), self.assertRaises(
                ValueError
            ):
                minimum_cell_support_filter(
                    min_points_per_cell=1,
                    global_min_points_per_cell=value,
                    **common,
                )

    def test_raw_maps_are_unchanged_and_mixed_mask_cell_remains(self):
        map_config = LocalCostmapConfig(min_range_m=0.0)
        front_config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )
        points = np.array(
            [
                [0.75, 0.1, 0.4],
                [0.75, 0.1, 0.8],
                [1.25, 0.1, 0.4],
                [1.75, 0.1, 0.4],
            ],
            dtype=np.float32,
        )
        cells = (
            ArtifactGridCell(0, 1, 0, 0.3, 0.5),
            ArtifactGridCell(0, 2, 0, 0.3, 0.5),
        )
        eligible, _ = obstacle_point_mask(points, map_config)
        raw_before = make_full_raw_grid(points[eligible], map_config)
        front_before = make_front_grid(points[eligible], front_config)
        valid, cell_ids, _ = front_point_cell_ids(points, front_config)
        rejected, _ = artifact_grid_membership(
            points, cell_ids, valid, cells, front_config, eligible
        )
        shadow_front = make_front_grid(
            points[eligible & ~rejected & valid], front_config
        )
        raw_after = make_full_raw_grid(points[eligible], map_config)
        front_after = make_front_grid(points[eligible], front_config)

        self.assertEqual(raw_before.tobytes(), raw_after.tobytes())
        self.assertEqual(front_before.tobytes(), front_after.tobytes())
        self.assertEqual(front_before[2, 1], 100)
        self.assertEqual(shadow_front[2, 1], 100)
        self.assertEqual(front_before[2, 2], 100)
        self.assertEqual(shadow_front[2, 2], 0)
        self.assertEqual(shadow_front[2, 3], 100)


if __name__ == "__main__":
    unittest.main()
