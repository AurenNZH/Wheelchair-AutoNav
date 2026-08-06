import unittest

import numpy as np
from std_msgs.msg import Header

from wheelchair_navigation.artifact_filter import (
    ArtifactPancakeMask,
    artifact_pancake_membership,
    artifact_xy_halo_membership,
    make_artifact_shadow_points,
    minimum_cell_support_filter,
    parse_artifact_pancake_masks,
)
from wheelchair_navigation.local_navigation import (
    FrontCostmapConfig,
    LocalCostmapConfig,
    make_front_grid,
    make_full_raw_grid,
    obstacle_point_mask,
    select_front_points,
)
from wheelchair_navigation.local_navigation_node import (
    artifact_shadow_error_reason,
    build_artifact_mask_markers,
)


class ArtifactFilterTests(unittest.TestCase):
    def test_parser_accepts_empty_and_valid_flat_groups(self):
        self.assertEqual(parse_artifact_pancake_masks(None), ())
        self.assertEqual(parse_artifact_pancake_masks([]), ())

        masks = parse_artifact_pancake_masks(
            [0.0, 0.0, 1.0, 1.0, 0.05, -0.1, 0.1]
        )

        self.assertEqual(len(masks), 1)
        self.assertAlmostEqual(masks[0].length_m, np.sqrt(2.0))

    def test_parser_rejects_incomplete_nonfinite_and_invalid_geometry(self):
        invalid = (
            [0.0, 1.0],
            [0.0, 0.0, np.nan, 1.0, 0.1, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 1.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 0.0, 0.1, 1.0, 0.0],
        )
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_artifact_pancake_masks(values)

    def test_rotated_segment_includes_endpoints_and_z_boundaries(self):
        mask = ArtifactPancakeMask(0.0, 0.0, 1.0, 1.0, 0.1, 0.2, 0.3)
        points = np.array(
            [
                [0.0, 0.0, 0.2],
                [1.0, 1.0, 0.3],
                [0.5, 0.55, 0.25],
                [0.5, 0.7, 0.25],
                [-0.01, -0.01, 0.25],
                [0.5, 0.5, 0.31],
                [np.nan, 0.0, 0.25],
            ],
            dtype=np.float32,
        )

        membership, stats = artifact_pancake_membership(points, (mask,))

        np.testing.assert_array_equal(
            membership, [True, True, True, False, False, False, False]
        )
        self.assertEqual(stats.per_mask_rejected_points, (3,))
        self.assertEqual(stats.unique_rejected_points, 3)

    def test_overlapping_masks_report_per_mask_and_unique_counts(self):
        masks = (
            ArtifactPancakeMask(0.0, 0.0, 1.0, 0.0, 0.1, 0.0, 1.0),
            ArtifactPancakeMask(0.5, 0.0, 1.5, 0.0, 0.1, 0.0, 1.0),
        )
        points = np.array(
            [[0.25, 0.0, 0.5], [0.75, 0.0, 0.5], [1.25, 0.0, 0.5]],
            dtype=np.float32,
        )

        membership, stats = artifact_pancake_membership(points, masks)

        np.testing.assert_array_equal(membership, [True, True, True])
        self.assertEqual(stats.per_mask_rejected_points, (2, 2))
        self.assertEqual(stats.unique_rejected_points, 3)

    def test_xy_halo_expands_width_and_endpoints_but_ignores_z(self):
        mask = ArtifactPancakeMask(0.0, 0.0, 1.0, 0.0, 0.1, 0.2, 0.3)
        points = np.array(
            [
                [-0.05, 0.0, 9.0],
                [1.05, 0.0, -9.0],
                [0.5, 0.15, 2.0],
                [0.5, 0.21, 0.25],
                [-0.11, 0.0, 0.25],
            ],
            dtype=np.float32,
        )

        membership = artifact_xy_halo_membership(
            points, (mask,), halo_m=0.1
        )

        np.testing.assert_array_equal(
            membership, [True, True, True, False, False]
        )

    def test_minimum_support_is_local_disjoint_and_overlap_safe(self):
        cell_ids = np.array([0, 0, 1, 2, 2, 3, 3], dtype=np.int64)
        valid = np.ones(7, dtype=bool)
        eligible = np.ones(7, dtype=bool)
        prism_rejected = np.array(
            [True, False, False, False, False, False, False]
        )
        # Cell 0 enters the threshold scope only because the prism touched it;
        # cells 1 and 2 enter through the halo. Repeated points in a cell are
        # counted once each, regardless of which mask halos overlap there.
        halo = np.array([False, False, True, True, False, False, False])

        result = minimum_cell_support_filter(
            cell_ids,
            valid,
            eligible,
            prism_rejected,
            halo,
            cell_count=4,
            min_points_per_cell=2,
            halo_m=0.1,
        )

        np.testing.assert_array_equal(
            result.low_support_mask,
            [False, True, True, False, False, False, False],
        )
        np.testing.assert_array_equal(
            result.shadow_mask,
            [False, False, False, True, True, True, True],
        )
        self.assertFalse(
            np.any(result.low_support_mask & prism_rejected)
        )
        self.assertEqual(result.stats.prism_touched_cells, 1)
        self.assertEqual(result.stats.prism_removed_cells, 0)
        self.assertEqual(result.stats.prism_mixed_cells, 1)
        self.assertEqual(result.stats.threshold_candidate_cells, 3)
        self.assertEqual(result.stats.low_support_cells, 2)
        self.assertEqual(result.stats.low_support_points, 2)

    def test_minimum_one_and_empty_scope_leave_points_unchanged(self):
        common = {
            "cell_ids": np.array([0, 1], dtype=np.int64),
            "valid_front_mask": np.ones(2, dtype=bool),
            "eligible_mask": np.ones(2, dtype=bool),
            "prism_rejected_mask": np.zeros(2, dtype=bool),
            "halo_mask": np.zeros(2, dtype=bool),
            "cell_count": 2,
            "halo_m": 0.1,
        }

        result = minimum_cell_support_filter(
            min_points_per_cell=1, **common
        )
        stricter_without_scope = minimum_cell_support_filter(
            min_points_per_cell=3, **common
        )

        np.testing.assert_array_equal(result.shadow_mask, [True, True])
        np.testing.assert_array_equal(
            stricter_without_scope.shadow_mask, [True, True]
        )
        self.assertEqual(result.stats.low_support_points, 0)
        self.assertEqual(
            stricter_without_scope.stats.threshold_candidate_cells, 0
        )

    def test_invalid_support_configuration_is_rejected(self):
        common = {
            "cell_ids": np.array([0], dtype=np.int64),
            "valid_front_mask": np.array([True]),
            "eligible_mask": np.array([True]),
            "prism_rejected_mask": np.array([False]),
            "halo_mask": np.array([True]),
            "cell_count": 1,
            "halo_m": 0.1,
        }
        for value in (0, -1, 1.5, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                minimum_cell_support_filter(
                    min_points_per_cell=value, **common
                )
        with self.assertRaises(ValueError):
            minimum_cell_support_filter(
                min_points_per_cell=2,
                **{**common, "halo_m": -0.1},
            )
        with self.assertRaises(ValueError):
            artifact_xy_halo_membership(
                np.zeros((1, 3), dtype=np.float32), (), np.nan
            )

    def test_empty_masks_leave_raw_and_front_maps_bit_for_bit_identical(self):
        points = np.array(
            [[0.5, 0.0, 0.4], [1.0, 0.2, 0.4]], dtype=np.float32
        )
        map_config = LocalCostmapConfig(size_m=4.0, resolution_m=0.1)
        front_config = FrontCostmapConfig(
            length_m=2.0, width_m=4.0, resolution_m=0.1
        )
        eligible, _ = obstacle_point_mask(points, map_config)
        raw_before = make_full_raw_grid(points[eligible], map_config)
        front_before = make_front_grid(
            select_front_points(points[eligible], front_config), front_config
        )

        shadow_points, rejected, stats = make_artifact_shadow_points(
            points,
            points,
            eligible,
            (),
            source_frame="rslidar",
            artifact_filter_frame="rslidar",
        )
        raw_after = make_full_raw_grid(points[eligible], map_config)
        front_after = make_front_grid(
            select_front_points(shadow_points, front_config), front_config
        )

        self.assertEqual(raw_before.tobytes(), raw_after.tobytes())
        self.assertEqual(front_before.tobytes(), front_after.tobytes())
        self.assertEqual(rejected.shape, (0, 3))
        self.assertEqual(stats.unique_rejected_points, 0)

    def test_masked_only_cell_disappears_but_mixed_cell_remains(self):
        points = np.array(
            [
                [0.55, 0.0, 0.4],
                [0.55, 0.2, 0.4],
                [1.10, 0.0, 0.4],
                [1.60, 0.0, 0.4],
            ],
            dtype=np.float32,
        )
        mask = ArtifactPancakeMask(0.4, 0.0, 1.4, 0.0, 0.05, 0.3, 0.5)
        map_config = LocalCostmapConfig(min_range_m=0.0)
        front_config = FrontCostmapConfig(
            length_m=2.0, width_m=4.0, resolution_m=0.5
        )
        eligible, _ = obstacle_point_mask(points, map_config)
        raw_full_before = make_full_raw_grid(points[eligible], map_config)
        shadow_points, rejected, _ = make_artifact_shadow_points(
            points,
            points,
            eligible,
            (mask,),
            source_frame="rslidar",
            artifact_filter_frame="rslidar",
        )
        raw_front = make_front_grid(points, front_config)
        raw_full_after = make_full_raw_grid(points[eligible], map_config)
        raw_front_after = make_front_grid(points[eligible], front_config)
        shadow_front = make_front_grid(shadow_points, front_config)

        self.assertEqual(raw_full_before.tobytes(), raw_full_after.tobytes())
        self.assertEqual(raw_front.tobytes(), raw_front_after.tobytes())
        self.assertEqual(raw_front[4, 1], 100)
        self.assertEqual(shadow_front[4, 1], 100)
        self.assertEqual(raw_front[4, 2], 100)
        self.assertEqual(shadow_front[4, 2], 0)
        self.assertEqual(raw_front[4, 3], 100)
        self.assertEqual(shadow_front[4, 3], 100)
        self.assertEqual(rejected.shape[0], 2)

    def test_source_frame_mismatch_suppresses_shadow_with_explicit_reason(self):
        with self.assertRaises(ValueError) as context:
            make_artifact_shadow_points(
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
                np.zeros(0, dtype=bool),
                (),
                source_frame="base_link",
                artifact_filter_frame="rslidar",
            )

        self.assertEqual(
            artifact_shadow_error_reason(context.exception),
            "artifact_shadow_frame_mismatch",
        )

    def test_mask_markers_match_configured_pose_and_dimensions(self):
        header = Header()
        header.frame_id = "rslidar"
        mask = ArtifactPancakeMask(1.0, 2.0, 1.0, 4.0, 0.15, 0.2, 0.3)

        markers = build_artifact_mask_markers(header, (mask,)).markers
        marker = markers[0]

        self.assertEqual(len(markers), 3)
        self.assertEqual(marker.header.frame_id, "rslidar")
        self.assertAlmostEqual(marker.pose.position.x, 1.0)
        self.assertAlmostEqual(marker.pose.position.y, 3.0)
        self.assertAlmostEqual(marker.pose.position.z, 0.25)
        self.assertAlmostEqual(marker.scale.x, 2.0)
        self.assertAlmostEqual(marker.scale.y, 0.3)
        self.assertAlmostEqual(marker.scale.z, 0.1)
        self.assertAlmostEqual(marker.pose.orientation.z, np.sqrt(0.5))
        self.assertAlmostEqual(marker.pose.orientation.w, np.sqrt(0.5))
        self.assertLess(marker.color.a, 0.5)
        self.assertEqual(markers[1].type, markers[1].LINE_LIST)
        self.assertEqual(len(markers[1].points), 24)
        self.assertEqual(markers[2].text, "MASK 0")


if __name__ == "__main__":
    unittest.main()
