import unittest

import numpy as np

from wheelchair_navigation.artifact_filter import (
    artifact_halo_cell_ids,
    parse_artifact_box,
    parse_artifact_boxes,
    parse_artifact_halo_bounds,
    points_in_artifact_box,
    points_in_artifact_boxes,
)
from wheelchair_navigation.costmap import SupportGridConfig


class ArtifactFilterTests(unittest.TestCase):
    def test_box_is_inclusive_and_rejects_only_finite_points(self):
        box = parse_artifact_box([0.0, 0.2, -0.1, 0.1, 0.3, 0.5])
        points = np.array(
            [
                [0.0, -0.1, 0.3],
                [0.2, 0.1, 0.5],
                [0.21, 0.0, 0.4],
                [np.nan, 0.0, 0.4],
            ],
            dtype=np.float32,
        )

        np.testing.assert_array_equal(
            points_in_artifact_box(points, box),
            [True, True, False, False],
        )

    def test_box_rejects_missing_nonfinite_and_reversed_bounds(self):
        for values in (
            [],
            [0.0] * 5,
            [0.0, 0.0, -0.1, 0.1, 0.3, 0.5],
            [0.0, 0.2, 0.1, -0.1, 0.3, 0.5],
            [0.0, 0.2, -0.1, 0.1, np.nan, 0.5],
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_artifact_box(values)

    def test_additional_boxes_parse_flat_groups_and_reject_their_union(self):
        boxes = parse_artifact_boxes(
            [
                0.50, 0.60, -0.30, -0.20, 0.77, 0.87,
                -0.10, 0.00, 0.10, 0.20, 0.30, 0.40,
            ]
        )
        points = np.asarray(
            [
                [0.55, -0.25, 0.82],
                [-0.05, 0.15, 0.35],
                [0.55, -0.25, 0.88],
                [0.61, -0.25, 0.82],
            ],
            dtype=np.float32,
        )

        self.assertEqual(len(boxes), 2)
        np.testing.assert_array_equal(
            points_in_artifact_boxes(points, boxes),
            [True, True, False, False],
        )
        self.assertEqual(parse_artifact_boxes([]), ())
        with self.assertRaises(ValueError):
            parse_artifact_boxes([0.0] * 7)

    def test_halo_is_expanded_and_clipped_to_support_grid(self):
        config = SupportGridConfig(
            origin_x_m=0.0,
            origin_y_m=-0.5,
            width_m=1.0,
            height_m=1.0,
            resolution_m=0.1,
        )
        box = parse_artifact_box([0.0, 0.2, -0.1, 0.1, 0.3, 0.5])

        cells = artifact_halo_cell_ids(box, config, 0.1)

        self.assertTrue(np.all(cells >= 0))
        self.assertTrue(np.all(cells < 100))
        self.assertIn(40, cells)
        self.assertIn(63, cells)
        outside = parse_artifact_box([-2.0, -1.0, -0.1, 0.1, 0.3, 0.5])
        self.assertEqual(artifact_halo_cell_ids(outside, config, 0.1).size, 0)
        with self.assertRaises(ValueError):
            artifact_halo_cell_ids(box, config, -0.1)

    def test_explicit_halo_bounds_allow_signed_base_link_geometry(self):
        halo = parse_artifact_halo_bounds([-0.30, 0.77, -0.70, 0.61])

        self.assertEqual(halo.min_x_m, -0.30)
        self.assertEqual(halo.max_y_m, 0.61)
        for values in (
            [],
            [-0.3, 0.7, -0.7],
            [0.7, -0.3, -0.7, 0.6],
            [-0.3, 0.7, np.nan, 0.6],
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_artifact_halo_bounds(values)


if __name__ == "__main__":
    unittest.main()
