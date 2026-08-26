import unittest

import numpy as np

from wheelchair_navigation.artifact_filter import (
    artifact_halo_cell_ids,
    parse_artifact_box,
    points_in_artifact_box,
)
from wheelchair_navigation.costmap import FrontCostmapConfig


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

    def test_halo_is_expanded_and_clipped_to_front_grid(self):
        config = FrontCostmapConfig(
            length_m=1.0, width_m=1.0, resolution_m=0.1
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


if __name__ == "__main__":
    unittest.main()
