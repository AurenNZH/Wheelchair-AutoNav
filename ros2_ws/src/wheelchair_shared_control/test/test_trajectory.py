import unittest

import numpy as np

from wheelchair_shared_control.models import (
    SafetyConfig,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.trajectory import (
    swept_path_costs,
    trajectory_points,
)


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.config = SafetyConfig(
            stop_distance_m=0.7,
            slow_distance_m=1.2,
            slow_cost_threshold=1,
            stop_cost_threshold=99,
        )

    @staticmethod
    def costmap(cells, *, width=40, height=80):
        values = np.zeros(width * height, dtype=np.int16)
        for (col, row), cost in cells.items():
            values[row * width + col] = cost
        return weighted_costmap_from_grid(
            values,
            frame_id="base_link",
            width=width,
            height=height,
            resolution_m=0.1,
            origin_x_m=0.0,
            origin_y_m=-4.0,
            origin_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

    def test_straight_and_curved_samples_keep_existing_geometry(self):
        straight = trajectory_points(0.0, self.config)
        left = trajectory_points(0.35, self.config)
        right = trajectory_points(-0.35, self.config)

        self.assertEqual(len(straight), 25)
        self.assertEqual(straight[0], (0.0, 0.0))
        self.assertAlmostEqual(straight[-1][0], 1.2)
        self.assertTrue(all(y_m == 0.0 for _, y_m in straight))
        self.assertAlmostEqual(left[-1][0], right[-1][0])
        self.assertAlmostEqual(left[-1][1], -right[-1][1])

    def test_swept_union_retains_cost_evidence(self):
        summary = swept_path_costs(
            self.costmap({(9, 38): 50}),
            -0.35,
            self.config,
        )

        self.assertTrue(summary.valid)
        self.assertEqual(summary.maximum_cost, 50)
        self.assertIsNotNone(summary.nearest_slow_distance_m)
        self.assertIsNone(summary.nearest_stop_distance_m)

    def test_unknown_and_out_of_bounds_paths_fail_closed(self):
        unknown = swept_path_costs(
            self.costmap({(5, 40): -1}),
            0.0,
            self.config,
        )
        outside = swept_path_costs(
            self.costmap({}, width=5),
            0.0,
            self.config,
        )

        self.assertFalse(unknown.valid)
        self.assertEqual(unknown.failure_reason, "unknown_nav2_cost")
        self.assertFalse(outside.valid)
        self.assertEqual(
            outside.failure_reason,
            "trajectory_outside_costmap",
        )


if __name__ == "__main__":
    unittest.main()
