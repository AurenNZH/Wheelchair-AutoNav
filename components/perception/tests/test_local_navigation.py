import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.local_navigation import (  # noqa: E402
    LocalCostmapConfig,
    SocialZone,
    TrajectoryConfig,
    choose_command,
    make_local_costmap,
    world_to_cell,
)
from perception.local_navigation_node import cloud_timestamp_error  # noqa: E402


class LocalNavigationTests(unittest.TestCase):
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

    def test_costmap_marks_and_inflates_lidar_obstacle(self):
        config = LocalCostmapConfig(size_m=4.0, resolution_m=0.2, inflation_radius_m=0.2)
        points = np.array([[1.0, 0.0, 0.4]], dtype=np.float32)

        costmap = make_local_costmap(points, config)
        cell = world_to_cell(1.0, 0.0, config, costmap.shape[0])

        self.assertIsNotNone(cell)
        self.assertEqual(costmap[cell[1], cell[0]], 100)

    def test_planner_stops_when_forward_path_is_blocked(self):
        map_config = LocalCostmapConfig(size_m=4.0, resolution_m=0.1, inflation_radius_m=0.2)
        trajectory_config = TrajectoryConfig(
            horizon_s=1.0,
            dt_s=0.1,
            linear_speed_mps=0.5,
            angular_samples_radps=(0.0,),
            footprint_radius_m=0.35,
        )
        points = np.array([[0.4, 0.0, 0.3]], dtype=np.float32)
        costmap = make_local_costmap(points, map_config)

        command, _ = choose_command(costmap, map_config, trajectory_config)

        self.assertFalse(command.safe)
        self.assertEqual(command.linear_x_mps, 0.0)

    def test_planner_selects_forward_when_clear(self):
        map_config = LocalCostmapConfig(size_m=4.0, resolution_m=0.1, inflation_radius_m=0.2)
        trajectory_config = TrajectoryConfig(
            horizon_s=1.0,
            dt_s=0.1,
            linear_speed_mps=0.4,
            angular_samples_radps=(-0.5, 0.0, 0.5),
            footprint_radius_m=0.25,
        )
        costmap = make_local_costmap(np.empty((0, 3), dtype=np.float32), map_config)

        command, _ = choose_command(costmap, map_config, trajectory_config)

        self.assertTrue(command.safe)
        self.assertAlmostEqual(command.angular_z_radps, 0.0)
        self.assertAlmostEqual(command.linear_x_mps, 0.4)

    def test_social_zone_adds_cost_without_lidar_obstacle(self):
        config = LocalCostmapConfig(size_m=4.0, resolution_m=0.2)

        costmap = make_local_costmap(
            np.empty((0, 3), dtype=np.float32),
            config,
            social_zones=(SocialZone(1.0, 0.0, 0.4, cost=80),),
        )
        cell = world_to_cell(1.0, 0.0, config, costmap.shape[0])

        self.assertIsNotNone(cell)
        self.assertEqual(costmap[cell[1], cell[0]], 80)


if __name__ == "__main__":
    unittest.main()
