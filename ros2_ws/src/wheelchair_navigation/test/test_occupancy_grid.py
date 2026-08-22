import unittest

import numpy as np
from std_msgs.msg import Header

from wheelchair_navigation.costmap import (
    FrontCostmapConfig,
    LocalCostmapConfig,
)
from wheelchair_navigation.occupancy_grid import (
    build_front_occupancy_grid,
    build_occupancy_grid,
)


class OccupancyGridTests(unittest.TestCase):
    def test_local_grid_preserves_header_geometry_and_row_major_data(self):
        header = Header()
        header.frame_id = "base_link"
        header.stamp.sec = 12
        header.stamp.nanosec = 34
        costmap = np.array([[0, 100], [-1, 0]], dtype=np.int8)

        message = build_occupancy_grid(
            header,
            costmap,
            LocalCostmapConfig(size_m=2.0, resolution_m=1.0),
        )

        self.assertEqual(message.header, header)
        self.assertEqual(message.info.map_load_time, header.stamp)
        self.assertEqual(message.info.resolution, 1.0)
        self.assertEqual((message.info.width, message.info.height), (2, 2))
        self.assertEqual(message.info.origin.position.x, -1.0)
        self.assertEqual(message.info.origin.position.y, -1.0)
        self.assertEqual(message.info.origin.orientation.w, 1.0)
        self.assertEqual(list(message.data), [0, 100, -1, 0])

    def test_front_grid_is_x_forward_and_centered_laterally(self):
        header = Header()
        costmap = np.zeros((4, 3), dtype=np.int8)

        message = build_front_occupancy_grid(
            header,
            costmap,
            FrontCostmapConfig(
                length_m=1.5,
                width_m=2.0,
                resolution_m=0.5,
            ),
        )

        self.assertEqual(message.info.origin.position.x, 0.0)
        self.assertEqual(message.info.origin.position.y, -1.0)
        self.assertEqual((message.info.width, message.info.height), (3, 4))


if __name__ == "__main__":
    unittest.main()
