"""ROS OccupancyGrid construction for local navigation maps."""

from __future__ import annotations

import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from wheelchair_navigation.costmap import (
    FrontCostmapConfig,
    LocalCostmapConfig,
    grid_origin_m,
)


def build_occupancy_grid(
    header: Header,
    costmap: np.ndarray,
    config: LocalCostmapConfig | FrontCostmapConfig,
    *,
    origin_x_m: float | None = None,
    origin_y_m: float | None = None,
) -> OccupancyGrid:
    """Build an OccupancyGrid with explicit metric geometry."""

    msg = OccupancyGrid()
    msg.header = header
    msg.info.map_load_time = header.stamp
    msg.info.resolution = float(config.resolution_m)
    msg.info.width = int(costmap.shape[1])
    msg.info.height = int(costmap.shape[0])
    default_origin = 0.0
    if isinstance(config, LocalCostmapConfig):
        default_origin = grid_origin_m(config)
    msg.info.origin.position.x = (
        default_origin if origin_x_m is None else float(origin_x_m)
    )
    msg.info.origin.position.y = (
        default_origin if origin_y_m is None else float(origin_y_m)
    )
    msg.info.origin.orientation.w = 1.0
    msg.data = costmap.reshape(-1).tolist()
    return msg


def build_front_occupancy_grid(
    header: Header,
    costmap: np.ndarray,
    config: FrontCostmapConfig,
) -> OccupancyGrid:
    """Build a front-grid message with the robot-forward origin."""

    return build_occupancy_grid(
        header,
        costmap,
        config,
        origin_x_m=0.0,
        origin_y_m=-(costmap.shape[0] * config.resolution_m) / 2.0,
    )


__all__ = ["build_front_occupancy_grid", "build_occupancy_grid"]
