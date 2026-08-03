"""Restamp recorded obstacle grids for wall-time ROS 2 Foxy playback."""

from __future__ import annotations

from copy import deepcopy

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node


def restamped_grid(msg: OccupancyGrid, stamp) -> OccupancyGrid:
    """Copy an occupancy grid and replace only its header timestamp."""

    output = deepcopy(msg)
    output.header.stamp = stamp
    return output


def validate_topic_separation(topics: tuple[str, str, str, str]) -> None:
    """Prevent replay inputs from feeding either restamped output recursively."""

    if any(not topic for topic in topics):
        raise ValueError("replay map topics must not be empty")
    if len(set(topics)) != len(topics):
        raise ValueError("replay input and output map topics must all differ")


class ReplayMapRestamperNode(Node):
    """Bridge recorded maps into the production topics without simulating sensors."""

    def __init__(self) -> None:
        super().__init__("replay_map_restamper")
        self.declare_parameter(
            "input_front_topic", "/replay/front_costmap"
        )
        self.declare_parameter(
            "input_full_topic", "/replay/local_obstacles"
        )
        self.declare_parameter("output_front_topic", "/front_costmap")
        self.declare_parameter("output_full_topic", "/local_obstacles")

        topics = (
            str(self.get_parameter("input_front_topic").value),
            str(self.get_parameter("input_full_topic").value),
            str(self.get_parameter("output_front_topic").value),
            str(self.get_parameter("output_full_topic").value),
        )
        validate_topic_separation(topics)
        input_front, input_full, output_front, output_full = topics

        self._front_pub = self.create_publisher(
            OccupancyGrid, output_front, 10
        )
        self._full_pub = self.create_publisher(OccupancyGrid, output_full, 10)
        self.create_subscription(
            OccupancyGrid, input_front, self._on_front, 10
        )
        self.create_subscription(OccupancyGrid, input_full, self._on_full, 10)
        self.get_logger().warn(
            "Replay map restamping enabled for Foxy wall-time playback. "
            "Original map timestamps are intentionally not used for latency tests."
        )

    def _on_front(self, msg: OccupancyGrid) -> None:
        self._front_pub.publish(
            restamped_grid(msg, self.get_clock().now().to_msg())
        )

    def _on_full(self, msg: OccupancyGrid) -> None:
        self._full_pub.publish(
            restamped_grid(msg, self.get_clock().now().to_msg())
        )


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = ReplayMapRestamperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
