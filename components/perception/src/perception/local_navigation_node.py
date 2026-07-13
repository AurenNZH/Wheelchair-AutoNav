"""ROS2 node for LiDAR-based local costmap and command proposals."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from perception.local_navigation import (
    LocalCostmapConfig,
    TrajectoryConfig,
    choose_command,
    make_local_costmap,
)
from perception.point_cloud import read_xyz_points, transform_points


class LocalNavigationNode(Node):
    """Publish debug local navigation outputs without controlling hardware."""

    def __init__(self) -> None:
        super().__init__("local_navigation")

        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("costmap_topic", "/local_costmap")
        self.declare_parameter("proposed_cmd_topic", "/proposed_cmd_vel")
        self.declare_parameter("selected_path_topic", "/local_planner/selected_path")
        self.declare_parameter("size_m", 8.0)
        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("min_height_m", -0.25)
        self.declare_parameter("max_height_m", 1.5)
        self.declare_parameter("inflation_radius_m", 0.45)
        self.declare_parameter("linear_speed_mps", 0.25)
        self.declare_parameter("footprint_radius_m", 0.45)
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            10,
        )
        self._cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter("proposed_cmd_topic").value,
            10,
        )
        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter("selected_path_topic").value,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self._on_lidar,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Local navigation debug node started.")

    def _on_lidar(self, msg: PointCloud2) -> None:
        timestamp_error = cloud_timestamp_error(
            now_ns=self.get_clock().now().nanoseconds,
            stamp_ns=Time.from_msg(msg.header.stamp).nanoseconds,
            max_age_s=float(self.get_parameter("max_cloud_age_s").value),
            max_future_offset_s=float(
                self.get_parameter("max_future_offset_s").value
            ),
        )
        if timestamp_error is not None:
            self._publish_stop(timestamp_error)
            return

        target_frame = self.get_parameter("target_frame").value
        try:
            points = np.asarray(list(read_xyz_points(msg)), dtype=np.float32)
        except ValueError as exc:
            self.get_logger().warn(
                "Invalid LiDAR cloud: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._publish_stop("invalid_lidar")
            return
        if points.size == 0:
            self._publish_stop("empty_lidar")
            return

        if msg.header.frame_id != target_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    "No timestamped TF from %s to %s: %s"
                    % (msg.header.frame_id, target_frame, exc),
                    throttle_duration_sec=5.0,
                )
                self._publish_stop("missing_tf")
                return
            points = transform_points(points, transform)

        map_config = self._map_config()
        trajectory_config = self._trajectory_config()
        costmap = make_local_costmap(points, map_config)
        command, candidates = choose_command(costmap, map_config, trajectory_config)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = target_frame
        self._costmap_pub.publish(build_occupancy_grid(header, costmap, map_config))
        self._cmd_pub.publish(command_to_twist(command))

        safe_candidates = [candidate for candidate in candidates if candidate.safe]
        if safe_candidates:
            best = max(safe_candidates, key=lambda candidate: candidate.score)
            self._path_pub.publish(build_path(header, best.points_xy))

    def _map_config(self) -> LocalCostmapConfig:
        return LocalCostmapConfig(
            size_m=float(self.get_parameter("size_m").value),
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_height_m=float(self.get_parameter("min_height_m").value),
            max_height_m=float(self.get_parameter("max_height_m").value),
            inflation_radius_m=float(self.get_parameter("inflation_radius_m").value),
        )

    def _trajectory_config(self) -> TrajectoryConfig:
        return TrajectoryConfig(
            linear_speed_mps=float(self.get_parameter("linear_speed_mps").value),
            footprint_radius_m=float(self.get_parameter("footprint_radius_m").value),
        )

    def _publish_stop(self, reason: str) -> None:
        self.get_logger().warn(
            "Publishing zero proposed command: %s" % reason,
            throttle_duration_sec=5.0,
        )
        self._cmd_pub.publish(Twist())


def cloud_timestamp_error(
    now_ns: int,
    stamp_ns: int,
    max_age_s: float,
    max_future_offset_s: float,
) -> str | None:
    """Return a fail-safe reason for an unusable cloud timestamp."""

    if stamp_ns <= 0:
        return "invalid_lidar_timestamp"
    age_ns = int(now_ns) - int(stamp_ns)
    if age_ns > int(max_age_s * 1e9):
        return "stale_lidar"
    if age_ns < -int(max_future_offset_s * 1e9):
        return "future_lidar"
    return None


def build_occupancy_grid(
    header: Header,
    costmap: np.ndarray,
    config: LocalCostmapConfig,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = header
    msg.info.resolution = float(config.resolution_m)
    msg.info.width = int(costmap.shape[1])
    msg.info.height = int(costmap.shape[0])
    msg.info.origin.position.x = -config.size_m / 2.0
    msg.info.origin.position.y = -config.size_m / 2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(value) for value in costmap.flatten()]
    return msg


def command_to_twist(command) -> Twist:
    msg = Twist()
    if command.safe:
        msg.linear.x = float(command.linear_x_mps)
        msg.angular.z = float(command.angular_z_radps)
    return msg


def build_path(header: Header, points_xy: np.ndarray) -> Path:
    from geometry_msgs.msg import PoseStamped

    path = Path()
    path.header = header
    for x_m, y_m in points_xy:
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x_m)
        pose.pose.position.y = float(y_m)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def main() -> int:
    rclpy.init()
    node = LocalNavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
