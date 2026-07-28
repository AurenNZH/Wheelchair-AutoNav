"""ROS2 node for the non-actuating LiDAR local-mapping baseline."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from wheelchair_navigation.local_navigation import (
    CostmapStats,
    FrontCostmapConfig,
    LocalCostmapConfig,
    grid_origin_m,
    make_local_and_front_costmaps,
    parse_self_filter_boxes,
)
from wheelchair_navigation.point_cloud import point_cloud_to_arrays, transform_points


class LocalNavigationNode(Node):
    """Publish raw and derived obstacle grids without producing motion commands."""

    def __init__(self) -> None:
        super().__init__("local_costmap")

        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("raw_obstacles_topic", "/local_obstacles")
        self.declare_parameter("costmap_topic", "/local_costmap")
        self.declare_parameter("front_costmap_topic", "/front_costmap")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("size_m", 8.0)
        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("min_height_m", 0.05)
        self.declare_parameter("max_height_m", 1.5)
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("max_range_m", 4.00)
        self.declare_parameter("inflation_radius_m", 0.0)
        self.declare_parameter("front_length_m", 4.0)
        self.declare_parameter("front_width_m", 8.0)
        self.declare_parameter("front_resolution_m", 0.1)
        self.declare_parameter("front_fov_deg", 180.0)
        self.declare_parameter("front_inflation_radius_m", 0.0)
        self.declare_parameter("self_filter_boxes", [])
        self.declare_parameter("self_filter_padding_m", 0.02)
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)
        self.declare_parameter("processing_warn_ms", 100.0)
        self.declare_parameter("cloud_age_warn_ms", 150.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._raw_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("raw_obstacles_topic").value, 1
        )
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("costmap_topic").value, 1
        )
        self._front_costmap_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("front_costmap_topic").value, 1
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
        )
        lidar_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("lidar_topic").value,
            self._on_lidar,
            lidar_qos,
        )

        self._processed_clouds = 0
        self._rejected_clouds = 0
        self._started_monotonic = time.monotonic()
        self._last_cloud_stamp_ns = None
        self._last_arrival_monotonic = None
        self.get_logger().info(
            "Mapping-only local costmap started; no motion commands are published."
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        arrival_monotonic = time.monotonic()
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        source_period_ms = self._period_ms(stamp_ns, self._last_cloud_stamp_ns)
        arrival_period_ms = self._period_ms(
            arrival_monotonic, self._last_arrival_monotonic, scale=1000.0
        )
        self._last_cloud_stamp_ns = stamp_ns
        self._last_arrival_monotonic = arrival_monotonic
        stage_ms = {}
        timestamp_error = cloud_timestamp_error(
            now_ns=self.get_clock().now().nanoseconds,
            stamp_ns=stamp_ns,
            max_age_s=float(self.get_parameter("max_cloud_age_s").value),
            max_future_offset_s=float(
                self.get_parameter("max_future_offset_s").value
            ),
        )
        if timestamp_error is not None:
            self._reject_cloud(timestamp_error, stamp_ns, started)
            return

        try:
            decode_started = time.perf_counter()
            cloud = point_cloud_to_arrays(msg)
            stage_ms["decode_ms"] = (
                time.perf_counter() - decode_started
            ) * 1000.0
        except ValueError as exc:
            self.get_logger().warn(
                "Invalid LiDAR cloud: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._reject_cloud("invalid_lidar", stamp_ns, started)
            return
        if cloud.xyz.size == 0:
            self._reject_cloud("empty_lidar", stamp_ns, started)
            return

        target_frame = str(self.get_parameter("target_frame").value)
        points_base = cloud.xyz
        if msg.header.frame_id != target_frame:
            try:
                transform_started = time.perf_counter()
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
                self._reject_cloud("missing_tf", stamp_ns, started)
                return
            points_base = transform_points(points_base, transform)
            stage_ms["transform_ms"] = (
                time.perf_counter() - transform_started
            ) * 1000.0
        else:
            stage_ms["transform_ms"] = 0.0

        try:
            mapping_started = time.perf_counter()
            map_config = self._map_config()
            front_config = self._front_map_config()
            boxes = parse_self_filter_boxes(
                self.get_parameter("self_filter_boxes").value,
                float(self.get_parameter("self_filter_padding_m").value),
            )
            raw, costmap, front_costmap, stats = make_local_and_front_costmaps(
                points_base, map_config, front_config, boxes
            )
            stage_ms["mapping_ms"] = (
                time.perf_counter() - mapping_started
            ) * 1000.0
        except ValueError as exc:
            self.get_logger().error(
                "Invalid local-mapping configuration: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._reject_cloud("invalid_configuration", stamp_ns, started)
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = target_frame
        publish_started = time.perf_counter()
        self._raw_pub.publish(build_occupancy_grid(header, raw, map_config))
        self._costmap_pub.publish(build_occupancy_grid(header, costmap, map_config))
        self._front_costmap_pub.publish(
            build_occupancy_grid(
                header,
                front_costmap,
                front_config,
                origin_x_m=0.0,
                origin_y_m=-(front_costmap.shape[0] * front_config.resolution_m)
                / 2.0,
            )
        )
        stage_ms["publish_ms"] = (
            time.perf_counter() - publish_started
        ) * 1000.0

        self._processed_clouds += 1
        processing_ms = (time.perf_counter() - started) * 1000.0
        cloud_age_ms = max(
            0.0,
            (self.get_clock().now().nanoseconds - stamp_ns) / 1e6,
        )
        self._publish_diagnostics(
            "ok",
            processing_ms,
            cloud_age_ms,
            stats=stats,
            stage_ms=stage_ms,
            source_period_ms=source_period_ms,
            arrival_period_ms=arrival_period_ms,
        )

    def _map_config(self) -> LocalCostmapConfig:
        return LocalCostmapConfig(
            size_m=float(self.get_parameter("size_m").value),
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_height_m=float(self.get_parameter("min_height_m").value),
            max_height_m=float(self.get_parameter("max_height_m").value),
            min_range_m=float(self.get_parameter("min_range_m").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
            inflation_radius_m=float(
                self.get_parameter("inflation_radius_m").value
            ),
        )

    def _front_map_config(self) -> FrontCostmapConfig:
        return FrontCostmapConfig(
            length_m=float(self.get_parameter("front_length_m").value),
            width_m=float(self.get_parameter("front_width_m").value),
            resolution_m=float(self.get_parameter("front_resolution_m").value),
            fov_deg=float(self.get_parameter("front_fov_deg").value),
            inflation_radius_m=float(
                self.get_parameter("front_inflation_radius_m").value
            ),
        )

    @staticmethod
    def _period_ms(current, previous, scale: float = 1e-6) -> float:
        if previous is None or current <= previous:
            return 0.0
        return float(current - previous) * scale

    def _reject_cloud(self, reason: str, stamp_ns: int, started: float) -> None:
        self._rejected_clouds += 1
        processing_ms = (time.perf_counter() - started) * 1000.0
        cloud_age_ms = (
            max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e6)
            if stamp_ns > 0
            else 0.0
        )
        self._publish_diagnostics(reason, processing_ms, cloud_age_ms)

    def _publish_diagnostics(
        self,
        reason: str,
        processing_ms: float,
        cloud_age_ms: float,
        stats: CostmapStats | None = None,
        stage_ms: dict[str, float] | None = None,
        source_period_ms: float = 0.0,
        arrival_period_ms: float = 0.0,
    ) -> None:
        processing_warn = float(self.get_parameter("processing_warn_ms").value)
        age_warn = float(self.get_parameter("cloud_age_warn_ms").value)
        if reason != "ok":
            level = DiagnosticStatus.ERROR
            message = reason
        elif processing_ms > processing_warn or cloud_age_ms > age_warn:
            level = DiagnosticStatus.WARN
            message = "mapping_latency_exceeded"
        else:
            level = DiagnosticStatus.OK
            message = "mapping_current"

        values = {
            "processing_ms": "%.3f" % processing_ms,
            "cloud_age_ms": "%.3f" % cloud_age_ms,
            "processed_clouds": str(self._processed_clouds),
            "rejected_clouds": str(self._rejected_clouds),
            "source_period_ms": "%.3f" % source_period_ms,
            "arrival_period_ms": "%.3f" % arrival_period_ms,
        }
        for key, value in (stage_ms or {}).items():
            values[key] = "%.3f" % value
        if stats is not None:
            values.update(
                {
                    "input_points": str(stats.input_points),
                    "finite_points": str(stats.finite_points),
                    "height_range_points": str(stats.height_range_points),
                    "self_filtered_points": str(stats.self_filtered_points),
                    "accepted_points": str(stats.accepted_points),
                    "occupied_cells": str(stats.occupied_cells),
                    "front_points": str(stats.front_points),
                    "front_occupied_cells": str(stats.front_occupied_cells),
                }
            )
        elapsed_s = max(time.monotonic() - self._started_monotonic, 1e-6)
        effective_rate_hz = self._processed_clouds / elapsed_s
        values["effective_rate_hz"] = "%.3f" % effective_rate_hz

        status = DiagnosticStatus()
        status.level = level
        status.name = "wheelchair_navigation/local_costmap"
        status.hardware_id = "robosense_airy"
        status.message = message
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)
        if stats is not None:
            self.get_logger().info(
                "mapping reason=%s processing_ms=%.3f cloud_age_ms=%.3f "
                "rate_hz=%.3f input_points=%d accepted_points=%d "
                "occupied_cells=%d front_points=%d front_occupied_cells=%d "
                "self_filtered_points=%d"
                % (
                    reason,
                    processing_ms,
                    cloud_age_ms,
                    effective_rate_hz,
                    stats.input_points,
                    stats.accepted_points,
                    stats.occupied_cells,
                    stats.front_points,
                    stats.front_occupied_cells,
                    stats.self_filtered_points,
                ),
                throttle_duration_sec=2.0,
            )


def cloud_timestamp_error(
    now_ns: int,
    stamp_ns: int,
    max_age_s: float,
    max_future_offset_s: float,
) -> str | None:
    """Return a diagnostic reason for an unusable cloud timestamp."""

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
    config: LocalCostmapConfig | FrontCostmapConfig,
    *,
    origin_x_m: float | None = None,
    origin_y_m: float | None = None,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = header
    msg.info.map_load_time = header.stamp
    msg.info.resolution = float(config.resolution_m)
    msg.info.width = int(costmap.shape[1])
    msg.info.height = int(costmap.shape[0])
    default_origin = (
        grid_origin_m(config) if isinstance(config, LocalCostmapConfig) else 0.0
    )
    msg.info.origin.position.x = (
        default_origin if origin_x_m is None else float(origin_x_m)
    )
    msg.info.origin.position.y = (
        default_origin if origin_y_m is None else float(origin_y_m)
    )
    msg.info.origin.orientation.w = 1.0
    msg.data = [int(value) for value in costmap.reshape(-1)]
    return msg


def main() -> int:
    rclpy.init()
    node = LocalNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # Launch/timeout can deliver a second SIGINT while ROS entities are
            # being destroyed. The process is already stopping at this point.
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
