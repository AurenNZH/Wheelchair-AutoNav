"""ROS2 node for non-actuating AIRY obstacle mapping."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from wheelchair_navigation.artifact_filter import (
    ArtifactCellSupportStats,
    ArtifactFilterStats,
    filter_artifact_points,
    parse_artifact_grid_cells,
    parse_artifact_grid_halo_spans,
    validate_artifact_filter_frame,
)
from wheelchair_navigation.artifact_markers import (
    build_artifact_grid_markers,
    build_artifact_threshold_cell_markers,
)
from wheelchair_navigation.costmap import (
    CostmapStats,
    FrontCostmapConfig,
    LocalCostmapConfig,
    make_costmap_stats,
    make_front_grid,
    make_full_raw_grid,
    obstacle_point_mask,
    parse_self_filter_boxes,
    select_front_points,
    validate_mapping_configs,
)
from wheelchair_navigation.mapping_diagnostics import (
    MappingDiagnosticSnapshot,
    MappingMetrics,
    artifact_shadow_error_reason,
    build_mapping_diagnostic_status,
)
from wheelchair_navigation.occupancy_grid import (
    build_front_occupancy_grid,
    build_occupancy_grid,
)
from wheelchair_navigation.point_cloud import (
    point_cloud_to_arrays,
    transform_points,
    xyz_to_point_cloud,
)
from wheelchair_navigation.timing import (
    CloudTimingTracker,
    cloud_age_ms,
    cloud_timestamp_error,
)


class LocalNavigationNode(Node):
    """Publish raw full and robot-forward grids without commanding motion."""

    def __init__(self) -> None:
        super().__init__("local_costmap")

        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("raw_obstacles_topic", "/local_obstacles")
        self.declare_parameter("front_costmap_topic", "/front_costmap")
        self.declare_parameter(
            "artifact_filtered_front_topic",
            "/front_costmap_artifact_filtered",
        )
        self.declare_parameter(
            "artifact_rejected_points_topic",
            "/artifact_filter/rejected_points",
        )
        self.declare_parameter(
            "artifact_low_support_points_topic",
            "/artifact_filter/low_support_points",
        )
        self.declare_parameter(
            "artifact_masks_topic", "/artifact_filter/masks"
        )
        self.declare_parameter(
            "artifact_threshold_cells_topic",
            "/artifact_filter/threshold_cells",
        )
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("size_m", 8.0)
        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("min_height_m", 0.05)
        self.declare_parameter("max_height_m", 1.5)
        self.declare_parameter("min_range_m", 0.30)
        self.declare_parameter("max_range_m", 4.00)
        self.declare_parameter("front_length_m", 4.0)
        self.declare_parameter("front_width_m", 8.0)
        self.declare_parameter("front_resolution_m", 0.1)
        self.declare_parameter("front_fov_deg", 180.0)
        self.declare_parameter("self_filter_boxes", [])
        self.declare_parameter("self_filter_padding_m", 0.0)
        self.declare_parameter("artifact_filter_frame", "base_link")
        self.declare_parameter("artifact_grid_mask_cells", [])
        self.declare_parameter("artifact_grid_halo_spans", [])
        self.declare_parameter("artifact_global_min_points_per_cell", 1)
        self.declare_parameter("artifact_min_points_per_cell", 2)
        self.declare_parameter("publish_artifact_shadow", True)
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)
        self.declare_parameter("validate_cloud_timestamps", True)
        self.declare_parameter("restamp_output_with_node_time", False)
        self.declare_parameter("processing_warn_ms", 100.0)
        self.declare_parameter("cloud_age_warn_ms", 150.0)
        self.declare_parameter("latency_window_samples", 120)
        self.declare_parameter("lag_spike_ms", 150.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._raw_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("raw_obstacles_topic").value),
            1,
        )
        self._front_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("front_costmap_topic").value),
            1,
        )
        self._artifact_front_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("artifact_filtered_front_topic").value),
            1,
        )
        self._artifact_rejected_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("artifact_rejected_points_topic").value),
            1,
        )
        self._artifact_low_support_pub = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter(
                    "artifact_low_support_points_topic"
                ).value
            ),
            1,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._artifact_masks_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("artifact_masks_topic").value),
            marker_qos,
        )
        self._artifact_threshold_cells_pub = self.create_publisher(
            MarkerArray,
            str(
                self.get_parameter(
                    "artifact_threshold_cells_topic"
                ).value
            ),
            marker_qos,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        lidar_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar_topic").value),
            self._on_lidar,
            lidar_qos,
        )

        self._processed_clouds = 0
        self._rejected_clouds = 0
        self._timing = CloudTimingTracker()
        self._metrics = MappingMetrics(
            int(self.get_parameter("latency_window_samples").value)
        )
        self.get_logger().info(
            "Mapping-only AIRY mapper started; publishing /local_obstacles "
            "and /front_costmap. No motion commands are published."
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        arrival_monotonic = time.monotonic()
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        periods = self._timing.record(stamp_ns, arrival_monotonic)
        source_period_ms = periods.source_ms
        arrival_period_ms = periods.arrival_ms

        validate_timestamps = bool(
            self.get_parameter("validate_cloud_timestamps").value
        )
        timestamp_error = None
        if validate_timestamps:
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

        stage_ms: dict[str, float] = {}
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
            map_config = self._map_config()
            front_config = self._front_map_config()
            validate_mapping_configs(map_config, front_config)
            boxes = parse_self_filter_boxes(
                self.get_parameter("self_filter_boxes").value,
                float(self.get_parameter("self_filter_padding_m").value),
            )

            filter_started = time.perf_counter()
            accepted_mask, counts = obstacle_point_mask(
                points_base, map_config, boxes
            )
            accepted = points_base[accepted_mask]
            stage_ms["filter_ms"] = (
                time.perf_counter() - filter_started
            ) * 1000.0

            raw_started = time.perf_counter()
            raw = make_full_raw_grid(accepted, map_config)
            stage_ms["raw_raster_ms"] = (
                time.perf_counter() - raw_started
            ) * 1000.0

            front_select_started = time.perf_counter()
            front_points = select_front_points(accepted, front_config)
            stage_ms["front_select_ms"] = (
                time.perf_counter() - front_select_started
            ) * 1000.0
            front_raster_started = time.perf_counter()
            front = make_front_grid(front_points, front_config)
            stage_ms["front_raster_ms"] = (
                time.perf_counter() - front_raster_started
            ) * 1000.0
            stage_ms["mapping_ms"] = sum(
                stage_ms[key]
                for key in (
                    "filter_ms",
                    "raw_raster_ms",
                    "front_select_ms",
                    "front_raster_ms",
                )
            )
            stats = make_costmap_stats(
                counts,
                accepted,
                raw,
                map_config,
                front_points=front_points,
                front=front,
                front_config=front_config,
            )
        except ValueError as exc:
            self.get_logger().error(
                "Invalid local-mapping configuration: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._reject_cloud("invalid_configuration", stamp_ns, started)
            return

        header = Header()
        header.stamp = (
            self.get_clock().now().to_msg()
            if bool(
                self.get_parameter(
                    "restamp_output_with_node_time"
                ).value
            )
            else msg.header.stamp
        )
        header.frame_id = target_frame
        publish_started = time.perf_counter()
        self._raw_pub.publish(build_occupancy_grid(header, raw, map_config))
        stage_ms["publish_raw_ms"] = (
            time.perf_counter() - publish_started
        ) * 1000.0
        publish_started = time.perf_counter()
        self._front_pub.publish(
            build_front_occupancy_grid(header, front, front_config)
        )
        stage_ms["publish_front_ms"] = (
            time.perf_counter() - publish_started
        ) * 1000.0
        stage_ms["publish_ms"] = (
            stage_ms["publish_raw_ms"] + stage_ms["publish_front_ms"]
        )

        artifact_stats = None
        artifact_support_stats = None
        artifact_front_cells = None
        diagnostic_reason = "ok"
        if bool(self.get_parameter("publish_artifact_shadow").value):
            artifact_started = time.perf_counter()
            try:
                artifact_frame = str(
                    self.get_parameter("artifact_filter_frame").value
                )
                cells = parse_artifact_grid_cells(
                    self.get_parameter("artifact_grid_mask_cells").value
                )
                halo_spans = parse_artifact_grid_halo_spans(
                    self.get_parameter("artifact_grid_halo_spans").value
                )
                validate_artifact_filter_frame(
                    artifact_frame, target_frame
                )
                filter_result = filter_artifact_points(
                    points_base,
                    accepted_mask,
                    boxes,
                    cells,
                    halo_spans,
                    front_config,
                    min_points_per_cell=self.get_parameter(
                        "artifact_min_points_per_cell"
                    ).value,
                    global_min_points_per_cell=self.get_parameter(
                        "artifact_global_min_points_per_cell"
                    ).value,
                )
                artifact_stats = filter_result.artifact_stats
                support_result = filter_result.support
                artifact_support_stats = support_result.stats
                shadow_front_points = points_base[
                    support_result.shadow_mask
                    & filter_result.front_valid_mask
                ]
                artifact_front = make_front_grid(
                    shadow_front_points, front_config
                )
                rejected_sensor = cloud.xyz[
                    filter_result.artifact_rejected_mask
                ]
                low_support_sensor = cloud.xyz[
                    support_result.low_support_mask
                ]
                artifact_front_cells = int(
                    np.count_nonzero(
                        artifact_front == front_config.occupied_cost
                    )
                )
                stage_ms["artifact_filter_ms"] = (
                    time.perf_counter() - artifact_started
                ) * 1000.0

                publish_started = time.perf_counter()
                self._artifact_front_pub.publish(
                    build_front_occupancy_grid(
                        header, artifact_front, front_config
                    )
                )
                self._artifact_rejected_pub.publish(
                    xyz_to_point_cloud(rejected_sensor, msg.header)
                )
                self._artifact_low_support_pub.publish(
                    xyz_to_point_cloud(low_support_sensor, msg.header)
                )
                marker_header = Header()
                marker_header.stamp = msg.header.stamp
                marker_header.frame_id = artifact_frame
                self._artifact_masks_pub.publish(
                    build_artifact_grid_markers(
                        marker_header,
                        cells,
                        halo_spans,
                        front_config,
                    )
                )
                threshold_header = Header()
                threshold_header.stamp = msg.header.stamp
                threshold_header.frame_id = target_frame
                self._artifact_threshold_cells_pub.publish(
                    build_artifact_threshold_cell_markers(
                        threshold_header,
                        front_config,
                        support_result.candidate_cell_ids,
                        support_result.low_support_cell_ids,
                    )
                )
                stage_ms["publish_artifact_ms"] = (
                    time.perf_counter() - publish_started
                ) * 1000.0
                stage_ms["publish_ms"] += stage_ms["publish_artifact_ms"]
            except (TypeError, ValueError) as exc:
                diagnostic_reason = artifact_shadow_error_reason(exc)
                stage_ms["artifact_filter_ms"] = (
                    time.perf_counter() - artifact_started
                ) * 1000.0
                self.get_logger().error(
                    "Artifact shadow suppressed; raw maps remain active: %s"
                    % exc,
                    throttle_duration_sec=5.0,
                )

        self._processed_clouds += 1
        processing_ms = (time.perf_counter() - started) * 1000.0
        current_cloud_age_ms = (
            cloud_age_ms(self.get_clock().now().nanoseconds, stamp_ns)
            if validate_timestamps
            else 0.0
        )
        self._metrics.record(
            processing_ms,
            arrival_monotonic,
            cloud_age_ms=current_cloud_age_ms,
            mapping_ms=stage_ms["mapping_ms"],
        )
        self._publish_diagnostics(
            diagnostic_reason,
            processing_ms,
            current_cloud_age_ms,
            stats=stats,
            stage_ms=stage_ms,
            source_period_ms=source_period_ms,
            arrival_period_ms=arrival_period_ms,
            artifact_stats=artifact_stats,
            artifact_support_stats=artifact_support_stats,
            artifact_front_cells=artifact_front_cells,
        )

    def _map_config(self) -> LocalCostmapConfig:
        return LocalCostmapConfig(
            size_m=float(self.get_parameter("size_m").value),
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_height_m=float(self.get_parameter("min_height_m").value),
            max_height_m=float(self.get_parameter("max_height_m").value),
            min_range_m=float(self.get_parameter("min_range_m").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
        )

    def _front_map_config(self) -> FrontCostmapConfig:
        return FrontCostmapConfig(
            length_m=float(self.get_parameter("front_length_m").value),
            width_m=float(self.get_parameter("front_width_m").value),
            resolution_m=float(
                self.get_parameter("front_resolution_m").value
            ),
            fov_deg=float(self.get_parameter("front_fov_deg").value),
        )

    def _reject_cloud(
        self, reason: str, stamp_ns: int, started: float
    ) -> None:
        self._rejected_clouds += 1
        processing_ms = (time.perf_counter() - started) * 1000.0
        current_cloud_age_ms = cloud_age_ms(
            self.get_clock().now().nanoseconds, stamp_ns
        )
        self._publish_diagnostics(reason, processing_ms, current_cloud_age_ms)

    def _publish_diagnostics(
        self,
        reason: str,
        processing_ms: float,
        cloud_age_ms: float,
        stats: CostmapStats | None = None,
        stage_ms: dict[str, float] | None = None,
        source_period_ms: float = 0.0,
        arrival_period_ms: float = 0.0,
        artifact_stats: ArtifactFilterStats | None = None,
        artifact_support_stats: ArtifactCellSupportStats | None = None,
        artifact_front_cells: int | None = None,
    ) -> None:
        snapshot = MappingDiagnosticSnapshot(
            reason=reason,
            processing_ms=processing_ms,
            cloud_age_ms=cloud_age_ms,
            processing_warn_ms=float(
                self.get_parameter("processing_warn_ms").value
            ),
            cloud_age_warn_ms=float(
                self.get_parameter("cloud_age_warn_ms").value
            ),
            processed_clouds=self._processed_clouds,
            rejected_clouds=self._rejected_clouds,
            source_period_ms=source_period_ms,
            arrival_period_ms=arrival_period_ms,
            artifact_shadow_enabled=bool(
                self.get_parameter("publish_artifact_shadow").value
            ),
            rolling_metrics=self._metrics.values(
                float(self.get_parameter("lag_spike_ms").value)
            ),
            stats=stats,
            stage_ms=stage_ms,
            artifact_stats=artifact_stats,
            artifact_support_stats=artifact_support_stats,
            artifact_front_cells=artifact_front_cells,
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [build_mapping_diagnostic_status(snapshot)]
        self._diagnostics_pub.publish(diagnostics)


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
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
