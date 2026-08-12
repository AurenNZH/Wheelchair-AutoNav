"""ROS2 node for non-actuating AIRY obstacle mapping."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
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
from visualization_msgs.msg import Marker, MarkerArray

from wheelchair_navigation.artifact_filter import (
    ArtifactCellSupportStats,
    ArtifactFilterStats,
    ArtifactGridCell,
    ArtifactHaloSpan,
    artifact_configured_halo_cell_ids,
    artifact_grid_membership,
    minimum_cell_support_filter,
    parse_artifact_grid_cells,
    parse_artifact_grid_halo_spans,
    validate_artifact_filter_frame,
)
from wheelchair_navigation.local_navigation import (
    CostmapStats,
    FrontCostmapConfig,
    LocalCostmapConfig,
    front_point_cell_ids,
    grid_origin_m,
    make_costmap_stats,
    make_front_grid,
    make_full_raw_grid,
    obstacle_point_mask,
    parse_self_filter_boxes,
    select_front_points,
    validate_mapping_configs,
)
from wheelchair_navigation.mapping_diagnostics import MappingMetrics
from wheelchair_navigation.point_cloud import (
    point_cloud_to_arrays,
    transform_points,
    xyz_to_point_cloud,
)


class LocalNavigationNode(Node):
    """Publish raw full and robot-forward grids without commanding motion."""

    def __init__(self) -> None:
        super().__init__("local_costmap")

        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("raw_obstacles_topic", "/local_obstacles")
        self.declare_parameter("front_costmap_topic", "/front_costmap")
        self.declare_parameter("publish_local_obstacles", True)
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
        self._last_cloud_stamp_ns = None
        self._last_arrival_monotonic = None
        self._metrics = MappingMetrics(
            int(self.get_parameter("latency_window_samples").value)
        )
        map_topics = "/front_costmap"
        if bool(self.get_parameter("publish_local_obstacles").value):
            map_topics += " and /local_obstacles"
        self.get_logger().info(
            f"Mapping-only AIRY mapper started; publishing {map_topics}. "
            "No motion commands are published."
        )

    def _on_lidar(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        arrival_monotonic = time.monotonic()
        arrival_ros_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        cloud_arrival_age_ms = (
            max(0.0, (arrival_ros_ns - stamp_ns) / 1e6)
            if stamp_ns > 0
            else 0.0
        )
        source_period_ms = self._period_ms(
            stamp_ns, self._last_cloud_stamp_ns
        )
        arrival_period_ms = self._period_ms(
            arrival_monotonic,
            self._last_arrival_monotonic,
            scale=1000.0,
        )
        self._last_cloud_stamp_ns = stamp_ns
        self._last_arrival_monotonic = arrival_monotonic

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

            raw = None
            if bool(
                self.get_parameter("publish_local_obstacles").value
            ):
                raw_started = time.perf_counter()
                raw = make_full_raw_grid(accepted, map_config)
                stage_ms["raw_raster_ms"] = (
                    time.perf_counter() - raw_started
                ) * 1000.0
            else:
                stage_ms["raw_raster_ms"] = 0.0
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
        self._front_pub.publish(
            build_front_occupancy_grid(header, front, front_config)
        )
        stage_ms["publish_front_ms"] = (
            time.perf_counter() - publish_started
        ) * 1000.0
        front_publish_monotonic = time.monotonic()
        front_publish_age_ms = (
            max(
                0.0,
                (self.get_clock().now().nanoseconds - stamp_ns) / 1e6,
            )
            if stamp_ns > 0
            else 0.0
        )

        if raw is not None:
            publish_started = time.perf_counter()
            self._raw_pub.publish(
                build_occupancy_grid(header, raw, map_config)
            )
            stage_ms["publish_raw_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        else:
            stage_ms["publish_raw_ms"] = 0.0
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
                front_valid_mask, cell_ids, cell_count = (
                    front_point_cell_ids(points_base, front_config)
                )
                mask_rejected, artifact_stats = artifact_grid_membership(
                    points_base,
                    cell_ids,
                    front_valid_mask,
                    cells,
                    front_config,
                    accepted_mask,
                )
                threshold_candidate_cells = (
                    artifact_configured_halo_cell_ids(
                        halo_spans,
                        cells,
                        front_config,
                    )
                )
                support_result = minimum_cell_support_filter(
                    cell_ids,
                    front_valid_mask,
                    accepted_mask,
                    mask_rejected,
                    threshold_candidate_cells,
                    cell_count=cell_count,
                    min_points_per_cell=self.get_parameter(
                        "artifact_min_points_per_cell"
                    ).value,
                    global_min_points_per_cell=self.get_parameter(
                        "artifact_global_min_points_per_cell"
                    ).value,
                )
                artifact_support_stats = support_result.stats
                shadow_front_points = points_base[
                    support_result.shadow_mask & front_valid_mask
                ]
                artifact_front = make_front_grid(
                    shadow_front_points, front_config
                )
                rejected_sensor = cloud.xyz[mask_rejected]
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
        cloud_age_ms = (
            max(
                0.0,
                (self.get_clock().now().nanoseconds - stamp_ns) / 1e6,
            )
            if validate_timestamps
            else 0.0
        )
        self._metrics.record(
            processing_ms,
            arrival_monotonic,
            cloud_age_ms=cloud_age_ms,
            mapping_ms=stage_ms["mapping_ms"],
            cloud_arrival_age_ms=cloud_arrival_age_ms,
            front_publish_age_ms=front_publish_age_ms,
            source_period_ms=source_period_ms,
            arrival_period_ms=arrival_period_ms,
            front_publish_s=front_publish_monotonic,
        )
        self._publish_diagnostics(
            diagnostic_reason,
            processing_ms,
            cloud_age_ms,
            stats=stats,
            stage_ms=stage_ms,
            source_period_ms=source_period_ms,
            arrival_period_ms=arrival_period_ms,
            artifact_stats=artifact_stats,
            artifact_support_stats=artifact_support_stats,
            artifact_front_cells=artifact_front_cells,
            cloud_arrival_age_ms=cloud_arrival_age_ms,
            front_publish_age_ms=front_publish_age_ms,
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

    @staticmethod
    def _period_ms(current, previous, scale: float = 1e-6) -> float:
        if previous is None or current <= previous:
            return 0.0
        return float(current - previous) * scale

    def _reject_cloud(
        self, reason: str, stamp_ns: int, started: float
    ) -> None:
        self._rejected_clouds += 1
        processing_ms = (time.perf_counter() - started) * 1000.0
        cloud_age_ms = (
            max(
                0.0,
                (self.get_clock().now().nanoseconds - stamp_ns) / 1e6,
            )
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
        artifact_stats: ArtifactFilterStats | None = None,
        artifact_support_stats: ArtifactCellSupportStats | None = None,
        artifact_front_cells: int | None = None,
        cloud_arrival_age_ms: float | None = None,
        front_publish_age_ms: float | None = None,
    ) -> None:
        processing_warn = float(
            self.get_parameter("processing_warn_ms").value
        )
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
            "publish_local_obstacles": str(
                bool(
                    self.get_parameter("publish_local_obstacles").value
                )
            ).lower(),
        }
        if cloud_arrival_age_ms is not None:
            values["cloud_arrival_age_ms"] = "%.3f" % (
                cloud_arrival_age_ms
            )
        if front_publish_age_ms is not None:
            values["front_publish_age_ms"] = "%.3f" % (
                front_publish_age_ms
            )
        values.update(
            self._metrics.values(
                float(self.get_parameter("lag_spike_ms").value)
            )
        )
        for key, value in (stage_ms or {}).items():
            values[key] = "%.3f" % value
        if stats is not None:
            values.update(
                {
                    "input_points": str(stats.input_points),
                    "finite_points": str(stats.finite_points),
                    "height_range_points": str(stats.height_range_points),
                    "self_filtered_points": str(
                        stats.self_filtered_points
                    ),
                    "accepted_points": str(stats.accepted_points),
                    "occupied_cells": str(stats.occupied_cells),
                    "front_points": str(stats.front_points),
                    "front_occupied_cells": str(
                        stats.front_occupied_cells
                    ),
                    "raw_front_cells": str(stats.front_occupied_cells),
                }
            )
        values["artifact_shadow_enabled"] = str(
            bool(self.get_parameter("publish_artifact_shadow").value)
        ).lower()
        if artifact_stats is not None:
            values.update(
                {
                    "artifact_region_count": str(
                        artifact_stats.region_count
                    ),
                    "artifact_unique_rejected_points": str(
                        artifact_stats.unique_rejected_points
                    ),
                    "artifact_filtered_front_cells": str(
                        artifact_front_cells
                    ),
                }
            )
            for index, count in enumerate(
                artifact_stats.per_region_rejected_points
            ):
                values["artifact_region_%d_rejected_points" % index] = str(
                    count
                )
        if artifact_support_stats is not None:
            values.update(
                {
                    "artifact_min_points_per_cell": str(
                        artifact_support_stats.min_points_per_cell
                    ),
                    "artifact_global_min_points_per_cell": str(
                        artifact_support_stats.global_min_points_per_cell
                    ),
                    "artifact_configured_halo_cells": str(
                        artifact_support_stats.configured_halo_cells
                    ),
                    "artifact_mask_touched_cells": str(
                        artifact_support_stats.mask_touched_cells
                    ),
                    "artifact_mask_removed_cells": str(
                        artifact_support_stats.mask_removed_cells
                    ),
                    "artifact_mask_mixed_cells": str(
                        artifact_support_stats.mask_mixed_cells
                    ),
                    "artifact_threshold_candidate_cells": str(
                        artifact_support_stats.threshold_candidate_cells
                    ),
                    "artifact_low_support_cells": str(
                        artifact_support_stats.low_support_cells
                    ),
                    "artifact_low_support_points": str(
                        artifact_support_stats.low_support_points
                    ),
                }
            )

        status = DiagnosticStatus()
        status.level = level
        status.name = "wheelchair_navigation/local_costmap"
        status.hardware_id = "robosense_airy"
        status.message = message
        status.values = [
            KeyValue(key=key, value=value) for key, value in values.items()
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


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


def artifact_shadow_error_reason(exc: Exception) -> str:
    """Map shadow-only failures to stable diagnostic messages."""

    if "frame mismatch" in str(exc):
        return "artifact_shadow_frame_mismatch"
    return "invalid_artifact_shadow_configuration"


def build_artifact_grid_markers(
    header: Header,
    cells: tuple[ArtifactGridCell, ...],
    halo_spans: tuple[ArtifactHaloSpan, ...],
    config: FrontCostmapConfig,
) -> MarkerArray:
    """Visualize consolidated cell meshes and their staircase halo outlines."""

    result = MarkerArray()
    clear = Marker()
    clear.header = header
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    colors = (
        (0.10, 0.85, 1.00),
        (0.85, 0.20, 1.00),
        (1.00, 0.55, 0.05),
    )
    resolution = float(config.resolution_m)
    region_ids = sorted({cell.region_id for cell in cells})
    for region_id in region_ids:
        region_cells = tuple(
            cell for cell in cells if cell.region_id == region_id
        )
        red, green, blue = colors[region_id % len(colors)]
        mesh = Marker()
        mesh.header = header
        mesh.ns = "artifact_grid_regions"
        mesh.id = region_id
        mesh.type = Marker.TRIANGLE_LIST
        mesh.action = Marker.ADD
        mesh.pose.orientation.w = 1.0
        mesh.color.r = red
        mesh.color.g = green
        mesh.color.b = blue
        mesh.color.a = 0.34
        mesh.frame_locked = True
        for cell in region_cells:
            mesh.points.extend(_cell_box_triangle_points(cell, resolution))
        result.markers.append(mesh)

        outline = Marker()
        outline.header = header
        outline.ns = "artifact_grid_region_outlines"
        outline.id = region_id
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.012
        outline.color.r = red
        outline.color.g = green
        outline.color.b = blue
        outline.color.a = 1.0
        outline.frame_locked = True
        outline.points = _cell_footprint_outline_points(
            {
                (cell.forward_cell, cell.lateral_cell)
                for cell in region_cells
            },
            resolution,
            max(cell.max_z_m for cell in region_cells) + 0.006,
        )
        result.markers.append(outline)

        halo_outline = Marker()
        halo_outline.header = header
        halo_outline.ns = "artifact_grid_halo_outlines"
        halo_outline.id = region_id
        halo_outline.type = Marker.LINE_LIST
        halo_outline.action = Marker.ADD
        halo_outline.pose.orientation.w = 1.0
        halo_outline.scale.x = 0.018
        halo_outline.color.r = 0.25
        halo_outline.color.g = 1.0
        halo_outline.color.b = 0.25
        halo_outline.color.a = 1.0
        halo_outline.frame_locked = True
        halo_ids = artifact_configured_halo_cell_ids(
            halo_spans,
            cells,
            config,
            region_id=region_id,
        )
        width = int(np.ceil(config.length_m / resolution))
        height = int(np.ceil(config.width_m / resolution))
        zero_row = height // 2
        halo_outline.points = _cell_footprint_outline_points(
            {
                (int(cell_id % width), int(cell_id // width) - zero_row)
                for cell_id in halo_ids
            },
            resolution,
            max(cell.max_z_m for cell in region_cells) + 0.025,
        )
        result.markers.append(halo_outline)
    return result


def build_artifact_threshold_cell_markers(
    header: Header,
    config: FrontCostmapConfig,
    candidate_cell_ids: np.ndarray,
    low_support_cell_ids: np.ndarray,
) -> MarkerArray:
    """Show exact base-frame cells evaluated by the support threshold."""

    geometry = np.asarray(
        [config.length_m, config.width_m, config.resolution_m],
        dtype=np.float64,
    )
    if not np.isfinite(geometry).all() or np.any(geometry <= 0.0):
        raise ValueError("artifact threshold marker grid must be positive")
    width = int(np.ceil(config.length_m / config.resolution_m))
    height = int(np.ceil(config.width_m / config.resolution_m))
    origin_y_m = -(height * config.resolution_m) / 2.0
    result = MarkerArray()
    result.markers.append(
        _cell_list_marker(
            header,
            "artifact_threshold_candidate_cells",
            candidate_cell_ids,
            width,
            height,
            origin_y_m,
            config.resolution_m,
            color=(0.05, 0.75, 1.0, 0.22),
            z_m=0.02,
        )
    )
    result.markers.append(
        _cell_list_marker(
            header,
            "artifact_threshold_low_support_cells",
            low_support_cell_ids,
            width,
            height,
            origin_y_m,
            config.resolution_m,
            color=(1.0, 0.85, 0.0, 0.48),
            z_m=0.035,
        )
    )
    return result


def _cell_list_marker(
    header: Header,
    namespace: str,
    cell_ids: np.ndarray,
    width: int,
    height: int,
    origin_y_m: float,
    resolution_m: float,
    *,
    color: tuple[float, float, float, float],
    z_m: float,
) -> Marker:
    """Build one efficient cube-list marker from flat front-grid IDs."""

    ids = np.asarray(cell_ids, dtype=np.int64)
    if ids.ndim != 1:
        raise ValueError("artifact threshold cell IDs must have shape (N,)")
    if np.any((ids < 0) | (ids >= width * height)):
        raise ValueError(
            "artifact threshold marker cell ID is outside the grid"
        )

    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD if ids.size else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = resolution_m * 0.96
    marker.scale.y = resolution_m * 0.96
    marker.scale.z = 0.02
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    marker.frame_locked = True
    rows = ids // width
    cols = ids % width
    for row, col in zip(rows, cols):
        point = Point()
        point.x = (float(col) + 0.5) * resolution_m
        point.y = origin_y_m + (float(row) + 0.5) * resolution_m
        point.z = z_m
        marker.points.append(point)
    return marker


def _cell_box_triangle_points(
    cell: ArtifactGridCell,
    resolution_m: float,
) -> list[Point]:
    """Return twelve triangles for one height-bounded costmap cell."""

    min_x = cell.forward_cell * resolution_m
    max_x = min_x + resolution_m
    min_y = cell.lateral_cell * resolution_m
    max_y = min_y + resolution_m
    corners = [
        (min_x, min_y, cell.min_z_m),
        (max_x, min_y, cell.min_z_m),
        (max_x, max_y, cell.min_z_m),
        (min_x, max_y, cell.min_z_m),
        (min_x, min_y, cell.max_z_m),
        (max_x, min_y, cell.max_z_m),
        (max_x, max_y, cell.max_z_m),
        (min_x, max_y, cell.max_z_m),
    ]
    triangles = (
        (0, 2, 1), (0, 3, 2),
        (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4),
        (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6),
        (3, 0, 4), (3, 4, 7),
    )
    points = []
    for triangle in triangles:
        for index in triangle:
            point = Point()
            point.x, point.y, point.z = corners[index]
            points.append(point)
    return points


def _cell_footprint_outline_points(
    cells: set[tuple[int, int]],
    resolution_m: float,
    z_m: float,
) -> list[Point]:
    """Return only the exterior line segments of a staircase footprint."""

    points = []
    for col, row in sorted(cells):
        min_x = col * resolution_m
        max_x = min_x + resolution_m
        min_y = row * resolution_m
        max_y = min_y + resolution_m
        edges = []
        if (col - 1, row) not in cells:
            edges.append(((min_x, min_y), (min_x, max_y)))
        if (col + 1, row) not in cells:
            edges.append(((max_x, min_y), (max_x, max_y)))
        if (col, row - 1) not in cells:
            edges.append(((min_x, min_y), (max_x, min_y)))
        if (col, row + 1) not in cells:
            edges.append(((min_x, max_y), (max_x, max_y)))
        for start, end in edges:
            for x_m, y_m in (start, end):
                point = Point()
                point.x = x_m
                point.y = y_m
                point.z = z_m
                points.append(point)
    return points


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
    default_origin = (
        grid_origin_m(config)
        if isinstance(config, LocalCostmapConfig)
        else 0.0
    )
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
