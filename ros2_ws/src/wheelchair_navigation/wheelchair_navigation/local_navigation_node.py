"""ROS2 node for the non-actuating LiDAR local-mapping baseline."""

from __future__ import annotations

from collections import deque
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from wheelchair_navigation.local_navigation import (
    CostmapStats,
    FrontCostmapConfig,
    GhostFilterConfig,
    GhostFilterStats,
    LocalCostmapConfig,
    TemporalGhostFilter,
    filter_obstacle_points,
    grid_origin_m,
    inflate_grid,
    make_costmap_stats,
    make_front_raw_grid,
    make_full_raw_grid,
    parse_self_filter_boxes,
    select_front_points,
    validate_mapping_configs,
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
        self.declare_parameter(
            "filtered_front_costmap_topic", "/front_costmap_filtered"
        )
        self.declare_parameter(
            "rejected_front_costmap_topic", "/front_costmap_rejected"
        )
        self.declare_parameter(
            "rejected_front_cells_topic", "/front_costmap_rejected_cells"
        )
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("publish_raw_obstacles", True)
        self.declare_parameter("publish_derived_costmap", False)
        self.declare_parameter("publish_front_costmap", True)
        self.declare_parameter("publish_filtered_front_costmap", True)
        self.declare_parameter("publish_rejected_front_costmap", True)
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
        self.declare_parameter("self_filter_padding_m", 0.0)
        self.declare_parameter("ghost_filter_min_component_cells", 2)
        self.declare_parameter("ghost_filter_history_frames", 3)
        self.declare_parameter("ghost_filter_min_hits", 2)
        self.declare_parameter("ghost_filter_match_radius_cells", 1)
        self.declare_parameter("ghost_filter_reset_gap_s", 0.5)
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)
        self.declare_parameter("processing_warn_ms", 100.0)
        self.declare_parameter("cloud_age_warn_ms", 150.0)
        self.declare_parameter("latency_window_samples", 120)
        self.declare_parameter("lag_spike_ms", 150.0)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publish_raw = bool(
            self.get_parameter("publish_raw_obstacles").value
        )
        self._publish_derived = bool(
            self.get_parameter("publish_derived_costmap").value
        )
        self._publish_front = bool(
            self.get_parameter("publish_front_costmap").value
        )
        self._publish_filtered_front = bool(
            self.get_parameter("publish_filtered_front_costmap").value
        )
        self._publish_rejected_front = bool(
            self.get_parameter("publish_rejected_front_costmap").value
        )
        self._build_front = (
            self._publish_front
            or self._publish_filtered_front
            or self._publish_rejected_front
        )
        if not (
            self._publish_raw
            or self._publish_derived
            or self._build_front
        ):
            raise ValueError("at least one costmap output must be enabled")
        self._raw_pub = (
            self.create_publisher(
                OccupancyGrid,
                self.get_parameter("raw_obstacles_topic").value,
                1,
            )
            if self._publish_raw
            else None
        )
        self._costmap_pub = (
            self.create_publisher(
                OccupancyGrid, self.get_parameter("costmap_topic").value, 1
            )
            if self._publish_derived
            else None
        )
        self._front_costmap_pub = (
            self.create_publisher(
                OccupancyGrid,
                self.get_parameter("front_costmap_topic").value,
                1,
            )
            if self._publish_front
            else None
        )
        self._filtered_front_costmap_pub = (
            self.create_publisher(
                OccupancyGrid,
                self.get_parameter("filtered_front_costmap_topic").value,
                1,
            )
            if self._publish_filtered_front
            else None
        )
        self._rejected_front_costmap_pub = (
            self.create_publisher(
                OccupancyGrid,
                self.get_parameter("rejected_front_costmap_topic").value,
                1,
            )
            if self._publish_rejected_front
            else None
        )
        self._rejected_front_cells_pub = (
            self.create_publisher(
                GridCells,
                self.get_parameter("rejected_front_cells_topic").value,
                1,
            )
            if self._publish_rejected_front
            else None
        )
        self._ghost_filter = (
            TemporalGhostFilter(self._ghost_filter_config())
            if self._publish_filtered_front or self._publish_rejected_front
            else None
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
        self._last_cloud_stamp_ns = None
        self._last_arrival_monotonic = None
        latency_window_samples = int(
            self.get_parameter("latency_window_samples").value
        )
        if latency_window_samples < 1:
            raise ValueError("latency_window_samples must be positive")
        self._processing_history_ms = deque(maxlen=latency_window_samples)
        self._processed_arrival_history = deque(
            maxlen=latency_window_samples
        )
        self.get_logger().info(
            "Mapping-only local costmap started; outputs raw=%s derived=%s "
            "front=%s filtered_front=%s rejected_front=%s; shadow filtering "
            "does not affect raw maps and no motion commands are published."
            % (
                self._publish_raw,
                self._publish_derived,
                self._publish_front,
                self._publish_filtered_front,
                self._publish_rejected_front,
            )
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
            map_config = self._map_config()
            front_config = self._front_map_config()
            validate_mapping_configs(map_config, front_config)
            boxes = parse_self_filter_boxes(
                self.get_parameter("self_filter_boxes").value,
                float(self.get_parameter("self_filter_padding_m").value),
            )

            filter_started = time.perf_counter()
            accepted, counts = filter_obstacle_points(
                points_base, map_config, boxes
            )
            stage_ms["filter_ms"] = (
                time.perf_counter() - filter_started
            ) * 1000.0

            raw = None
            costmap = None
            if self._publish_raw or self._publish_derived:
                raw_started = time.perf_counter()
                raw = make_full_raw_grid(accepted, map_config)
                stage_ms["raw_raster_ms"] = (
                    time.perf_counter() - raw_started
                ) * 1000.0
            if self._publish_derived:
                inflation_started = time.perf_counter()
                costmap = inflate_grid(
                    raw,
                    map_config.inflation_radius_m,
                    map_config.resolution_m,
                    map_config.occupied_cost,
                )
                stage_ms["derived_inflation_ms"] = (
                    time.perf_counter() - inflation_started
                ) * 1000.0

            front_points = None
            front_raw_grid = None
            front_costmap = None
            filtered_front_costmap = None
            rejected_front_costmap = None
            ghost_stats = None
            if self._build_front:
                front_select_started = time.perf_counter()
                front_points = select_front_points(accepted, front_config)
                stage_ms["front_select_ms"] = (
                    time.perf_counter() - front_select_started
                ) * 1000.0
                front_raster_started = time.perf_counter()
                front_raw_grid = make_front_raw_grid(
                    front_points, front_config
                )
                front_costmap = inflate_grid(
                    front_raw_grid,
                    front_config.inflation_radius_m,
                    front_config.resolution_m,
                    front_config.occupied_cost,
                )
                stage_ms["front_raster_ms"] = (
                    time.perf_counter() - front_raster_started
                ) * 1000.0
            if self._ghost_filter is not None:
                ghost_started = time.perf_counter()
                filtered_front_raw, rejected_front_costmap, ghost_stats = (
                    self._ghost_filter.filter(front_raw_grid, stamp_ns)
                )
                stage_ms["ghost_filter_ms"] = (
                    time.perf_counter() - ghost_started
                ) * 1000.0
                filtered_inflation_started = time.perf_counter()
                filtered_front_costmap = inflate_grid(
                    filtered_front_raw,
                    front_config.inflation_radius_m,
                    front_config.resolution_m,
                    front_config.occupied_cost,
                )
                stage_ms["filtered_front_inflation_ms"] = (
                    time.perf_counter() - filtered_inflation_started
                ) * 1000.0

            stage_ms["mapping_ms"] = sum(
                stage_ms.get(key, 0.0)
                for key in (
                    "filter_ms",
                    "raw_raster_ms",
                    "derived_inflation_ms",
                    "front_select_ms",
                    "front_raster_ms",
                    "ghost_filter_ms",
                    "filtered_front_inflation_ms",
                )
            )
            stats = make_costmap_stats(
                counts,
                accepted,
                raw,
                map_config,
                front_points=front_points,
                front=front_costmap,
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
        header.stamp = msg.header.stamp
        header.frame_id = target_frame
        if self._raw_pub is not None:
            publish_started = time.perf_counter()
            self._raw_pub.publish(build_occupancy_grid(header, raw, map_config))
            stage_ms["publish_raw_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        if self._costmap_pub is not None:
            publish_started = time.perf_counter()
            self._costmap_pub.publish(
                build_occupancy_grid(header, costmap, map_config)
            )
            stage_ms["publish_derived_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        if self._front_costmap_pub is not None:
            publish_started = time.perf_counter()
            self._front_costmap_pub.publish(
                build_front_occupancy_grid(
                    header, front_costmap, front_config
                )
            )
            stage_ms["publish_front_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        if self._filtered_front_costmap_pub is not None:
            publish_started = time.perf_counter()
            self._filtered_front_costmap_pub.publish(
                build_front_occupancy_grid(
                    header, filtered_front_costmap, front_config
                )
            )
            stage_ms["publish_filtered_front_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        if self._rejected_front_costmap_pub is not None:
            publish_started = time.perf_counter()
            self._rejected_front_costmap_pub.publish(
                build_front_occupancy_grid(
                    header, rejected_front_costmap, front_config
                )
            )
            stage_ms["publish_rejected_front_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        if self._rejected_front_cells_pub is not None:
            publish_started = time.perf_counter()
            self._rejected_front_cells_pub.publish(
                build_front_grid_cells(
                    header, rejected_front_costmap, front_config
                )
            )
            stage_ms["publish_rejected_cells_ms"] = (
                time.perf_counter() - publish_started
            ) * 1000.0
        stage_ms["publish_ms"] = sum(
            stage_ms.get(key, 0.0)
            for key in (
                "publish_raw_ms",
                "publish_derived_ms",
                "publish_front_ms",
                "publish_filtered_front_ms",
                "publish_rejected_front_ms",
                "publish_rejected_cells_ms",
            )
        )

        self._processed_clouds += 1
        self._processed_arrival_history.append(arrival_monotonic)
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
            ghost_stats=ghost_stats,
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

    def _ghost_filter_config(self) -> GhostFilterConfig:
        return GhostFilterConfig(
            min_component_cells=int(
                self.get_parameter("ghost_filter_min_component_cells").value
            ),
            history_frames=int(
                self.get_parameter("ghost_filter_history_frames").value
            ),
            min_hits=int(
                self.get_parameter("ghost_filter_min_hits").value
            ),
            match_radius_cells=int(
                self.get_parameter("ghost_filter_match_radius_cells").value
            ),
            reset_gap_s=float(
                self.get_parameter("ghost_filter_reset_gap_s").value
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
        ghost_stats: GhostFilterStats | None = None,
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
            "publish_raw_obstacles": str(self._publish_raw),
            "publish_derived_costmap": str(self._publish_derived),
            "publish_front_costmap": str(self._publish_front),
            "publish_filtered_front_costmap": str(
                self._publish_filtered_front
            ),
            "publish_rejected_front_costmap": str(
                self._publish_rejected_front
            ),
        }
        if reason == "ok":
            self._processing_history_ms.append(float(processing_ms))
        if self._processing_history_ms:
            history = np.asarray(self._processing_history_ms, dtype=np.float32)
            lag_spike_ms = float(self.get_parameter("lag_spike_ms").value)
            values.update(
                {
                    "latency_window_count": str(history.size),
                    "processing_p50_ms": "%.3f"
                    % float(np.percentile(history, 50)),
                    "processing_p95_ms": "%.3f"
                    % float(np.percentile(history, 95)),
                    "processing_max_ms": "%.3f" % float(np.max(history)),
                    "lag_spike_count": str(
                        int(np.count_nonzero(history > lag_spike_ms))
                    ),
                }
            )
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
        if ghost_stats is not None:
            values.update(
                {
                    "ghost_raw_cells": str(ghost_stats.raw_cells),
                    "ghost_component_count": str(
                        ghost_stats.component_count
                    ),
                    "ghost_strong_component_cells": str(
                        ghost_stats.strong_component_cells
                    ),
                    "ghost_isolated_cells": str(
                        ghost_stats.isolated_cells
                    ),
                    "ghost_temporal_rescued_cells": str(
                        ghost_stats.temporal_rescued_cells
                    ),
                    "ghost_filtered_cells": str(
                        ghost_stats.filtered_cells
                    ),
                    "ghost_rejected_cells": str(
                        ghost_stats.rejected_cells
                    ),
                    "ghost_history_reset": str(
                        ghost_stats.history_reset
                    ),
                }
            )
        if len(self._processed_arrival_history) > 1:
            elapsed_s = (
                self._processed_arrival_history[-1]
                - self._processed_arrival_history[0]
            )
            effective_rate_hz = (
                (len(self._processed_arrival_history) - 1)
                / max(elapsed_s, 1e-6)
            )
        else:
            effective_rate_hz = 0.0
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
            self.get_logger().debug(
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
                throttle_duration_sec=5.0,
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
    msg.data = costmap.reshape(-1).tolist()
    return msg


def build_front_occupancy_grid(
    header: Header,
    costmap: np.ndarray,
    config: FrontCostmapConfig,
) -> OccupancyGrid:
    """Build a front-grid message with the shared robot-forward origin."""

    return build_occupancy_grid(
        header,
        costmap,
        config,
        origin_x_m=0.0,
        origin_y_m=-(costmap.shape[0] * config.resolution_m) / 2.0,
    )


def build_front_grid_cells(
    header: Header,
    costmap: np.ndarray,
    config: FrontCostmapConfig,
) -> GridCells:
    """Build a color-configurable RViz overlay for occupied front cells."""
    msg = GridCells()
    msg.header = header
    msg.cell_width = float(config.resolution_m)
    msg.cell_height = float(config.resolution_m)
    origin_y_m = -(costmap.shape[0] * config.resolution_m) / 2.0
    rows, columns = np.nonzero(costmap >= config.occupied_cost)
    msg.cells = [
        Point(
            x=(float(column) + 0.5) * config.resolution_m,
            y=origin_y_m + (float(row) + 0.5) * config.resolution_m,
            z=0.02,
        )
        for row, column in zip(rows, columns)
    ]
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
