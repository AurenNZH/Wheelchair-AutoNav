"""ROS wiring for the L2 point-cell support filter."""

from __future__ import annotations

import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
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
    artifact_halo_cell_ids,
    parse_artifact_box,
    parse_artifact_boxes,
    parse_artifact_halo_bounds,
    points_in_artifact_box,
    points_in_artifact_boxes,
)
from wheelchair_navigation.artifact_markers import build_artifact_markers
from wheelchair_navigation.costmap import (
    LocalCostmapConfig,
    SupportGridConfig,
    minimum_range_rejection_mask,
    obstacle_point_mask,
    validate_mapping_configs,
)
from wheelchair_navigation.mapping_diagnostics import MappingMetrics
from wheelchair_navigation.point_cloud import (
    point_cloud_to_arrays,
    transform_points,
    xyz_to_point_cloud,
)
from wheelchair_navigation.point_support import (
    PointSupportResult,
    filter_points_by_cell_support,
)
from wheelchair_navigation.timing import cloud_timestamp_error


class PointSupportFilterNode(Node):
    """Remove low-support obstacle cells before the stock Nav2 layer."""

    def __init__(self) -> None:
        super().__init__("point_support_filter")
        self._declare_parameters()
        self._configuration_error = None
        self._artifact_box = None
        self._artifact_additional_boxes = ()
        self._artifact_halo_bounds = None
        self._artifact_halo_cell_ids = np.empty(0, dtype=np.int64)
        try:
            self._map_config = self._load_map_config()
            self._support_grid_config = self._load_support_grid_config()
            validate_mapping_configs(
                self._map_config, self._support_grid_config
            )
            self._load_artifact_configuration()
            if float(self.get_parameter("diagnostics_period_s").value) <= 0.0:
                raise ValueError("diagnostics_period_s must be positive")
        except (TypeError, ValueError) as exc:
            self._configuration_error = str(exc)
            self.get_logger().error("Invalid point-support configuration: %s" % exc)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._filtered_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("filtered_cloud_topic").value),
            sensor_qos,
        )
        self._source_header_pub = self.create_publisher(
            Header,
            str(self.get_parameter("source_header_topic").value),
            10,
        )
        self._low_support_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("low_support_points_topic").value),
            sensor_qos,
        )
        self._artifact_rejected_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("artifact_rejected_points_topic").value),
            sensor_qos,
        )
        self._artifact_markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("artifact_markers_topic").value),
            marker_qos,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar_topic").value),
            self._on_cloud,
            sensor_qos,
        )
        self._received_clouds = 0
        self._published_clouds = 0
        self._rejected_clouds = 0
        self._metrics = MappingMetrics(
            int(self.get_parameter("latency_window_samples").value)
        )
        self._last_success_diagnostics_s = float("-inf")
        self._publish_artifact_markers()
        self.get_logger().warn(
            "%s support filter active: %s -> %s; artifact_rule=%s. "
            "No motion commands are published."
            % (
                self.get_parameter("sensor_label").value,
                self.get_parameter("lidar_topic").value,
                self.get_parameter("filtered_cloud_topic").value,
                self.get_parameter("artifact_filter_enabled").value,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("sensor_label", "L2 right")
        self.declare_parameter("lidar_topic", "/lidar_right/points")
        self.declare_parameter(
            "filtered_cloud_topic", "/lidar_right/points_filtered"
        )
        self.declare_parameter(
            "source_header_topic", "/lidar_right/filter/source_header"
        )
        self.declare_parameter(
            "low_support_points_topic", "/lidar_right/low_support_points"
        )
        self.declare_parameter(
            "artifact_rejected_points_topic",
            "/lidar_right/artifact_rejected_points",
        )
        self.declare_parameter(
            "artifact_markers_topic", "/lidar_right/artifact_filter/markers"
        )
        self.declare_parameter(
            "diagnostic_name",
            "wheelchair_navigation/point_support_filter/lidar_right",
        )
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("size_m", 8.0)
        self.declare_parameter("resolution_m", 0.1)
        self.declare_parameter("min_height_m", 0.05)
        self.declare_parameter("max_height_m", 1.5)
        self.declare_parameter("min_range_m", 0.45)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("support_origin_x_m", -0.6)
        self.declare_parameter("support_origin_y_m", -4.0)
        self.declare_parameter("support_width_m", 5.0)
        self.declare_parameter("support_height_m", 8.0)
        self.declare_parameter("support_resolution_m", 0.1)
        self.declare_parameter("min_points_per_cell", 3)
        self.declare_parameter("artifact_filter_enabled", False)
        self.declare_parameter("artifact_filter_frame", "base_link")
        self.declare_parameter("artifact_box", [])
        self.declare_parameter("artifact_additional_boxes", [])
        self.declare_parameter("artifact_halo_bounds_xy", [])
        self.declare_parameter("artifact_halo_margin_m", 0.10)
        self.declare_parameter("artifact_halo_min_points_per_cell", 15)
        self.declare_parameter("artifact_marker_namespace", "lidar_right")
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)
        self.declare_parameter("validate_cloud_timestamps", True)
        self.declare_parameter("processing_warn_ms", 100.0)
        self.declare_parameter("cloud_age_warn_ms", 150.0)
        self.declare_parameter("latency_window_samples", 120)
        self.declare_parameter("lag_spike_ms", 150.0)
        self.declare_parameter("diagnostics_period_s", 1.0)

    def _load_map_config(self) -> LocalCostmapConfig:
        return LocalCostmapConfig(
            size_m=float(self.get_parameter("size_m").value),
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_height_m=float(self.get_parameter("min_height_m").value),
            max_height_m=float(self.get_parameter("max_height_m").value),
            min_range_m=float(self.get_parameter("min_range_m").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
        )

    def _load_support_grid_config(self) -> SupportGridConfig:
        return SupportGridConfig(
            origin_x_m=float(
                self.get_parameter("support_origin_x_m").value
            ),
            origin_y_m=float(
                self.get_parameter("support_origin_y_m").value
            ),
            width_m=float(self.get_parameter("support_width_m").value),
            height_m=float(self.get_parameter("support_height_m").value),
            resolution_m=float(
                self.get_parameter("support_resolution_m").value
            ),
        )

    def _load_artifact_configuration(self) -> None:
        if not bool(self.get_parameter("artifact_filter_enabled").value):
            return
        target_frame = str(self.get_parameter("target_frame").value)
        filter_frame = str(self.get_parameter("artifact_filter_frame").value)
        if not filter_frame or filter_frame != target_frame:
            raise ValueError(
                "artifact filter frame must match target_frame '%s'"
                % target_frame
            )
        self._artifact_box = parse_artifact_box(
            self.get_parameter("artifact_box").value
        )
        self._artifact_additional_boxes = parse_artifact_boxes(
            self.get_parameter("artifact_additional_boxes").value
        )
        explicit_halo = self.get_parameter("artifact_halo_bounds_xy").value
        if explicit_halo:
            self._artifact_halo_bounds = parse_artifact_halo_bounds(
                explicit_halo
            )
        else:
            self._artifact_halo_cell_ids = artifact_halo_cell_ids(
                self._artifact_box,
                self._support_grid_config,
                float(self.get_parameter("artifact_halo_margin_m").value),
            )
        halo_minimum = self.get_parameter(
            "artifact_halo_min_points_per_cell"
        ).value
        if isinstance(halo_minimum, bool) or int(halo_minimum) != halo_minimum:
            raise ValueError("artifact halo threshold must be an integer")
        if int(halo_minimum) < 1:
            raise ValueError("artifact halo threshold must be at least one")

    def _publish_artifact_markers(self) -> None:
        if self._artifact_box is None or self._configuration_error is not None:
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("artifact_filter_frame").value)
        self._artifact_markers_pub.publish(
            build_artifact_markers(
                header,
                self._artifact_box,
                float(self.get_parameter("artifact_halo_margin_m").value),
                str(self.get_parameter("artifact_marker_namespace").value),
                self._artifact_halo_bounds,
                self._artifact_additional_boxes,
            )
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        arrival_s = time.monotonic()
        self._received_clouds += 1
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        validate_timestamps = bool(
            self.get_parameter("validate_cloud_timestamps").value
        )
        if validate_timestamps:
            reason = cloud_timestamp_error(
                now_ns=self.get_clock().now().nanoseconds,
                stamp_ns=stamp_ns,
                max_age_s=float(self.get_parameter("max_cloud_age_s").value),
                max_future_offset_s=float(
                    self.get_parameter("max_future_offset_s").value
                ),
            )
            if reason is not None:
                self._reject(reason, started)
                return
        if self._configuration_error is not None:
            self._reject("invalid_point_support", started)
            return

        stage_ms = {}
        try:
            stage_started = time.perf_counter()
            cloud = point_cloud_to_arrays(msg)
            stage_ms["decode_ms"] = (time.perf_counter() - stage_started) * 1000.0
        except ValueError as exc:
            self.get_logger().warn(
                "Invalid %s PointCloud2: %s"
                % (self.get_parameter("sensor_label").value, exc),
                throttle_duration_sec=5.0,
            )
            self._reject("invalid_lidar", started)
            return
        if not cloud.xyz.size:
            self._reject("empty_lidar", started)
            return

        target_frame = str(self.get_parameter("target_frame").value)
        points_base = cloud.xyz
        stage_started = time.perf_counter()
        if msg.header.frame_id != target_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)
                )
            except TransformException as exc:
                self.get_logger().warn(
                    "No timestamped TF from %s to %s: %s"
                    % (msg.header.frame_id, target_frame, exc),
                    throttle_duration_sec=5.0,
                )
                self._reject("missing_tf", started)
                return
            points_base = transform_points(points_base, transform)
        stage_ms["transform_ms"] = (time.perf_counter() - stage_started) * 1000.0

        try:
            stage_started = time.perf_counter()
            eligible, _ = obstacle_point_mask(points_base, self._map_config)
            minimum_range_rejected = minimum_range_rejection_mask(
                points_base, self._map_config.min_range_m
            )
            artifact_rejected = (
                points_in_artifact_box(points_base, self._artifact_box)
                if self._artifact_box is not None
                else np.zeros(points_base.shape[0], dtype=bool)
            )
            artifact_rejected |= points_in_artifact_boxes(
                points_base, self._artifact_additional_boxes
            )
            hard_rejected = minimum_range_rejected | artifact_rejected
            result = filter_points_by_cell_support(
                points_base,
                eligible,
                self._support_grid_config,
                min_points_per_cell=self.get_parameter("min_points_per_cell").value,
                hard_rejected_mask=hard_rejected,
                halo_cell_ids=(
                    self._artifact_halo_cell_ids
                    if self._artifact_halo_bounds is None
                    else None
                ),
                halo_bounds_xy=self._artifact_halo_bounds,
                halo_min_points_per_cell=self.get_parameter(
                    "artifact_halo_min_points_per_cell"
                ).value,
            )
            stage_ms["filter_ms"] = (time.perf_counter() - stage_started) * 1000.0
            stage_started = time.perf_counter()
            output_header = Header()
            output_header.stamp = msg.header.stamp
            output_header.frame_id = target_frame
            filtered = xyz_to_point_cloud(
                points_base[result.keep_mask], output_header
            )
            stage_ms["pack_ms"] = (time.perf_counter() - stage_started) * 1000.0
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                "Point support filter suppressed output: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._reject("invalid_point_support", started)
            return

        stage_started = time.perf_counter()
        self._filtered_pub.publish(filtered)
        self._source_header_pub.publish(filtered.header)
        if self._low_support_pub.get_subscription_count() > 0:
            self._low_support_pub.publish(
                xyz_to_point_cloud(
                    points_base[result.low_support_mask], output_header
                )
            )
        if self._artifact_rejected_pub.get_subscription_count() > 0:
            self._artifact_rejected_pub.publish(
                xyz_to_point_cloud(
                    points_base[artifact_rejected], output_header
                )
            )
        stage_ms["publish_ms"] = (time.perf_counter() - stage_started) * 1000.0
        self._published_clouds += 1

        processing_ms = (time.perf_counter() - started) * 1000.0
        cloud_age_ms = (
            max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e6)
            if validate_timestamps
            else 0.0
        )
        self._metrics.record(
            processing_ms,
            arrival_s,
            cloud_age_ms=cloud_age_ms,
            mapping_ms=processing_ms,
        )
        if arrival_s - self._last_success_diagnostics_s >= float(
            self.get_parameter("diagnostics_period_s").value
        ):
            self._last_success_diagnostics_s = arrival_s
            self._publish_diagnostics(
                "ok",
                processing_ms,
                cloud_age_ms,
                result=result,
                input_points=int(cloud.xyz.shape[0]),
                output_points=int(np.count_nonzero(result.keep_mask)),
                stage_ms=stage_ms,
                artifact_rejected_points=int(
                    np.count_nonzero(artifact_rejected)
                ),
                minimum_range_rejected_points=int(
                    np.count_nonzero(minimum_range_rejected)
                ),
            )

    def _reject(self, reason: str, started: float) -> None:
        self._rejected_clouds += 1
        self._publish_diagnostics(
            reason, (time.perf_counter() - started) * 1000.0, 0.0
        )

    def _publish_diagnostics(
        self,
        reason: str,
        processing_ms: float,
        cloud_age_ms: float,
        *,
        result: PointSupportResult | None = None,
        input_points: int = 0,
        output_points: int = 0,
        stage_ms: dict[str, float] | None = None,
        artifact_rejected_points: int = 0,
        minimum_range_rejected_points: int = 0,
    ) -> None:
        level = DiagnosticStatus.OK
        message = reason
        if reason != "ok":
            level = DiagnosticStatus.ERROR
        elif processing_ms > float(self.get_parameter("processing_warn_ms").value):
            level = DiagnosticStatus.WARN
            message = "slow_filter"
        elif cloud_age_ms > float(self.get_parameter("cloud_age_warn_ms").value):
            level = DiagnosticStatus.WARN
            message = "old_cloud"
        values = {
            "reason": reason,
            "received_clouds": self._received_clouds,
            "published_clouds": self._published_clouds,
            "rejected_clouds": self._rejected_clouds,
            "input_points": input_points,
            "output_points": output_points,
            "processing_ms": "%.3f" % processing_ms,
            "cloud_age_ms": "%.3f" % cloud_age_ms,
        }
        values.update(self._metrics.values(float(self.get_parameter("lag_spike_ms").value)))
        if stage_ms is not None:
            values.update({key: "%.3f" % value for key, value in stage_ms.items()})
        if result is not None:
            values.update(
                {
                    "min_points_per_cell": result.stats.min_points_per_cell,
                    "occupied_cells": result.stats.occupied_cells,
                    "low_support_cells": result.stats.low_support_cells,
                    "low_support_points": result.stats.low_support_points,
                    "hard_rejected_points": result.stats.hard_rejected_points,
                    "artifact_rejected_points": artifact_rejected_points,
                    "minimum_range_rejected_points": (
                        minimum_range_rejected_points
                    ),
                    "global_low_support_cells": (
                        result.stats.global_low_support_cells
                    ),
                    "halo_candidate_cells": result.stats.halo_candidate_cells,
                    "halo_low_support_cells": (
                        result.stats.halo_low_support_cells
                    ),
                    "halo_low_support_points": (
                        result.stats.halo_low_support_points
                    ),
                    "artifact_filter_enabled": self._artifact_box is not None,
                    "artifact_box": (
                        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
                        % (
                            self._artifact_box.min_x_m,
                            self._artifact_box.max_x_m,
                            self._artifact_box.min_y_m,
                            self._artifact_box.max_y_m,
                            self._artifact_box.min_z_m,
                            self._artifact_box.max_z_m,
                        )
                        if self._artifact_box is not None
                        else "disabled"
                    ),
                    "artifact_halo_margin_m": self.get_parameter(
                        "artifact_halo_margin_m"
                    ).value,
                    "artifact_halo_bounds_xy": (
                        "%.3f,%.3f,%.3f,%.3f"
                        % (
                            self._artifact_halo_bounds.min_x_m,
                            self._artifact_halo_bounds.max_x_m,
                            self._artifact_halo_bounds.min_y_m,
                            self._artifact_halo_bounds.max_y_m,
                        )
                        if self._artifact_halo_bounds is not None
                        else "margin_fallback"
                    ),
                    "artifact_halo_min_points_per_cell": self.get_parameter(
                        "artifact_halo_min_points_per_cell"
                    ).value,
                }
            )
        status = DiagnosticStatus()
        status.name = str(self.get_parameter("diagnostic_name").value)
        status.hardware_id = str(self.get_parameter("sensor_label").value)
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointSupportFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
