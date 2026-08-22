"""Upstream AIRY PointCloud2 artifact filter for stock Nav2."""

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
    ArtifactPointFilterResult,
    artifact_configured_halo_cell_ids,
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
    FrontCostmapConfig,
    LocalCostmapConfig,
    obstacle_point_mask,
    parse_self_filter_boxes,
    validate_mapping_configs,
)
from wheelchair_navigation.mapping_diagnostics import MappingMetrics
from wheelchair_navigation.point_cloud import (
    point_cloud_to_arrays,
    select_point_cloud_records,
    transform_points,
    xyz_to_point_cloud,
)
from wheelchair_navigation.timing import cloud_timestamp_error


class ArtifactPointFilterNode(Node):
    """Remove calibrated AIRY artifacts without generating a costmap."""

    def __init__(self) -> None:
        super().__init__("artifact_point_filter")
        self._declare_parameters()
        self._configuration_error = None
        try:
            self._load_filter_configuration()
        except (TypeError, ValueError) as exc:
            self._configuration_error = str(exc)
            self.get_logger().error(
                "Invalid artifact-filter configuration: %s" % exc
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            # Match the AIRY publisher and avoid the measured best-effort
            # delivery gaps while retaining only the newest live cloud.
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
            str(self.get_parameter("artifact_filtered_cloud_topic").value),
            sensor_qos,
        )
        self._source_header_pub = self.create_publisher(
            Header,
            str(
                self.get_parameter(
                    "artifact_filtered_source_header_topic"
                ).value
            ),
            10,
        )
        self._rejected_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("artifact_rejected_points_topic").value),
            sensor_qos,
        )
        self._low_support_pub = self.create_publisher(
            PointCloud2,
            str(
                self.get_parameter("artifact_low_support_points_topic").value
            ),
            sensor_qos,
        )
        self._masks_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("artifact_masks_topic").value),
            marker_qos,
        )
        self._threshold_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("artifact_threshold_cells_topic").value),
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
        self._static_markers_published = False
        self._last_threshold_cell_ids = None
        self._last_threshold_publish_s = float("-inf")
        self._last_success_diagnostics_s = float("-inf")
        self.get_logger().warn(
            "AIRY artifact PointCloud2 filter active: %s -> %s. "
            "No costmaps or motion commands are published."
            % (
                self.get_parameter("lidar_topic").value,
                self.get_parameter("artifact_filtered_cloud_topic").value,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("lidar_topic", "/rslidar_points")
        self.declare_parameter(
            "artifact_filtered_cloud_topic",
            "/rslidar_points_artifact_filtered",
        )
        self.declare_parameter(
            "artifact_filtered_source_header_topic",
            "/artifact_filter/source_header",
        )
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter(
            "artifact_rejected_points_topic",
            "/artifact_filter/rejected_points",
        )
        self.declare_parameter(
            "artifact_low_support_points_topic",
            "/artifact_filter/low_support_points",
        )
        self.declare_parameter("artifact_masks_topic", "/artifact_filter/masks")
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
        self.declare_parameter("max_range_m", 4.0)
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
        self.declare_parameter("max_cloud_age_s", 1.0)
        self.declare_parameter("max_future_offset_s", 0.1)
        self.declare_parameter("validate_cloud_timestamps", True)
        self.declare_parameter("processing_warn_ms", 100.0)
        self.declare_parameter("cloud_age_warn_ms", 150.0)
        self.declare_parameter("latency_window_samples", 120)
        self.declare_parameter("lag_spike_ms", 150.0)
        self.declare_parameter("diagnostics_period_s", 1.0)

    def _load_filter_configuration(self) -> None:
        self._cached_map_config = self._map_config()
        self._cached_front_config = self._front_config()
        validate_mapping_configs(
            self._cached_map_config, self._cached_front_config
        )
        self._cached_boxes = parse_self_filter_boxes(
            self.get_parameter("self_filter_boxes").value,
            float(self.get_parameter("self_filter_padding_m").value),
        )
        self._cached_cells = parse_artifact_grid_cells(
            self.get_parameter("artifact_grid_mask_cells").value
        )
        self._cached_halo_spans = parse_artifact_grid_halo_spans(
            self.get_parameter("artifact_grid_halo_spans").value
        )
        validate_artifact_filter_frame(
            str(self.get_parameter("artifact_filter_frame").value),
            str(self.get_parameter("target_frame").value),
        )
        self._cached_threshold_cells = artifact_configured_halo_cell_ids(
            self._cached_halo_spans,
            self._cached_cells,
            self._cached_front_config,
        )
        if float(self.get_parameter("diagnostics_period_s").value) <= 0.0:
            raise ValueError("diagnostics_period_s must be positive")

    def _on_cloud(self, msg: PointCloud2) -> None:
        started = time.perf_counter()
        arrival_s = time.monotonic()
        stage_ms = {}
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
            self._reject("invalid_artifact_filter", started)
            return

        try:
            stage_started = time.perf_counter()
            cloud = point_cloud_to_arrays(msg)
            stage_ms["decode_ms"] = (
                time.perf_counter() - stage_started
            ) * 1000.0
        except ValueError as exc:
            self.get_logger().warn(
                "Invalid AIRY PointCloud2: %s" % exc,
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
                self._reject("missing_tf", started)
                return
            points_base = transform_points(points_base, transform)
        stage_ms["transform_ms"] = (
            time.perf_counter() - stage_started
        ) * 1000.0

        try:
            stage_started = time.perf_counter()
            eligible_mask, _ = obstacle_point_mask(
                points_base,
                self._cached_map_config,
                self._cached_boxes,
            )
            result = filter_artifact_points(
                points_base,
                eligible_mask,
                self._cached_boxes,
                self._cached_cells,
                self._cached_halo_spans,
                self._cached_front_config,
                min_points_per_cell=self.get_parameter(
                    "artifact_min_points_per_cell"
                ).value,
                global_min_points_per_cell=self.get_parameter(
                    "artifact_global_min_points_per_cell"
                ).value,
                threshold_candidate_cell_ids=self._cached_threshold_cells,
            )
            stage_ms["filter_ms"] = (
                time.perf_counter() - stage_started
            ) * 1000.0
            stage_started = time.perf_counter()
            filtered = select_point_cloud_records(msg, result.keep_mask)
            stage_ms["pack_ms"] = (
                time.perf_counter() - stage_started
            ) * 1000.0
        except (TypeError, ValueError) as exc:
            self.get_logger().error(
                "Artifact PointCloud2 filter suppressed output: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._reject("invalid_artifact_filter", started)
            return

        stage_started = time.perf_counter()
        self._filtered_pub.publish(filtered)
        # A small, reliable heartbeat exposes the original AIRY acquisition
        # stamp without making the supervisor deserialize another PointCloud2.
        # Publish only after the corresponding filtered cloud succeeds.
        self._source_header_pub.publish(filtered.header)
        stage_ms["publish_filtered_ms"] = (
            time.perf_counter() - stage_started
        ) * 1000.0
        stage_started = time.perf_counter()
        if self._rejected_pub.get_subscription_count() > 0:
            self._rejected_pub.publish(
                xyz_to_point_cloud(
                    cloud.xyz[result.artifact_rejected_mask], msg.header
                )
            )
        if self._low_support_pub.get_subscription_count() > 0:
            self._low_support_pub.publish(
                xyz_to_point_cloud(
                    cloud.xyz[result.support.low_support_mask], msg.header
                )
            )
        self._publish_markers(
            msg,
            result,
            self._cached_cells,
            self._cached_halo_spans,
            self._cached_front_config,
        )
        stage_ms["publish_debug_ms"] = (
            time.perf_counter() - stage_started
        ) * 1000.0
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
            )

    def _publish_markers(
        self,
        msg: PointCloud2,
        result: ArtifactPointFilterResult,
        cells,
        halo_spans,
        front_config: FrontCostmapConfig,
    ) -> None:
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = str(self.get_parameter("artifact_filter_frame").value)
        if not self._static_markers_published:
            self._masks_pub.publish(
                build_artifact_grid_markers(
                    header, cells, halo_spans, front_config
                )
            )
            self._static_markers_published = True
        low_support_ids = tuple(
            int(value) for value in result.support.low_support_cell_ids
        )
        if low_support_ids == self._last_threshold_cell_ids:
            return
        now_s = time.monotonic()
        if now_s - self._last_threshold_publish_s < 1.0:
            return
        self._threshold_pub.publish(
            build_artifact_threshold_cell_markers(
                header,
                front_config,
                result.support.candidate_cell_ids,
                result.support.low_support_cell_ids,
            )
        )
        self._last_threshold_cell_ids = low_support_ids
        self._last_threshold_publish_s = now_s

    def _map_config(self) -> LocalCostmapConfig:
        return LocalCostmapConfig(
            size_m=float(self.get_parameter("size_m").value),
            resolution_m=float(self.get_parameter("resolution_m").value),
            min_height_m=float(self.get_parameter("min_height_m").value),
            max_height_m=float(self.get_parameter("max_height_m").value),
            min_range_m=float(self.get_parameter("min_range_m").value),
            max_range_m=float(self.get_parameter("max_range_m").value),
        )

    def _front_config(self) -> FrontCostmapConfig:
        return FrontCostmapConfig(
            length_m=float(self.get_parameter("front_length_m").value),
            width_m=float(self.get_parameter("front_width_m").value),
            resolution_m=float(
                self.get_parameter("front_resolution_m").value
            ),
            fov_deg=float(self.get_parameter("front_fov_deg").value),
        )

    def _reject(self, reason: str, started: float) -> None:
        self._rejected_clouds += 1
        self._publish_diagnostics(
            reason,
            (time.perf_counter() - started) * 1000.0,
            0.0,
        )

    def _publish_diagnostics(
        self,
        reason: str,
        processing_ms: float,
        cloud_age_ms: float,
        *,
        result: ArtifactPointFilterResult | None = None,
        input_points: int = 0,
        output_points: int = 0,
        stage_ms: dict[str, float] | None = None,
    ) -> None:
        level = DiagnosticStatus.OK
        message = reason
        if reason != "ok":
            level = DiagnosticStatus.ERROR
        elif processing_ms > float(
            self.get_parameter("processing_warn_ms").value
        ):
            level = DiagnosticStatus.WARN
            message = "slow_filter"
        elif cloud_age_ms > float(
            self.get_parameter("cloud_age_warn_ms").value
        ):
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
        values.update(
            self._metrics.values(
                float(self.get_parameter("lag_spike_ms").value)
            )
        )
        if stage_ms is not None:
            values.update(
                {
                    key: "%.3f" % value
                    for key, value in stage_ms.items()
                }
            )
        if result is not None:
            values.update(
                {
                    "self_rejected_points": int(
                        np.count_nonzero(result.self_rejected_mask)
                    ),
                    "artifact_rejected_points": result.artifact_stats.unique_rejected_points,
                    "low_support_points": result.support.stats.low_support_points,
                    "global_min_points_per_cell": result.support.stats.global_min_points_per_cell,
                    "halo_min_points_per_cell": result.support.stats.min_points_per_cell,
                }
            )
        status = DiagnosticStatus()
        status.name = "wheelchair_navigation/artifact_point_filter"
        status.hardware_id = "AIRY"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArtifactPointFilterNode()
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
