"""Observe filtered-cloud and Nav2 continuity without inventing map latency."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2


@dataclass
class ArrivalStats:
    """Arrival-rate and gap statistics for one ROS stream."""

    arrivals_s: list[float] = field(default_factory=list)
    intervals_ms: list[float] = field(default_factory=list)

    def observe(self, arrival_s: float) -> None:
        if self.arrivals_s and arrival_s > self.arrivals_s[-1]:
            self.intervals_ms.append(
                (arrival_s - self.arrivals_s[-1]) * 1000.0
            )
        self.arrivals_s.append(float(arrival_s))

    @property
    def count(self) -> int:
        return len(self.arrivals_s)

    @property
    def rate_hz(self) -> float | None:
        if len(self.arrivals_s) < 2:
            return None
        elapsed = self.arrivals_s[-1] - self.arrivals_s[0]
        if elapsed <= 0.0:
            return None
        return (len(self.arrivals_s) - 1) / elapsed

    @property
    def maximum_gap_ms(self) -> float | None:
        return max(self.intervals_ms) if self.intervals_ms else None


def percentile(values: list[float], level: float) -> float | None:
    """Return a linearly interpolated percentile without extra dependencies."""

    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return None
    position = (len(finite) - 1) * float(level) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] + fraction * (finite[upper] - finite[lower])


def continuity_summary(
    cloud: ArrivalStats,
    costmap: ArrivalStats,
    *,
    minimum_map_rate_hz: float,
    maximum_gap_ms: float,
) -> dict[str, float | int | bool | None]:
    """Summarize publication continuity; this is not a latency verdict."""

    map_rate = costmap.rate_hz
    cloud_gap = cloud.maximum_gap_ms
    map_gap = costmap.maximum_gap_ms
    return {
        "cloud_count": cloud.count,
        "map_count": costmap.count,
        "cloud_rate_hz": cloud.rate_hz,
        "map_rate_hz": map_rate,
        "cloud_gap_max_ms": cloud_gap,
        "map_gap_max_ms": map_gap,
        "passes": (
            cloud.count > 1
            and costmap.count > 1
            and map_rate is not None
            and map_rate >= minimum_map_rate_hz
            and cloud_gap is not None
            and cloud_gap <= maximum_gap_ms
            and map_gap is not None
            and map_gap <= maximum_gap_ms
        ),
    }


def filtered_continuity_summary(
    raw_cloud: ArrivalStats,
    filtered_cloud: ArrivalStats,
    costmap: ArrivalStats,
    *,
    minimum_map_rate_hz: float,
    maximum_gap_ms: float,
    minimum_filtered_ratio: float,
) -> dict[str, float | int | bool | None]:
    """Summarize the upstream filter and map publication continuity."""

    raw_rate = raw_cloud.rate_hz
    filtered_rate = filtered_cloud.rate_hz
    map_rate = costmap.rate_hz
    map_gap = costmap.maximum_gap_ms
    ratio = (
        filtered_rate / raw_rate
        if raw_rate is not None
        and raw_rate > 0.0
        and filtered_rate is not None
        else None
    )
    return {
        "raw_count": raw_cloud.count,
        "filtered_count": filtered_cloud.count,
        "map_count": costmap.count,
        "raw_rate_hz": raw_rate,
        "filtered_rate_hz": filtered_rate,
        "filtered_rate_ratio": ratio,
        "map_rate_hz": map_rate,
        "raw_gap_max_ms": raw_cloud.maximum_gap_ms,
        "filtered_gap_max_ms": filtered_cloud.maximum_gap_ms,
        "map_gap_max_ms": map_gap,
        "passes": (
            raw_cloud.count > 1
            and filtered_cloud.count > 1
            and costmap.count > 1
            and ratio is not None
            and ratio >= minimum_filtered_ratio
            and map_rate is not None
            and map_rate >= minimum_map_rate_hz
            and map_gap is not None
            and map_gap <= maximum_gap_ms
        ),
    }


def artifact_filter_diagnostic_values(
    msg: DiagnosticArray,
) -> dict[str, str] | None:
    """Extract the upstream filter's latest self-reported counters."""

    for status in msg.status:
        if status.name == "wheelchair_navigation/artifact_point_filter":
            return {item.key: item.value for item in status.values}
    return None


def _number(value: float | None) -> str:
    return "none" if value is None else "%.2f" % value


class Nav2CostmapMonitor(Node):
    """Measure raw, artifact-filtered, and Nav2 streams side by side."""

    def __init__(self) -> None:
        super().__init__("nav2_costmap_monitor")
        self.declare_parameter("cloud_topic", "/rslidar_points")
        self.declare_parameter(
            "filtered_cloud_topic", "/rslidar_points_artifact_filtered"
        )
        self.declare_parameter("costmap_topic", "/nav2_front_costmap")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("warmup_s", 10.0)
        self.declare_parameter("duration_s", 360.0)
        self.declare_parameter("print_period_s", 5.0)
        self.declare_parameter("minimum_map_rate_hz", 5.7)
        self.declare_parameter("maximum_gap_ms", 300.0)
        self.declare_parameter("minimum_filtered_ratio", 0.9)

        self._warmup_s = float(self.get_parameter("warmup_s").value)
        self._duration_s = float(self.get_parameter("duration_s").value)
        print_period_s = float(self.get_parameter("print_period_s").value)
        self._minimum_map_rate_hz = float(
            self.get_parameter("minimum_map_rate_hz").value
        )
        self._maximum_gap_ms = float(
            self.get_parameter("maximum_gap_ms").value
        )
        self._minimum_filtered_ratio = float(
            self.get_parameter("minimum_filtered_ratio").value
        )
        if (
            self._warmup_s < 0.0
            or self._duration_s <= 0.0
            or print_period_s <= 0.0
            or self._minimum_map_rate_hz <= 0.0
            or self._maximum_gap_ms <= 0.0
            or self._minimum_filtered_ratio <= 0.0
            or self._minimum_filtered_ratio > 1.0
        ):
            raise ValueError("invalid Nav2 monitor timing parameters")

        self._started_s = time.monotonic()
        self._cloud = ArrivalStats()
        self._filtered = ArrivalStats()
        self._map = ArrivalStats()
        self._cloud_ages_ms: list[float] = []
        self._filtered_ages_ms: list[float] = []
        self._filter_delays_ms: list[float] = []
        self._raw_arrivals_by_stamp: dict[int, float] = {}
        self._zero_map_stamps = 0
        self._nonzero_map_stamps = 0
        self._latest_occupied_cells = 0
        self._filter_diagnostics: dict[str, str] = {}
        self._finalized = False

        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cloud_topic").value),
            self._on_cloud,
            cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("filtered_cloud_topic").value),
            self._on_filtered_cloud,
            cloud_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics,
            10,
        )
        self.create_timer(print_period_s, self._report)
        self.get_logger().warn(
            "Filtered Nav2 continuity monitor started. Raw-to-filter delay "
            "uses matching cloud stamps; map header time is not treated as "
            "LiDAR acquisition time. warmup=%.1fs duration=%.1fs"
            % (self._warmup_s, self._duration_s)
        )

    def _capturing(self, arrival_s: float) -> bool:
        return arrival_s - self._started_s >= self._warmup_s

    def _on_cloud(self, msg: PointCloud2) -> None:
        arrival_s = time.monotonic()
        if not self._capturing(arrival_s):
            return
        self._cloud.observe(arrival_s)
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns > 0:
            self._raw_arrivals_by_stamp[stamp_ns] = arrival_s
            while len(self._raw_arrivals_by_stamp) > 512:
                oldest = next(iter(self._raw_arrivals_by_stamp))
                self._raw_arrivals_by_stamp.pop(oldest)
            age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
            if age_ms >= 0.0:
                self._cloud_ages_ms.append(age_ms)

    def _on_filtered_cloud(self, msg: PointCloud2) -> None:
        arrival_s = time.monotonic()
        if not self._capturing(arrival_s):
            return
        self._filtered.observe(arrival_s)
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= 0:
            return
        raw_arrival_s = self._raw_arrivals_by_stamp.pop(stamp_ns, None)
        if raw_arrival_s is not None and arrival_s >= raw_arrival_s:
            self._filter_delays_ms.append(
                (arrival_s - raw_arrival_s) * 1000.0
            )
        age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
        if age_ms >= 0.0:
            self._filtered_ages_ms.append(age_ms)

    def _on_map(self, msg: OccupancyGrid) -> None:
        arrival_s = time.monotonic()
        if not self._capturing(arrival_s):
            return
        self._map.observe(arrival_s)
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp_ns <= 0:
            self._zero_map_stamps += 1
        else:
            self._nonzero_map_stamps += 1
        self._latest_occupied_cells = sum(value >= 100 for value in msg.data)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        values = artifact_filter_diagnostic_values(msg)
        if values is not None:
            self._filter_diagnostics = values

    def _summary(self):
        summary = filtered_continuity_summary(
            self._cloud,
            self._filtered,
            self._map,
            minimum_map_rate_hz=self._minimum_map_rate_hz,
            maximum_gap_ms=self._maximum_gap_ms,
            minimum_filtered_ratio=self._minimum_filtered_ratio,
        )
        received = self._diagnostic_int("received_clouds")
        published = self._diagnostic_int("published_clouds")
        rejected = self._diagnostic_int("rejected_clouds")
        node_ratio = (
            published / received
            if received is not None
            and received > 0
            and published is not None
            else None
        )
        summary["filter_node_received"] = received
        summary["filter_node_published"] = published
        summary["filter_node_rejected"] = rejected
        summary["filter_node_ratio"] = node_ratio
        if node_ratio is not None:
            summary["passes"] = (
                summary["raw_count"] > 1
                and summary["filtered_count"] > 1
                and summary["map_count"] > 1
                and summary["map_rate_hz"] is not None
                and summary["map_rate_hz"] >= self._minimum_map_rate_hz
                and summary["map_gap_max_ms"] is not None
                and summary["map_gap_max_ms"] <= self._maximum_gap_ms
                and node_ratio >= self._minimum_filtered_ratio
                and rejected == 0
            )
        return summary

    def _diagnostic_int(self, key: str) -> int | None:
        try:
            return int(self._filter_diagnostics[key])
        except (KeyError, TypeError, ValueError):
            return None

    @property
    def finalized(self) -> bool:
        """Return whether the requested capture has emitted its final report."""

        return self._finalized

    def _report(self) -> None:
        elapsed_s = time.monotonic() - self._started_s
        if elapsed_s < self._warmup_s:
            self.get_logger().info(
                "NAV2_MAP WARMUP remaining_s=%.1f"
                % (self._warmup_s - elapsed_s)
            )
            return

        capture_s = elapsed_s - self._warmup_s
        summary = self._summary()
        final = capture_s >= self._duration_s
        label = "FINAL" if final else "ACTIVE"
        self.get_logger().info(
            "NAV2_MAP %s pass=%s raw_count=%d filtered_count=%d map_count=%d "
            "raw_rate_hz=%s filtered_rate_hz=%s filtered_ratio=%s "
            "map_rate_hz=%s raw_gap_max_ms=%s filtered_gap_max_ms=%s "
            "map_gap_max_ms=%s raw_age_p99_ms=%s filtered_age_p99_ms=%s "
            "filter_delay_p50_ms=%s filter_delay_p95_ms=%s "
            "filter_delay_p99_ms=%s filter_delay_samples=%d occupied_cells=%d "
            "filter_node_received=%s filter_node_published=%s "
            "filter_node_rejected=%s filter_node_ratio=%s "
            "filter_processing_p95_ms=%s "
            "map_stamps_zero=%d map_stamps_nonzero=%d"
            % (
                label,
                str(summary["passes"]).lower(),
                summary["raw_count"],
                summary["filtered_count"],
                summary["map_count"],
                _number(summary["raw_rate_hz"]),
                _number(summary["filtered_rate_hz"]),
                _number(summary["filtered_rate_ratio"]),
                _number(summary["map_rate_hz"]),
                _number(summary["raw_gap_max_ms"]),
                _number(summary["filtered_gap_max_ms"]),
                _number(summary["map_gap_max_ms"]),
                _number(percentile(self._cloud_ages_ms, 99.0)),
                _number(percentile(self._filtered_ages_ms, 99.0)),
                _number(percentile(self._filter_delays_ms, 50.0)),
                _number(percentile(self._filter_delays_ms, 95.0)),
                _number(percentile(self._filter_delays_ms, 99.0)),
                len(self._filter_delays_ms),
                self._latest_occupied_cells,
                str(summary["filter_node_received"]),
                str(summary["filter_node_published"]),
                str(summary["filter_node_rejected"]),
                _number(summary["filter_node_ratio"]),
                self._filter_diagnostics.get("processing_p95_ms", "none"),
                self._zero_map_stamps,
                self._nonzero_map_stamps,
            )
        )
        if final and not self._finalized:
            self._finalized = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2CostmapMonitor()
    try:
        # Calling rclpy.shutdown() from a Foxy timer callback can deadlock the
        # executor. Let the outer spin loop observe completion and shut down.
        while rclpy.ok() and not node.finalized:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
