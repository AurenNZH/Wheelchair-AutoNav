"""Observe stock Nav2 costmap continuity without estimating sensor latency."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

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


def _number(value: float | None) -> str:
    return "none" if value is None else "%.2f" % value


class Nav2CostmapMonitor(Node):
    """Measure raw cloud and stock Nav2 map continuity side by side."""

    def __init__(self) -> None:
        super().__init__("nav2_costmap_monitor")
        self.declare_parameter("cloud_topic", "/rslidar_points")
        self.declare_parameter("costmap_topic", "/nav2_front_costmap")
        self.declare_parameter("warmup_s", 10.0)
        self.declare_parameter("duration_s", 360.0)
        self.declare_parameter("print_period_s", 5.0)
        self.declare_parameter("minimum_map_rate_hz", 9.0)
        self.declare_parameter("maximum_gap_ms", 300.0)

        self._warmup_s = float(self.get_parameter("warmup_s").value)
        self._duration_s = float(self.get_parameter("duration_s").value)
        print_period_s = float(self.get_parameter("print_period_s").value)
        self._minimum_map_rate_hz = float(
            self.get_parameter("minimum_map_rate_hz").value
        )
        self._maximum_gap_ms = float(
            self.get_parameter("maximum_gap_ms").value
        )
        if (
            self._warmup_s < 0.0
            or self._duration_s <= 0.0
            or print_period_s <= 0.0
            or self._minimum_map_rate_hz <= 0.0
            or self._maximum_gap_ms <= 0.0
        ):
            raise ValueError("invalid Nav2 monitor timing parameters")

        self._started_s = time.monotonic()
        self._cloud = ArrivalStats()
        self._map = ArrivalStats()
        self._cloud_ages_ms: list[float] = []
        self._zero_map_stamps = 0
        self._nonzero_map_stamps = 0
        self._latest_occupied_cells = 0
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
            OccupancyGrid,
            str(self.get_parameter("costmap_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_timer(print_period_s, self._report)
        self.get_logger().warn(
            "Stock Nav2 continuity monitor started. Map header time is not "
            "treated as LiDAR acquisition time; no latency or safety claim "
            "is made. warmup=%.1fs duration=%.1fs"
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
            age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
            if age_ms >= 0.0:
                self._cloud_ages_ms.append(age_ms)

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

    def _summary(self):
        return continuity_summary(
            self._cloud,
            self._map,
            minimum_map_rate_hz=self._minimum_map_rate_hz,
            maximum_gap_ms=self._maximum_gap_ms,
        )

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
            "NAV2_MAP %s pass=%s cloud_count=%d map_count=%d "
            "cloud_rate_hz=%s map_rate_hz=%s cloud_gap_max_ms=%s "
            "map_gap_max_ms=%s cloud_age_p99_ms=%s occupied_cells=%d "
            "map_stamps_zero=%d map_stamps_nonzero=%d"
            % (
                label,
                str(summary["passes"]).lower(),
                summary["cloud_count"],
                summary["map_count"],
                _number(summary["cloud_rate_hz"]),
                _number(summary["map_rate_hz"]),
                _number(summary["cloud_gap_max_ms"]),
                _number(summary["map_gap_max_ms"]),
                _number(percentile(self._cloud_ages_ms, 99.0)),
                self._latest_occupied_cells,
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
