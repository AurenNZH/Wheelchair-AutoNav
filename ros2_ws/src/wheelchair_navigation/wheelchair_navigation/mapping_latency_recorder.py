"""Record end-to-end front-costmap latency without commanding motion."""

from __future__ import annotations

import csv
import math
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from wheelchair_navigation.mapping_monitor import (
    diagnostic_values,
    find_mapping_status,
)


SUPERVISOR_STATUS_NAME = "wheelchair_shared_control/safety_supervisor"
MAPPER_FIELDS = (
    "cloud_arrival_age_ms",
    "front_publish_age_ms",
    "processing_ms",
    "processing_p95_ms",
    "processing_p99_ms",
    "front_publish_age_p99_ms",
    "front_period_max_ms",
    "effective_rate_hz",
    "front_publish_rate_hz",
    "decode_ms",
    "transform_ms",
    "filter_ms",
    "front_select_ms",
    "front_raster_ms",
    "publish_front_ms",
    "raw_raster_ms",
    "publish_raw_ms",
    "artifact_filter_ms",
    "publish_artifact_ms",
)
SUPERVISOR_FIELDS = (
    "map_age_ms",
    "map_age_at_receipt_ms",
    "map_receipt_age_ms",
    "map_receipt_interval_ms",
    "map_receipt_interval_p99_ms",
    "map_receipt_rate_hz",
    "stale_map_receipt_count",
    "stale_map_event_count",
)
CSV_FIELDS = (
    "wall_time_s",
    "source_stamp_ns",
    "map_age_ms",
    "arrival_interval_ms",
    "stamp_interval_ms",
    "over_250_ms",
    "over_300_ms",
) + tuple("mapper_" + name for name in MAPPER_FIELDS) + tuple(
    "supervisor_" + name for name in SUPERVISOR_FIELDS
)


def percentile(values: list[float], level: float) -> float | None:
    """Return one percentile for finite samples, or None when unavailable."""

    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return None
    return float(np.percentile(finite, level))


def latency_summary(
    ages_ms: list[float],
    intervals_ms: list[float],
    *,
    target_ms: float = 250.0,
    deadline_ms: float = 300.0,
) -> dict[str, float | int | bool | None]:
    """Build the stable acceptance summary used by tests and the CLI."""

    return {
        "count": len(ages_ms),
        "age_p50_ms": percentile(ages_ms, 50),
        "age_p95_ms": percentile(ages_ms, 95),
        "age_p99_ms": percentile(ages_ms, 99),
        "age_max_ms": max(ages_ms) if ages_ms else None,
        "interval_p99_ms": percentile(intervals_ms, 99),
        "interval_max_ms": max(intervals_ms) if intervals_ms else None,
        "over_target_count": sum(age > target_ms for age in ages_ms),
        "over_deadline_count": sum(age > deadline_ms for age in ages_ms),
        "passes": bool(ages_ms)
        and percentile(ages_ms, 99) <= target_ms
        and not any(age > deadline_ms for age in ages_ms),
    }


def find_status(
    msg: DiagnosticArray, status_name: str
) -> DiagnosticStatus | None:
    return next(
        (status for status in msg.status if status.name == status_name),
        None,
    )


class MappingLatencyRecorder(Node):
    """Write one low-overhead CSV row for every received front map."""

    def __init__(self) -> None:
        super().__init__("mapping_latency_recorder")
        self.declare_parameter("front_costmap_topic", "/front_costmap")
        self.declare_parameter("mapping_diagnostics_topic", "/diagnostics")
        self.declare_parameter(
            "supervisor_diagnostics_topic", "/shared_control/diagnostics"
        )
        self.declare_parameter(
            "output_csv", "/tmp/front_costmap_latency.csv"
        )
        self.declare_parameter("print_period_s", 1.0)
        self.declare_parameter("warmup_s", 15.0)
        self.declare_parameter("duration_s", 120.0)
        self.declare_parameter("target_age_ms", 250.0)
        self.declare_parameter("deadline_age_ms", 300.0)

        self._target_ms = float(self.get_parameter("target_age_ms").value)
        self._deadline_ms = float(
            self.get_parameter("deadline_age_ms").value
        )
        print_period_s = float(self.get_parameter("print_period_s").value)
        self._warmup_s = float(self.get_parameter("warmup_s").value)
        self._duration_s = float(self.get_parameter("duration_s").value)
        if (
            self._target_ms <= 0.0
            or self._deadline_ms < self._target_ms
            or print_period_s <= 0.0
            or self._warmup_s < 0.0
            or self._duration_s <= 0.0
        ):
            raise ValueError("invalid latency recorder thresholds")

        output = Path(str(self.get_parameter("output_csv").value))
        self._file = output.open("x", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
        self._writer.writeheader()
        self._ages_ms: list[float] = []
        self._intervals_ms: list[float] = []
        self._last_arrival_monotonic = None
        self._last_stamp_ns = None
        self._latest_mapper = {}
        self._latest_supervisor = {}
        self._started_monotonic = time.monotonic()

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("front_costmap_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("mapping_diagnostics_topic").value),
            self._on_mapping_diagnostics,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("supervisor_diagnostics_topic").value),
            self._on_supervisor_diagnostics,
            10,
        )
        self.create_timer(print_period_s, self._print_status)
        self.get_logger().info("Recording front-map latency to %s" % output)

    def _on_mapping_diagnostics(self, msg: DiagnosticArray) -> None:
        status = find_mapping_status(msg)
        if status is not None:
            self._latest_mapper = diagnostic_values(status)

    def _on_supervisor_diagnostics(self, msg: DiagnosticArray) -> None:
        status = find_status(msg, SUPERVISOR_STATUS_NAME)
        if status is not None:
            self._latest_supervisor = diagnostic_values(status)

    def _on_map(self, msg: OccupancyGrid) -> None:
        now_monotonic = time.monotonic()
        if now_monotonic - self._started_monotonic < self._warmup_s:
            return
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        age_ms = max(
            0.0,
            (self.get_clock().now().nanoseconds - stamp_ns) / 1e6,
        )
        interval_ms = self._difference_ms(
            now_monotonic, self._last_arrival_monotonic, scale=1000.0
        )
        stamp_interval_ms = self._difference_ms(
            stamp_ns, self._last_stamp_ns, scale=1e-6
        )
        self._last_arrival_monotonic = now_monotonic
        self._last_stamp_ns = stamp_ns
        self._ages_ms.append(age_ms)
        if interval_ms is not None:
            self._intervals_ms.append(interval_ms)

        row = {
            "wall_time_s": "%.6f" % time.time(),
            "source_stamp_ns": stamp_ns,
            "map_age_ms": "%.3f" % age_ms,
            "arrival_interval_ms": self._optional(interval_ms),
            "stamp_interval_ms": self._optional(stamp_interval_ms),
            "over_250_ms": int(age_ms > self._target_ms),
            "over_300_ms": int(age_ms > self._deadline_ms),
        }
        row.update(
            {
                "mapper_" + key: self._latest_mapper.get(key, "")
                for key in MAPPER_FIELDS
            }
        )
        row.update(
            {
                "supervisor_" + key: self._latest_supervisor.get(key, "")
                for key in SUPERVISOR_FIELDS
            }
        )
        self._writer.writerow(row)

    @staticmethod
    def _difference_ms(current, previous, *, scale: float) -> float | None:
        if previous is None:
            return None
        return max(0.0, float(current - previous) * scale)

    @staticmethod
    def _optional(value: float | None) -> str:
        return "" if value is None else "%.3f" % value

    def _print_status(self) -> None:
        elapsed_s = time.monotonic() - self._started_monotonic
        if elapsed_s < self._warmup_s:
            print(
                "front_map=WARMUP remaining=%.1fs"
                % (self._warmup_s - elapsed_s),
                flush=True,
            )
            return
        summary = latency_summary(
            self._ages_ms,
            self._intervals_ms,
            target_ms=self._target_ms,
            deadline_ms=self._deadline_ms,
        )
        if not summary["count"]:
            print("front_map=WAITING", flush=True)
            self._finish_if_expired(elapsed_s)
            return
        print(
            "front_map=count:%d p50:%0.1fms p95:%0.1fms p99:%0.1fms "
            "max:%0.1fms over_300:%d"
            % (
                summary["count"],
                summary["age_p50_ms"],
                summary["age_p95_ms"],
                summary["age_p99_ms"],
                summary["age_max_ms"],
                summary["over_deadline_count"],
            ),
            flush=True,
        )
        self._file.flush()
        self._finish_if_expired(elapsed_s)

    def _finish_if_expired(self, elapsed_s: float) -> None:
        if elapsed_s >= self._warmup_s + self._duration_s:
            self.close()
            if rclpy.ok():
                rclpy.shutdown()

    def close(self) -> None:
        if self._file is None:
            return
        summary = latency_summary(
            self._ages_ms,
            self._intervals_ms,
            target_ms=self._target_ms,
            deadline_ms=self._deadline_ms,
        )
        if summary["count"]:
            print(
                "front_map=FINAL pass=%s p99=%.1fms max=%.1fms over_300=%d"
                % (
                    summary["passes"],
                    summary["age_p99_ms"],
                    summary["age_max_ms"],
                    summary["over_deadline_count"],
                ),
                flush=True,
            )
        self._file.close()
        self._file = None


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = MappingLatencyRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
