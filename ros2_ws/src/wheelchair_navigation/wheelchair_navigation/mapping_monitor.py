"""Compact terminal monitor for wheelchair local-mapping diagnostics."""

from __future__ import annotations

from collections import Counter
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


MAPPING_STATUS_NAME = "wheelchair_navigation/local_costmap"


def diagnostic_values(status: DiagnosticStatus) -> dict[str, str]:
    """Return diagnostic key/value pairs as a normal dictionary."""

    return {item.key: item.value for item in status.values}


def find_mapping_status(msg: DiagnosticArray) -> DiagnosticStatus | None:
    """Find the local mapper status without relying on array position."""

    return next(
        (status for status in msg.status if status.name == MAPPING_STATUS_NAME),
        None,
    )


def duplicate_fully_qualified_names(
    names_and_namespaces: list[tuple[str, str]],
) -> list[str]:
    """Return sorted duplicate ROS node names including their namespaces."""

    qualified = []
    for name, namespace in names_and_namespaces:
        prefix = namespace.rstrip("/")
        qualified.append("%s/%s" % (prefix, name) if prefix else "/" + name)
    counts = Counter(qualified)
    return sorted(name for name, count in counts.items() if count > 1)


def format_mapping_status(status: DiagnosticStatus) -> str:
    """Format one mapper status as a compact, stable terminal line."""

    values = diagnostic_values(status)
    level_names = {
        DiagnosticStatus.OK: "OK",
        DiagnosticStatus.WARN: "WARN",
        DiagnosticStatus.ERROR: "ERROR",
        DiagnosticStatus.STALE: "STALE",
    }
    return (
        "state=%s message=%s rate=%sHz process=%sms p95=%sms max=%sms "
        "point_filter=%sms ghost_filter=%sms raw=%s filtered=%s rejected=%s "
        "self_filtered_points=%s lag_spikes=%s"
        % (
            level_names.get(status.level, str(status.level)),
            status.message or "-",
            values.get("effective_rate_hz", "-"),
            values.get("processing_ms", "-"),
            values.get("processing_p95_ms", "-"),
            values.get("processing_max_ms", "-"),
            values.get("filter_ms", "-"),
            values.get("ghost_filter_ms", "-"),
            values.get("ghost_raw_cells", "-"),
            values.get("ghost_filtered_cells", "-"),
            values.get("ghost_rejected_cells", "-"),
            values.get("self_filtered_points", "-"),
            values.get("lag_spike_count", "-"),
        )
    )


class MappingMonitor(Node):
    """Print readable mapper health without shell pipelines."""

    def __init__(self) -> None:
        super().__init__("mapping_monitor")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("print_period_s", 1.0)
        self.declare_parameter("no_data_timeout_s", 2.5)

        print_period_s = float(self.get_parameter("print_period_s").value)
        no_data_timeout_s = float(
            self.get_parameter("no_data_timeout_s").value
        )
        if print_period_s <= 0.0 or no_data_timeout_s <= 0.0:
            raise ValueError("monitor periods must be positive")

        self._no_data_timeout_s = no_data_timeout_s
        self._latest_status: DiagnosticStatus | None = None
        self._latest_monotonic: float | None = None
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics,
            10,
        )
        self.create_timer(print_period_s, self._print_status)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        status = find_mapping_status(msg)
        if status is None:
            return
        self._latest_status = status
        self._latest_monotonic = time.monotonic()

    def _print_status(self) -> None:
        now = time.monotonic()
        age_s = (
            None
            if self._latest_monotonic is None
            else now - self._latest_monotonic
        )
        if age_s is None or age_s > self._no_data_timeout_s:
            duplicates = duplicate_fully_qualified_names(
                self.get_node_names_and_namespaces()
            )
            duplicate_text = ",".join(duplicates) if duplicates else "none"
            age_text = "never" if age_s is None else "%.1fs" % age_s
            print(
                "state=NO_DIAGNOSTICS age=%s duplicate_nodes=%s "
                "hint=check_/rslidar_points_and_restart_old_launches"
                % (age_text, duplicate_text),
                flush=True,
            )
            return
        print(format_mapping_status(self._latest_status), flush=True)


def main() -> int:
    rclpy.init()
    node = MappingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
