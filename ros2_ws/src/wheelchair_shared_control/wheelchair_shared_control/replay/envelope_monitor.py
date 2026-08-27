"""Concise pipeline-state monitor for recorded-map safety decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope

from wheelchair_shared_control.operator_intent import (
    RELEASED,
    classify_normalized_axes,
    intent_label,
)


WAITING = "WAITING"
READY = "READY"
STALE = "STALE"


@dataclass(frozen=True)
class MapPipelineState:
    status: str
    age_s: float | None = None

    @property
    def signature(self) -> str:
        """Return stable fields so increasing ages do not create log spam."""

        return self.status


def map_pipeline_state(
    *,
    merged_received_s: float | None,
    now_s: float,
    timeout_s: float,
) -> MapPipelineState:
    """Classify merged-map receipt without inspecting production timestamps."""

    if not math.isfinite(timeout_s) or timeout_s <= 0.0:
        raise ValueError("map timeout must be finite and positive")
    if merged_received_s is None:
        return MapPipelineState(WAITING)

    merged_age_s = max(0.0, now_s - merged_received_s)
    return MapPipelineState(
        STALE if merged_age_s > timeout_s else READY,
        merged_age_s,
    )


def format_map_state(state: MapPipelineState) -> str:
    if state.status == WAITING:
        return "[MAP] WAITING for merged_costmap"
    if state.status == READY:
        return "[MAP] READY"
    return "[MAP] STALE merged_costmap age=%.2fs" % state.age_s


def intent_signature(msg: OperatorIntent) -> tuple[bool, int, float, float]:
    """Ignore timestamp and sequence heartbeats when identifying intent changes."""

    intent_class, lateral, longitudinal = _intent_view(msg)
    return (
        bool(msg.deadman),
        intent_class,
        lateral,
        longitudinal,
    )


def format_intent(msg: OperatorIntent) -> str:
    intent_class, lateral, longitudinal = _intent_view(msg)
    if not msg.deadman or intent_class == RELEASED:
        return "[INTENT] RELEASED"
    try:
        label = intent_label(intent_class).upper()
    except ValueError:
        label = "UNKNOWN(%d)" % intent_class
    heading_deg = math.degrees(
        math.atan2(lateral, longitudinal)
    )
    return "[INTENT] %s lateral=%.3f longitudinal=%.3f angle=%.1fdeg" % (
        label,
        lateral,
        longitudinal,
        heading_deg,
    )


def _intent_view(msg: OperatorIntent) -> tuple[int, float, float]:
    """Return v2 axes, deriving them for existing ROS-only publishers."""

    intent_class = int(msg.intent_class)
    lateral = float(msg.lateral)
    longitudinal = float(msg.longitudinal)
    if (
        lateral == 0.0
        and longitudinal == 0.0
        and intent_class == RELEASED
        and bool(msg.deadman)
    ):
        longitudinal = float(msg.forward)
        lateral = float(msg.steering) * longitudinal
        try:
            intent_class = classify_normalized_axes(
                lateral, longitudinal
            ).intent_class
        except ValueError:
            intent_class = -1
    return intent_class, lateral, longitudinal


def decision_name(decision: int) -> str:
    names = {
        int(SafetyEnvelope.STOP): "STOP",
        int(SafetyEnvelope.SLOW): "SLOW",
        int(SafetyEnvelope.CLEAR): "CLEAR",
    }
    return names.get(int(decision), "UNKNOWN(%d)" % int(decision))


def envelope_signature(msg: SafetyEnvelope) -> tuple[int, str, float, float]:
    """Return decision fields that matter to an operator-facing transition."""

    return (
        int(msg.decision),
        str(msg.reason),
        float(msg.permitted_forward),
        float(msg.permitted_steering),
    )


def format_envelope(msg: SafetyEnvelope) -> str:
    return "[DECISION] %s reason=%s permitted_forward=%.3f" % (
        decision_name(msg.decision),
        msg.reason,
        msg.permitted_forward,
    )


class SafetyEnvelopeMonitorNode(Node):
    """Report semantic intent, map health, and decision changes only."""

    def __init__(self) -> None:
        super().__init__("safety_envelope_monitor")
        self.declare_parameter("intent_topic", "/operator_intent")
        self.declare_parameter("envelope_topic", "/safety_envelope")
        self.declare_parameter("merged_costmap_topic", "/nav2_merged_costmap")
        self.declare_parameter("map_timeout_s", 2.0)
        self.declare_parameter("status_rate_hz", 5.0)

        self._map_timeout_s = float(
            self.get_parameter("map_timeout_s").value
        )
        status_rate_hz = float(self.get_parameter("status_rate_hz").value)
        if (
            not math.isfinite(self._map_timeout_s)
            or not math.isfinite(status_rate_hz)
            or self._map_timeout_s <= 0.0
            or status_rate_hz <= 0.0
        ):
            raise ValueError(
                "monitor timeout and status rate must be finite and positive"
            )

        self._last_intent_signature = None
        self._last_envelope_signature = None
        self._last_map_signature = None
        self._merged_received_s = None
        self.create_subscription(
            OperatorIntent,
            str(self.get_parameter("intent_topic").value),
            self._on_intent,
            1,
        )
        self.create_subscription(
            SafetyEnvelope,
            str(self.get_parameter("envelope_topic").value),
            self._on_envelope,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("merged_costmap_topic").value),
            self._on_merged_map,
            1,
        )
        self.create_timer(1.0 / status_rate_hz, self._publish_map_state)
        self.get_logger().info("[INTENT] WAITING")

    def _on_intent(self, msg: OperatorIntent) -> None:
        signature = intent_signature(msg)
        if signature == self._last_intent_signature:
            return
        self.get_logger().info(format_intent(msg))
        self._last_intent_signature = signature

    def _on_envelope(self, msg: SafetyEnvelope) -> None:
        signature = envelope_signature(msg)
        if signature == self._last_envelope_signature:
            return
        self.get_logger().info(format_envelope(msg))
        self._last_envelope_signature = signature

    def _on_merged_map(self, _msg: OccupancyGrid) -> None:
        self._merged_received_s = time.monotonic()
        self._publish_map_state()

    def _publish_map_state(self) -> None:
        state = map_pipeline_state(
            merged_received_s=self._merged_received_s,
            now_s=time.monotonic(),
            timeout_s=self._map_timeout_s,
        )
        if state.signature == self._last_map_signature:
            return
        self.get_logger().info(format_map_state(state))
        self._last_map_signature = state.signature


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = SafetyEnvelopeMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
