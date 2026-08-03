"""Human-readable transition monitor for recorded-map safety decisions."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import SafetyEnvelope


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
    return (
        "%s reason=%s permitted_forward=%.3f permitted_steering=%.3f "
        "map_age_ms=%.1f session=%s sequence=%d"
        % (
            decision_name(msg.decision),
            msg.reason,
            msg.permitted_forward,
            msg.permitted_steering,
            msg.map_age_ms,
            msg.session_id,
            msg.intent_sequence,
        )
    )


class SafetyEnvelopeMonitorNode(Node):
    """Log safety-envelope state changes without producing control output."""

    def __init__(self) -> None:
        super().__init__("safety_envelope_monitor")
        self.declare_parameter("envelope_topic", "/safety_envelope")
        self._last_signature = None
        self.create_subscription(
            SafetyEnvelope,
            str(self.get_parameter("envelope_topic").value),
            self._on_envelope,
            10,
        )
        self.get_logger().info(
            "Safety-envelope transition monitor started; no commands are published."
        )

    def _on_envelope(self, msg: SafetyEnvelope) -> None:
        signature = envelope_signature(msg)
        if signature == self._last_signature:
            return
        self.get_logger().info(format_envelope(msg))
        self._last_signature = signature


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
