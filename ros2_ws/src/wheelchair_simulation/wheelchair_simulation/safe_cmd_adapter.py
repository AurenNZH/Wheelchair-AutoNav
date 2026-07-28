"""Simulation-only adapter from safety envelopes to Gazebo velocity."""

from __future__ import annotations

from dataclasses import dataclass
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope


@dataclass(frozen=True)
class SimIntent:
    session_id: str
    sequence: int
    steering: float
    forward: float
    deadman: bool


@dataclass(frozen=True)
class SimEnvelope:
    session_id: str
    intent_sequence: int
    decision: int
    permitted_forward: float
    permitted_steering: float


def safe_sim_velocity(
    intent: SimIntent | None,
    envelope: SimEnvelope | None,
    *,
    intent_age_s: float,
    envelope_age_s: float,
    max_age_s: float,
    max_linear_mps: float,
    max_angular_radps: float,
) -> tuple[float, float, str]:
    """Convert only a fresh, matching, non-stop envelope into simulated motion."""

    if intent is None or envelope is None:
        return 0.0, 0.0, "missing_intent_or_envelope"
    if intent_age_s < 0.0 or envelope_age_s < 0.0:
        return 0.0, 0.0, "invalid_age"
    if intent_age_s > max_age_s or envelope_age_s > max_age_s:
        return 0.0, 0.0, "stale_intent_or_envelope"
    if not intent.deadman or intent.forward <= 0.0:
        return 0.0, 0.0, "operator_released"
    if (
        envelope.session_id != intent.session_id
        or envelope.intent_sequence != intent.sequence
    ):
        return 0.0, 0.0, "sequence_mismatch"
    if envelope.decision == SafetyEnvelope.STOP:
        return 0.0, 0.0, "safety_stop"
    return (
        max(0.0, min(1.0, envelope.permitted_forward)) * max_linear_mps,
        max(-1.0, min(1.0, envelope.permitted_steering))
        * max_angular_radps,
        "permitted",
    )


class SafeCmdAdapterNode(Node):
    """Publish `/sim/safe_cmd_vel`; this node has no real actuator interface."""

    def __init__(self) -> None:
        super().__init__("sim_safe_cmd_adapter")
        self.declare_parameter("enable_sim_motion", False)
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter("safety_envelope_topic", "/safety_envelope")
        self.declare_parameter("cmd_vel_topic", "/sim/safe_cmd_vel")
        self.declare_parameter("max_message_age_s", 0.20)
        self.declare_parameter("max_linear_mps", 0.15)
        self.declare_parameter("max_angular_radps", 0.40)
        self.declare_parameter("publish_rate_hz", 20.0)

        self._enabled = bool(self.get_parameter("enable_sim_motion").value)
        self._intent = None
        self._intent_received = 0.0
        self._envelope = None
        self._envelope_received = 0.0
        self._last_reason = None
        self._publisher = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.create_subscription(
            OperatorIntent,
            self.get_parameter("operator_intent_topic").value,
            self._on_intent,
            10,
        )
        self.create_subscription(
            SafetyEnvelope,
            self.get_parameter("safety_envelope_topic").value,
            self._on_envelope,
            10,
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            "Simulation motion adapter enabled=%s; output is isolated on /sim."
            % self._enabled
        )

    def _on_intent(self, msg: OperatorIntent) -> None:
        self._intent = SimIntent(
            msg.session_id,
            int(msg.sequence),
            float(msg.steering),
            float(msg.forward),
            bool(msg.deadman),
        )
        self._intent_received = time.monotonic()

    def _on_envelope(self, msg: SafetyEnvelope) -> None:
        self._envelope = SimEnvelope(
            msg.session_id,
            int(msg.intent_sequence),
            int(msg.decision),
            float(msg.permitted_forward),
            float(msg.permitted_steering),
        )
        self._envelope_received = time.monotonic()

    def _publish(self) -> None:
        now = time.monotonic()
        if self._enabled:
            linear, angular, reason = safe_sim_velocity(
                self._intent,
                self._envelope,
                intent_age_s=now - self._intent_received,
                envelope_age_s=now - self._envelope_received,
                max_age_s=float(self.get_parameter("max_message_age_s").value),
                max_linear_mps=float(
                    self.get_parameter("max_linear_mps").value
                ),
                max_angular_radps=float(
                    self.get_parameter("max_angular_radps").value
                ),
            )
        else:
            linear, angular, reason = 0.0, 0.0, "simulation_motion_disabled"
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._publisher.publish(msg)
        if reason != self._last_reason:
            self.get_logger().info("Simulation command state: %s" % reason)
            self._last_reason = reason


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = SafeCmdAdapterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
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
