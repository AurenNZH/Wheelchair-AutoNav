"""ROS node that turns operator intent and obstacle grids into safe limits."""

from __future__ import annotations

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope

from wheelchair_shared_control.safety import (
    SafetyConfig,
    OperatorIntentData,
    evaluate_safety,
    occupied_points_from_grid,
)


class SafetySupervisorNode(Node):
    """Publish limits only; never publish Twist or access CAN hardware."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter("safety_envelope_topic", "/safety_envelope")
        self.declare_parameter("front_costmap_topic", "/front_costmap")
        self.declare_parameter("diagnostics_topic", "/shared_control/diagnostics")
        self.declare_parameter("decision_rate_hz", 20.0)
        self.declare_parameter("max_intent_age_s", 0.20)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("geometry_calibrated", False)
        self.declare_parameter("chair_width_m", 0.70)
        self.declare_parameter("front_extent_m", 0.80)
        self.declare_parameter("rear_extent_m", 0.40)
        self.declare_parameter("lateral_margin_m", 0.15)
        self.declare_parameter("stop_distance_m", 0.70)
        self.declare_parameter("slow_distance_m", 1.20)
        self.declare_parameter("min_turn_radius_m", 1.20)
        self.declare_parameter("min_steering", -0.35)
        self.declare_parameter("max_steering", 0.0)
        self.declare_parameter("slow_forward_limit", 0.35)
        self.declare_parameter("path_sample_step_m", 0.05)
        self.declare_parameter("max_map_age_s", 0.30)

        self._config = self._load_config()
        self._intent = None
        self._front_points = None
        self._front_stamp_ns = 0
        self._last_reason = None

        self._envelope_pub = self.create_publisher(
            SafetyEnvelope, self.get_parameter("safety_envelope_topic").value, 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
        )
        self.create_subscription(
            OperatorIntent,
            self.get_parameter("operator_intent_topic").value,
            self._on_intent,
            1,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("front_costmap_topic").value,
            self._on_front_map,
            1,
        )
        rate_hz = float(self.get_parameter("decision_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("decision_rate_hz must be positive")
        self.create_timer(1.0 / rate_hz, self._publish_decision)
        self.get_logger().info(
            "Shared-control supervisor started fail-closed: "
            "enable_motion=%s geometry_calibrated=%s"
            % (self._config.enable_motion, self._config.geometry_calibrated)
        )

    def _load_config(self) -> SafetyConfig:
        return SafetyConfig(
            enable_motion=bool(self.get_parameter("enable_motion").value),
            geometry_calibrated=bool(
                self.get_parameter("geometry_calibrated").value
            ),
            chair_width_m=float(self.get_parameter("chair_width_m").value),
            front_extent_m=float(self.get_parameter("front_extent_m").value),
            rear_extent_m=float(self.get_parameter("rear_extent_m").value),
            lateral_margin_m=float(
                self.get_parameter("lateral_margin_m").value
            ),
            stop_distance_m=float(self.get_parameter("stop_distance_m").value),
            slow_distance_m=float(self.get_parameter("slow_distance_m").value),
            min_turn_radius_m=float(
                self.get_parameter("min_turn_radius_m").value
            ),
            min_steering=float(self.get_parameter("min_steering").value),
            max_steering=float(self.get_parameter("max_steering").value),
            slow_forward_limit=float(
                self.get_parameter("slow_forward_limit").value
            ),
            path_sample_step_m=float(
                self.get_parameter("path_sample_step_m").value
            ),
            max_map_age_s=float(self.get_parameter("max_map_age_s").value),
        )

    def _on_intent(self, msg: OperatorIntent) -> None:
        self._intent = msg

    def _on_front_map(self, msg: OccupancyGrid) -> None:
        try:
            self._front_points = self._grid_points(msg)
            self._front_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        except ValueError as exc:
            self.get_logger().error(
                "Rejected invalid front costmap: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._front_points = None
            self._front_stamp_ns = 0

    @staticmethod
    def _grid_points(msg: OccupancyGrid):
        return occupied_points_from_grid(
            msg.data,
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution_m=float(msg.info.resolution),
            origin_x_m=float(msg.info.origin.position.x),
            origin_y_m=float(msg.info.origin.position.y),
        )

    def _publish_decision(self) -> None:
        started = time.perf_counter()
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        session_id = "no-session"
        intent_sequence = 0
        map_age_s = 0.0

        if self._intent is None:
            decision = self._stop("missing_intent")
        else:
            session_id = self._intent.session_id or "invalid-session"
            intent_sequence = int(self._intent.sequence)
            intent_stamp_ns = Time.from_msg(self._intent.header.stamp).nanoseconds
            intent_age_s = (now_ns - intent_stamp_ns) / 1e9
            if intent_stamp_ns <= 0 or intent_age_s < 0.0:
                decision = self._stop("invalid_intent_timestamp")
            elif intent_age_s > float(
                self.get_parameter("max_intent_age_s").value
            ):
                decision = self._stop("stale_intent")
            elif self._front_points is None:
                decision = self._stop("missing_map")
            elif self._front_stamp_ns <= 0:
                decision = self._stop("invalid_map_timestamp")
            else:
                map_age_s = (now_ns - self._front_stamp_ns) / 1e9
                intent = OperatorIntentData(
                    session_id=session_id,
                    sequence=intent_sequence,
                    steering=float(self._intent.steering),
                    forward=float(self._intent.forward),
                    deadman=bool(self._intent.deadman),
                )
                decision = evaluate_safety(
                    intent,
                    self._front_points,
                    map_age_s,
                    self._config,
                )

        envelope = SafetyEnvelope()
        envelope.header.stamp = now.to_msg()
        envelope.header.frame_id = "base_link"
        envelope.session_id = session_id
        envelope.intent_sequence = intent_sequence
        envelope.decision = decision.decision
        envelope.permitted_forward = decision.permitted_forward
        envelope.permitted_steering = decision.permitted_steering
        envelope.reason = decision.reason
        envelope.map_age_ms = max(0.0, map_age_s * 1000.0)
        self._envelope_pub.publish(envelope)
        self._publish_diagnostics(
            decision.reason,
            decision.decision,
            envelope.map_age_ms,
            (time.perf_counter() - started) * 1000.0,
            decision.nearest_path_distance_m,
        )

    @staticmethod
    def _stop(reason: str):
        from wheelchair_shared_control.safety import SafetyDecision, STOP

        return SafetyDecision(STOP, 0.0, 0.0, reason)

    def _publish_diagnostics(
        self,
        reason: str,
        decision: int,
        map_age_ms: float,
        processing_ms: float,
        nearest_path_distance_m: float | None,
    ) -> None:
        status = DiagnosticStatus()
        status.name = "wheelchair_shared_control/safety_supervisor"
        status.hardware_id = "jetson"
        status.level = (
            DiagnosticStatus.OK
            if decision == SafetyEnvelope.CLEAR
            else DiagnosticStatus.WARN
        )
        status.message = reason
        status.values = [
            KeyValue(key="map_age_ms", value="%.3f" % map_age_ms),
            KeyValue(key="processing_ms", value="%.3f" % processing_ms),
            KeyValue(key="enable_motion", value=str(self._config.enable_motion)),
            KeyValue(
                key="geometry_calibrated",
                value=str(self._config.geometry_calibrated),
            ),
            KeyValue(
                key="min_steering",
                value="%.3f" % self._config.min_steering,
            ),
            KeyValue(
                key="max_steering",
                value="%.3f" % self._config.max_steering,
            ),
            KeyValue(
                key="nearest_path_distance_m",
                value=(
                    "none"
                    if nearest_path_distance_m is None
                    else "%.3f" % nearest_path_distance_m
                ),
            ),
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)
        if reason != self._last_reason:
            self.get_logger().info(
                "Safety decision changed: %s (nearest=%s)"
                % (
                    reason,
                    "none"
                    if nearest_path_distance_m is None
                    else "%.3f m" % nearest_path_distance_m,
                )
            )
            self._last_reason = reason


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = SafetySupervisorNode()
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
