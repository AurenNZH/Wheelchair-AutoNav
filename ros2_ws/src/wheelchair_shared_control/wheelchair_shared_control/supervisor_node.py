"""ROS node that turns operator intent and obstacle grids into safe limits."""

from __future__ import annotations

from dataclasses import dataclass
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope

from wheelchair_shared_control.safety import (
    SafetyConfig,
    SafetyDecision,
    OperatorIntentData,
    evaluate_safety,
    validate_cost_policy,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.operator_intent import classify_normalized_axes


NAV2_LIVE = "nav2_live"
LEGACY_MAP_STAMP = "legacy_map_stamp"
FRESHNESS_MODES = (NAV2_LIVE, LEGACY_MAP_STAMP)


@dataclass(frozen=True)
class FreshnessStatus:
    """Ages and any fail-closed reason for one supervisor cycle."""

    map_age_s: float = 0.0
    source_age_s: float | None = None
    failure_reason: str | None = None
    map_age_basis: str = "receipt_time"


def evaluate_freshness(
    *,
    mode: str,
    now_ros_ns: int,
    now_monotonic_ns: int,
    map_stamp_ns: int,
    map_received_monotonic_ns: int,
    source_stamp_ns: int | None,
    max_source_age_s: float,
    max_future_source_offset_s: float,
) -> FreshnessStatus:
    """Evaluate either live Nav2 continuity or legacy map provenance."""

    if mode not in FRESHNESS_MODES:
        raise ValueError("unsupported freshness_mode: %s" % mode)
    if mode == LEGACY_MAP_STAMP:
        if map_stamp_ns <= 0:
            return FreshnessStatus(
                failure_reason="invalid_map_timestamp",
                map_age_basis="map_header_stamp",
            )
        return FreshnessStatus(
            map_age_s=(now_ros_ns - map_stamp_ns) / 1e9,
            map_age_basis="map_header_stamp",
        )

    if map_received_monotonic_ns <= 0:
        return FreshnessStatus(failure_reason="missing_map_receipt")
    map_age_s = (
        now_monotonic_ns - map_received_monotonic_ns
    ) / 1e9
    if map_age_s < 0.0:
        return FreshnessStatus(
            failure_reason="invalid_map_receipt_time"
        )
    if source_stamp_ns is None:
        return FreshnessStatus(
            map_age_s=map_age_s,
            failure_reason="missing_source_heartbeat",
        )
    if source_stamp_ns <= 0:
        return FreshnessStatus(
            map_age_s=map_age_s,
            failure_reason="invalid_source_timestamp",
        )
    source_age_s = (now_ros_ns - source_stamp_ns) / 1e9
    if source_age_s < -max_future_source_offset_s:
        return FreshnessStatus(
            map_age_s=map_age_s,
            source_age_s=source_age_s,
            failure_reason="future_source_timestamp",
        )
    source_age_s = max(0.0, source_age_s)
    if source_age_s > max_source_age_s:
        return FreshnessStatus(
            map_age_s=map_age_s,
            source_age_s=source_age_s,
            failure_reason="stale_source",
        )
    return FreshnessStatus(
        map_age_s=map_age_s,
        source_age_s=source_age_s,
    )


def safety_diagnostic_values(
    decision: SafetyDecision,
    config: SafetyConfig,
    map_age_ms: float,
    processing_ms: float,
    *,
    freshness_mode: str = NAV2_LIVE,
    map_age_basis: str = "receipt_time",
    source_age_ms: float | None = None,
) -> list[KeyValue]:
    """Build stable, machine-readable evidence for one decision."""

    return [
        KeyValue(key="map_age_ms", value="%.3f" % map_age_ms),
        KeyValue(key="freshness_mode", value=freshness_mode),
        KeyValue(key="map_age_basis", value=map_age_basis),
        KeyValue(
            key="source_age_ms",
            value=(
                "none" if source_age_ms is None else "%.3f" % source_age_ms
            ),
        ),
        KeyValue(key="processing_ms", value="%.3f" % processing_ms),
        KeyValue(key="enable_motion", value=str(config.enable_motion)),
        KeyValue(
            key="geometry_calibrated",
            value=str(config.geometry_calibrated),
        ),
        KeyValue(key="min_steering", value="%.3f" % config.min_steering),
        KeyValue(key="max_steering", value="%.3f" % config.max_steering),
        KeyValue(
            key="nearest_path_distance_m",
            value=(
                "none"
                if decision.nearest_path_distance_m is None
                else "%.3f" % decision.nearest_path_distance_m
            ),
        ),
        KeyValue(
            key="maximum_path_cost",
            value=(
                "none"
                if decision.maximum_path_cost is None
                else str(decision.maximum_path_cost)
            ),
        ),
        KeyValue(
            key="nearest_slow_cost_distance_m",
            value=(
                "none"
                if decision.nearest_slow_cost_distance_m is None
                else "%.3f" % decision.nearest_slow_cost_distance_m
            ),
        ),
        KeyValue(
            key="nearest_stop_cost_distance_m",
            value=(
                "none"
                if decision.nearest_stop_cost_distance_m is None
                else "%.3f" % decision.nearest_stop_cost_distance_m
            ),
        ),
        KeyValue(
            key="slow_cost_threshold",
            value=str(config.slow_cost_threshold),
        ),
        KeyValue(
            key="stop_cost_threshold",
            value=str(config.stop_cost_threshold),
        ),
        KeyValue(
            key="path_cost_valid",
            value=str(decision.path_cost_valid),
        ),
    ]


class SafetySupervisorNode(Node):
    """Publish limits only; never publish Twist or access CAN hardware."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter("safety_envelope_topic", "/safety_envelope")
        self.declare_parameter(
            "front_costmap_topic", "/nav2_front_costmap"
        )
        self.declare_parameter(
            "source_header_topic", "/artifact_filter/source_header"
        )
        self.declare_parameter("freshness_mode", NAV2_LIVE)
        self.declare_parameter("diagnostics_topic", "/shared_control/diagnostics")
        self.declare_parameter("decision_rate_hz", 20.0)
        self.declare_parameter("max_intent_age_s", 0.20)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("geometry_calibrated", False)
        self.declare_parameter("stop_distance_m", 0.70)
        self.declare_parameter("slow_distance_m", 1.20)
        self.declare_parameter("min_turn_radius_m", 1.20)
        self.declare_parameter("min_steering", -0.466307658)
        self.declare_parameter("max_steering", 0.466307658)
        self.declare_parameter("slow_forward_limit", 0.65)
        self.declare_parameter("path_sample_step_m", 0.05)
        self.declare_parameter("steering_sample_step", 0.05)
        self.declare_parameter("neutral_deadzone", 0.05)
        self.declare_parameter("forward_cone_half_angle_deg", 25.0)
        self.declare_parameter("max_map_age_s", 0.50)
        self.declare_parameter("max_source_age_s", 0.50)
        self.declare_parameter("max_future_source_offset_s", 0.10)
        self.declare_parameter("slow_cost_threshold", 1)
        self.declare_parameter("stop_cost_threshold", 99)

        self._config = self._load_config()
        self._intent = None
        self._front_costmap = None
        self._front_stamp_ns = 0
        self._front_received_monotonic_ns = 0
        self._source_stamp_ns = None
        self._freshness_mode = str(
            self.get_parameter("freshness_mode").value
        )
        self._validate_freshness_parameters()
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
        self.create_subscription(
            Header,
            self.get_parameter("source_header_topic").value,
            self._on_source_header,
            10,
        )
        rate_hz = float(self.get_parameter("decision_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("decision_rate_hz must be positive")
        self.create_timer(1.0 / rate_hz, self._publish_decision)
        self.get_logger().info(
            "Nav2-cost shared-control supervisor started fail-closed: "
            "map=%s freshness=%s slow_cost=%d stop_cost=%d enable_motion=%s "
            "geometry_calibrated=%s"
            % (
                self.get_parameter("front_costmap_topic").value,
                self._freshness_mode,
                self._config.slow_cost_threshold,
                self._config.stop_cost_threshold,
                self._config.enable_motion,
                self._config.geometry_calibrated,
            )
        )

    def _load_config(self) -> SafetyConfig:
        config = SafetyConfig(
            enable_motion=bool(self.get_parameter("enable_motion").value),
            geometry_calibrated=bool(
                self.get_parameter("geometry_calibrated").value
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
            steering_sample_step=float(
                self.get_parameter("steering_sample_step").value
            ),
            neutral_deadzone=float(
                self.get_parameter("neutral_deadzone").value
            ),
            forward_cone_half_angle_deg=float(
                self.get_parameter("forward_cone_half_angle_deg").value
            ),
            max_map_age_s=float(self.get_parameter("max_map_age_s").value),
            slow_cost_threshold=int(
                self.get_parameter("slow_cost_threshold").value
            ),
            stop_cost_threshold=int(
                self.get_parameter("stop_cost_threshold").value
            ),
        )
        validate_cost_policy(config)
        return config

    def _validate_freshness_parameters(self) -> None:
        if self._freshness_mode not in FRESHNESS_MODES:
            raise ValueError(
                "freshness_mode must be one of %s"
                % ", ".join(FRESHNESS_MODES)
            )
        if float(self.get_parameter("max_source_age_s").value) <= 0.0:
            raise ValueError("max_source_age_s must be positive")
        if float(
            self.get_parameter("max_future_source_offset_s").value
        ) < 0.0:
            raise ValueError(
                "max_future_source_offset_s must be non-negative"
            )

    def _on_intent(self, msg: OperatorIntent) -> None:
        self._intent = msg

    def _on_source_header(self, msg: Header) -> None:
        self._source_stamp_ns = Time.from_msg(msg.stamp).nanoseconds

    def _on_front_map(self, msg: OccupancyGrid) -> None:
        try:
            self._front_costmap = self._grid_costmap(msg)
            self._front_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
            self._front_received_monotonic_ns = time.monotonic_ns()
        except ValueError as exc:
            self.get_logger().error(
                "Rejected invalid front costmap: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._front_costmap = None
            self._front_stamp_ns = 0
            self._front_received_monotonic_ns = 0

    @staticmethod
    def _grid_costmap(msg: OccupancyGrid):
        orientation = msg.info.origin.orientation
        return weighted_costmap_from_grid(
            msg.data,
            frame_id=str(msg.header.frame_id),
            width=int(msg.info.width),
            height=int(msg.info.height),
            resolution_m=float(msg.info.resolution),
            origin_x_m=float(msg.info.origin.position.x),
            origin_y_m=float(msg.info.origin.position.y),
            origin_orientation_xyzw=(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )

    def _publish_decision(self) -> None:
        started = time.perf_counter()
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        session_id = "no-session"
        intent_sequence = 0
        map_age_s = 0.0
        source_age_s = None
        map_age_basis = (
            "receipt_time"
            if self._freshness_mode == NAV2_LIVE
            else "map_header_stamp"
        )

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
            elif self._front_costmap is None:
                decision = self._stop("missing_map")
            else:
                freshness = evaluate_freshness(
                    mode=self._freshness_mode,
                    now_ros_ns=now_ns,
                    now_monotonic_ns=time.monotonic_ns(),
                    map_stamp_ns=self._front_stamp_ns,
                    map_received_monotonic_ns=(
                        self._front_received_monotonic_ns
                    ),
                    source_stamp_ns=self._source_stamp_ns,
                    max_source_age_s=float(
                        self.get_parameter("max_source_age_s").value
                    ),
                    max_future_source_offset_s=float(
                        self.get_parameter(
                            "max_future_source_offset_s"
                        ).value
                    ),
                )
                map_age_s = freshness.map_age_s
                source_age_s = freshness.source_age_s
                map_age_basis = freshness.map_age_basis
                if freshness.failure_reason is not None:
                    decision = self._stop(freshness.failure_reason)
                else:
                    decision = self._evaluate_intent(map_age_s)

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
            decision,
            envelope.map_age_ms,
            (time.perf_counter() - started) * 1000.0,
            map_age_basis,
            (
                None
                if source_age_s is None
                else max(0.0, source_age_s * 1000.0)
            ),
        )

    def _evaluate_intent(self, map_age_s: float) -> SafetyDecision:
        lateral = float(self._intent.lateral)
        longitudinal = float(self._intent.longitudinal)
        intent_class = int(self._intent.intent_class)
        deadman = bool(self._intent.deadman)
        if (
            lateral == 0.0
            and longitudinal == 0.0
            and intent_class == int(OperatorIntent.RELEASED)
            and deadman
        ):
            # Compatibility for ROS-only simulation publishers that still
            # fill the original steering/forward projection.
            longitudinal = float(self._intent.forward)
            lateral = float(self._intent.steering) * longitudinal
            try:
                intent_class = classify_normalized_axes(
                    lateral,
                    longitudinal,
                    neutral_deadzone=self._config.neutral_deadzone,
                    forward_cone_half_angle_deg=(
                        self._config.forward_cone_half_angle_deg
                    ),
                ).intent_class
            except ValueError:
                intent_class = -1
        intent = OperatorIntentData(
            session_id=self._intent.session_id or "invalid-session",
            sequence=int(self._intent.sequence),
            lateral=lateral,
            longitudinal=longitudinal,
            intent_class=intent_class,
            deadman=deadman,
        )
        return evaluate_safety(
            intent,
            self._front_costmap,
            map_age_s,
            self._config,
        )

    @staticmethod
    def _stop(reason: str):
        from wheelchair_shared_control.safety import SafetyDecision, STOP

        return SafetyDecision(STOP, 0.0, 0.0, reason)

    def _publish_diagnostics(
        self,
        decision,
        map_age_ms: float,
        processing_ms: float,
        map_age_basis: str,
        source_age_ms: float | None,
    ) -> None:
        status = DiagnosticStatus()
        status.name = "wheelchair_shared_control/safety_supervisor"
        status.hardware_id = "jetson"
        status.level = (
            DiagnosticStatus.OK
            if decision.decision == SafetyEnvelope.CLEAR
            else DiagnosticStatus.WARN
        )
        status.message = decision.reason
        status.values = safety_diagnostic_values(
            decision,
            self._config,
            map_age_ms,
            processing_ms,
            freshness_mode=self._freshness_mode,
            map_age_basis=map_age_basis,
            source_age_ms=source_age_ms,
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)
        if decision.reason != self._last_reason:
            self.get_logger().info(
                "Safety decision changed: %s (nearest=%s max_cost=%s)"
                % (
                    decision.reason,
                    "none"
                    if decision.nearest_path_distance_m is None
                    else "%.3f m" % decision.nearest_path_distance_m,
                    "none"
                    if decision.maximum_path_cost is None
                    else str(decision.maximum_path_cost),
                )
            )
            self._last_reason = decision.reason


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
