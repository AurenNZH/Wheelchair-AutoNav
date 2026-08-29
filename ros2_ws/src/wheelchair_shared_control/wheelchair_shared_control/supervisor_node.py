"""ROS node that turns operator intent and obstacle grids into safe limits."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Header
from wheelchair_msgs.msg import (
    AvoidanceSuggestion,
    OperatorIntent,
    SafetyEnvelope,
)
from visualization_msgs.msg import MarkerArray

from wheelchair_shared_control.corridor_visualization import (
    build_checked_corridor_markers,
    corridor_intent_view,
)
from wheelchair_shared_control.avoidance_arbitration import (
    bound_current_assist,
    candidate_is_no_worse,
    suggestion_age_is_fresh,
    suggestion_compatibility_reason,
)
from wheelchair_shared_control.diagnostics import (
    SafetyDiagnosticSnapshot,
    build_safety_diagnostic_status,
    format_decision_transition,
)
from wheelchair_shared_control.freshness import (
    FreshnessInputs,
    FreshnessPolicy,
    NAV2_LIVE,
    evaluate_freshness,
    validate_freshness_policy,
)
from wheelchair_shared_control.models import (
    SafetyConfig,
    SafetyDecision,
    OperatorIntentData,
    validate_safety_config,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.safety_policy import (
    evaluate_assisted_forward_safety,
    evaluate_safety,
    motion_configuration_decision,
    stop_decision,
)
from wheelchair_shared_control.operator_intent import (
    classify_normalized_axes,
    is_valid_hard_turn,
)


class SafetySupervisorNode(Node):
    """Publish limits only; never publish Twist or access CAN hardware."""

    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter("safety_envelope_topic", "/safety_envelope")
        self.declare_parameter(
            "merged_costmap_topic", "/nav2_merged_costmap"
        )
        self.declare_parameter(
            "source_header_topic", "/lidar_right/filter/source_header"
        )
        self.declare_parameter(
            "left_source_header_topic", "/lidar_left/filter/source_header"
        )
        self.declare_parameter("freshness_mode", NAV2_LIVE)
        self.declare_parameter(
            "diagnostics_topic", "/shared_control/diagnostics"
        )
        self.declare_parameter(
            "checked_corridor_topic", "/shared_control/checked_corridor"
        )
        self.declare_parameter("decision_rate_hz", 20.0)
        self.declare_parameter("max_intent_age_s", 0.20)
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("geometry_calibrated", False)
        self.declare_parameter("stop_distance_m", 0.70)
        self.declare_parameter("slow_distance_m", 1.20)
        self.declare_parameter("min_turn_radius_m", 1.20)
        self.declare_parameter("min_steering", -0.577350269)
        self.declare_parameter("max_steering", 0.577350269)
        self.declare_parameter("slow_forward_limit", 0.60)
        self.declare_parameter("reverse_limit", 0.65)
        self.declare_parameter("turn_clearance_radius_m", 0.55)
        self.declare_parameter("clear_turn_limit", 0.90)
        self.declare_parameter("slow_turn_limit", 0.60)
        self.declare_parameter("turn_longitudinal_limit", 0.15)
        self.declare_parameter("path_sample_step_m", 0.05)
        self.declare_parameter("steering_sample_step", 0.05)
        self.declare_parameter("neutral_deadzone", 0.05)
        self.declare_parameter("forward_cone_half_angle_deg", 30.0)
        self.declare_parameter("max_map_age_s", 0.50)
        self.declare_parameter("max_source_age_s", 0.50)
        self.declare_parameter("max_future_source_offset_s", 0.10)
        self.declare_parameter("slow_cost_threshold", 1)
        self.declare_parameter("stop_cost_threshold", 99)
        self.declare_parameter("avoidance_mode", "disabled")
        self.declare_parameter(
            "avoidance_suggestion_topic",
            "/shared_control/avoidance_suggestion",
        )
        self.declare_parameter("avoidance_suggestion_max_age_s", 0.25)
        self.declare_parameter("avoidance_source_steering_tolerance", 0.05)
        self.declare_parameter("maximum_steering_assist", 0.15)

        self._config = self._load_config()
        self._intent = None
        self._merged_costmap = None
        self._merged_stamp_ns = 0
        self._merged_received_monotonic_ns = 0
        self._source_stamp_ns = None
        self._left_source_stamp_ns = None
        self._freshness_policy = self._load_freshness_policy()
        self._freshness_mode = self._freshness_policy.mode
        self._last_reason = None
        self._avoidance_mode = str(
            self.get_parameter("avoidance_mode").value
        ).lower()
        if self._avoidance_mode not in ("disabled", "shadow", "enforce"):
            raise ValueError("avoidance_mode must be disabled, shadow, or enforce")
        self._avoidance_max_age_s = float(
            self.get_parameter("avoidance_suggestion_max_age_s").value
        )
        self._avoidance_source_tolerance = float(
            self.get_parameter("avoidance_source_steering_tolerance").value
        )
        self._maximum_steering_assist = float(
            self.get_parameter("maximum_steering_assist").value
        )
        if (
            not math.isfinite(self._avoidance_max_age_s)
            or self._avoidance_max_age_s <= 0.0
            or not math.isfinite(self._avoidance_source_tolerance)
            or self._avoidance_source_tolerance < 0.0
            or not math.isfinite(self._maximum_steering_assist)
            or not 0.0 <= self._maximum_steering_assist <= 1.0
        ):
            raise ValueError("invalid avoidance arbitration parameters")
        self._avoidance_suggestion = None
        self._avoidance_received_monotonic = 0.0
        self._avoidance_status = "disabled"

        self._envelope_pub = self.create_publisher(
            SafetyEnvelope,
            self.get_parameter("safety_envelope_topic").value,
            10,
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, self.get_parameter("diagnostics_topic").value, 10
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._corridor_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("checked_corridor_topic").value,
            marker_qos,
        )
        self.create_subscription(
            OperatorIntent,
            self.get_parameter("operator_intent_topic").value,
            self._on_intent,
            1,
        )
        self.create_subscription(
            AvoidanceSuggestion,
            self.get_parameter("avoidance_suggestion_topic").value,
            self._on_avoidance_suggestion,
            1,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("merged_costmap_topic").value,
            self._on_merged_map,
            1,
        )
        self.create_subscription(
            Header,
            self.get_parameter("source_header_topic").value,
            self._on_source_header,
            10,
        )
        self.create_subscription(
            Header,
            self.get_parameter("left_source_header_topic").value,
            self._on_left_source_header,
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
                self.get_parameter("merged_costmap_topic").value,
                self._freshness_mode,
                self._config.slow_cost_threshold,
                self._config.stop_cost_threshold,
                self._config.enable_motion,
                self._config.geometry_calibrated,
            )
        )
        self.get_logger().info(
            "Local avoidance mode=%s suggestion_age<=%.3fs assist<=%.3f"
            % (
                self._avoidance_mode,
                self._avoidance_max_age_s,
                self._maximum_steering_assist,
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
            reverse_limit=float(self.get_parameter("reverse_limit").value),
            turn_clearance_radius_m=float(
                self.get_parameter("turn_clearance_radius_m").value
            ),
            clear_turn_limit=float(
                self.get_parameter("clear_turn_limit").value
            ),
            slow_turn_limit=float(
                self.get_parameter("slow_turn_limit").value
            ),
            turn_longitudinal_limit=float(
                self.get_parameter("turn_longitudinal_limit").value
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
            slow_cost_threshold=int(
                self.get_parameter("slow_cost_threshold").value
            ),
            stop_cost_threshold=int(
                self.get_parameter("stop_cost_threshold").value
            ),
        )
        validate_safety_config(config)
        return config

    def _load_freshness_policy(self) -> FreshnessPolicy:
        policy = FreshnessPolicy(
            mode=str(self.get_parameter("freshness_mode").value),
            max_intent_age_s=float(
                self.get_parameter("max_intent_age_s").value
            ),
            max_map_age_s=float(
                self.get_parameter("max_map_age_s").value
            ),
            max_source_age_s=float(
                self.get_parameter("max_source_age_s").value
            ),
            max_future_source_offset_s=float(
                self.get_parameter("max_future_source_offset_s").value
            ),
        )
        validate_freshness_policy(policy)
        return policy

    def _on_intent(self, msg: OperatorIntent) -> None:
        self._intent = msg

    def _on_avoidance_suggestion(self, msg: AvoidanceSuggestion) -> None:
        self._avoidance_suggestion = msg
        self._avoidance_received_monotonic = time.monotonic()

    def _on_source_header(self, msg: Header) -> None:
        self._source_stamp_ns = Time.from_msg(msg.stamp).nanoseconds

    def _on_left_source_header(self, msg: Header) -> None:
        self._left_source_stamp_ns = Time.from_msg(msg.stamp).nanoseconds

    def _on_merged_map(self, msg: OccupancyGrid) -> None:
        try:
            self._merged_costmap = self._grid_costmap(msg)
            self._merged_stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
            self._merged_received_monotonic_ns = time.monotonic_ns()
        except ValueError as exc:
            self.get_logger().error(
                "Rejected invalid merged costmap: %s" % exc,
                throttle_duration_sec=5.0,
            )
            self._merged_costmap = None
            self._merged_stamp_ns = 0
            self._merged_received_monotonic_ns = 0

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
        intent_stamp_ns = None
        if self._intent is not None:
            session_id = self._intent.session_id or "invalid-session"
            intent_sequence = int(self._intent.sequence)
            intent_stamp_ns = Time.from_msg(
                self._intent.header.stamp
            ).nanoseconds

        freshness = evaluate_freshness(
            FreshnessInputs(
                now_ros_ns=now_ns,
                now_monotonic_ns=time.monotonic_ns(),
                intent_stamp_ns=intent_stamp_ns,
                map_available=self._merged_costmap is not None,
                map_stamp_ns=self._merged_stamp_ns,
                map_received_monotonic_ns=(
                    self._merged_received_monotonic_ns
                ),
                source_stamp_ns=self._source_stamp_ns,
                left_source_stamp_ns=self._left_source_stamp_ns,
                require_left_source=self._turn_intent_requested(),
            ),
            self._freshness_policy,
        )
        if freshness.input_failure_reason is not None:
            decision = stop_decision(freshness.input_failure_reason)
            if self._avoidance_mode != "disabled":
                self._avoidance_status = "blocked_%s" % (
                    freshness.input_failure_reason
                )
        else:
            decision = motion_configuration_decision(self._config)
            if decision is None:
                if freshness.map_age_failure_reason is not None:
                    decision = stop_decision(
                        freshness.map_age_failure_reason
                    )
                    if self._avoidance_mode != "disabled":
                        self._avoidance_status = "blocked_%s" % (
                            freshness.map_age_failure_reason
                        )
                else:
                    decision = self._evaluate_intent()
            elif self._avoidance_mode != "disabled":
                self._avoidance_status = "blocked_%s" % decision.reason

        envelope = SafetyEnvelope()
        envelope.header.stamp = now.to_msg()
        envelope.header.frame_id = "base_link"
        envelope.session_id = session_id
        envelope.intent_sequence = intent_sequence
        envelope.decision = decision.decision
        envelope.permitted_forward = decision.permitted_forward
        envelope.permitted_steering = decision.permitted_steering
        envelope.reason = decision.reason
        envelope.map_age_ms = max(0.0, freshness.map_age_s * 1000.0)
        self._envelope_pub.publish(envelope)
        self._publish_diagnostics(
            decision,
            envelope.map_age_ms,
            (time.perf_counter() - started) * 1000.0,
            freshness.map_age_basis,
            (
                None
                if freshness.source_age_s is None
                else max(0.0, freshness.source_age_s * 1000.0)
            ),
            (
                None
                if freshness.left_source_age_s is None
                else max(0.0, freshness.left_source_age_s * 1000.0)
            ),
        )
        self._publish_checked_corridor(decision, envelope.header)

    def _turn_intent_requested(self) -> bool:
        """Require dual-source freshness only for a valid hard-turn intent."""

        if self._intent is None or not bool(self._intent.deadman):
            return False
        return is_valid_hard_turn(
            float(self._intent.lateral),
            float(self._intent.longitudinal),
            int(self._intent.intent_class),
            bool(self._intent.deadman),
            neutral_deadzone=self._config.neutral_deadzone,
            forward_cone_half_angle_deg=(
                self._config.forward_cone_half_angle_deg
            ),
        )

    def _evaluate_intent(self) -> SafetyDecision:
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
        direct = evaluate_safety(
            intent,
            self._merged_costmap,
            self._config,
        )
        assisted = self._assisted_decision(intent, direct)
        return direct if assisted is None else assisted

    def _assisted_decision(
        self,
        intent: OperatorIntentData,
        direct: SafetyDecision,
    ) -> SafetyDecision | None:
        """Return an enforceable fresh suggestion, otherwise direct fallback."""

        if self._avoidance_mode == "disabled":
            self._avoidance_status = "disabled"
            return None
        suggestion = self._avoidance_suggestion
        if suggestion is None:
            self._avoidance_status = "no_suggestion"
            return None
        age_s = time.monotonic() - self._avoidance_received_monotonic
        if not suggestion_age_is_fresh(age_s, self._avoidance_max_age_s):
            self._avoidance_status = "stale_suggestion"
            return None
        advertised = float(self._intent.max_steering_assist)
        source = float(suggestion.source_steering)
        planned = float(suggestion.suggested_steering)
        current = float(intent.lateral) / float(intent.longitudinal)
        compatibility = suggestion_compatibility_reason(
            current_session=intent.session_id,
            current_sequence=intent.sequence,
            current_intent_class=intent.intent_class,
            current_steering=current,
            suggestion_session=str(suggestion.session_id),
            suggestion_sequence=int(suggestion.intent_sequence),
            suggestion_intent_class=int(suggestion.intent_class),
            source_steering=source,
            suggested_steering=planned,
            valid=bool(suggestion.valid),
            source_tolerance=self._avoidance_source_tolerance,
        )
        if compatibility is not None:
            self._avoidance_status = compatibility
            return None
        if not math.isfinite(advertised):
            self._avoidance_status = "invalid_authority"
            return None
        authority = (
            self._maximum_steering_assist
            if self._avoidance_mode == "shadow"
            else min(max(0.0, advertised), self._maximum_steering_assist)
        )
        bounded = bound_current_assist(
            requested=current,
            intent_class=intent.intent_class,
            planned=planned,
            authority=authority,
            system_maximum=self._maximum_steering_assist,
            minimum_steering=self._config.min_steering,
            maximum_steering=self._config.max_steering,
        )
        if bounded is None:
            self._avoidance_status = "outside_current_authority"
            return None
        if abs(bounded - current) < 0.02:
            self._avoidance_status = "below_minimum_correction"
            return None
        candidate = evaluate_assisted_forward_safety(
            intent,
            bounded,
            self._merged_costmap,
            self._config,
        )
        if not candidate_is_no_worse(direct.decision, candidate.decision):
            self._avoidance_status = "candidate_worse_than_direct"
            return None
        self._avoidance_status = (
            "shadow_accepted"
            if self._avoidance_mode == "shadow"
            else "enforced"
        )
        if self._avoidance_mode != "enforce":
            return None
        return candidate

    def _publish_checked_corridor(
        self,
        decision: SafetyDecision,
        header: Header,
    ) -> None:
        requested_steering = None
        turn_disc_requested = False
        label = "waiting"
        if self._intent is not None:
            view = corridor_intent_view(
                lateral=float(self._intent.lateral),
                longitudinal=float(self._intent.longitudinal),
                legacy_forward=float(self._intent.forward),
                legacy_steering=float(self._intent.steering),
                config=self._config,
            )
            requested_steering = view.requested_steering
            turn_disc_requested = view.turn_disc_requested
            label = view.label
        markers = build_checked_corridor_markers(
            header=header,
            decision=decision,
            requested_steering=requested_steering,
            turn_disc_requested=turn_disc_requested,
            config=self._config,
            label=label,
        )
        self._corridor_pub.publish(markers)

    def _publish_diagnostics(
        self,
        decision: SafetyDecision,
        map_age_ms: float,
        processing_ms: float,
        map_age_basis: str,
        source_age_ms: float | None,
        left_source_age_ms: float | None,
    ) -> None:
        snapshot = SafetyDiagnosticSnapshot(
            decision=decision,
            config=self._config,
            map_age_ms=map_age_ms,
            processing_ms=processing_ms,
            freshness_mode=self._freshness_mode,
            map_age_basis=map_age_basis,
            source_age_ms=source_age_ms,
            left_source_age_ms=left_source_age_ms,
            avoidance_mode=self._avoidance_mode,
            avoidance_status=self._avoidance_status,
        )
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [build_safety_diagnostic_status(snapshot)]
        self._diagnostics_pub.publish(diagnostics)
        if decision.reason != self._last_reason:
            self.get_logger().info(format_decision_transition(decision))
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
