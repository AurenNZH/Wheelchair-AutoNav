"""ROS client that turns fresh forward intent into bounded Nav2 suggestions."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from wheelchair_msgs.msg import AvoidanceSuggestion, OperatorIntent

from wheelchair_obstacle_avoidance.planning import (
    FORWARD,
    FORWARD_CLASSES,
    PlanningConfig,
    StraightSideHysteresis,
    bound_assisted_steering,
    path_steering,
    temporary_goal,
    validate_config,
    validate_path,
)


@dataclass(frozen=True)
class _IntentSnapshot:
    session_id: str
    sequence: int
    intent_class: int
    lateral: float
    longitudinal: float
    source_steering: float


class LocalAvoidancePlannerNode(Node):
    """Keep at most one planning request in flight and coalesce newer intent."""

    def __init__(self) -> None:
        super().__init__("local_avoidance_planner")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter(
            "suggestion_topic", "/shared_control/avoidance_suggestion"
        )
        self.declare_parameter("accepted_path_topic", "/local_avoidance/path")
        self.declare_parameter("goal_topic", "/local_avoidance/goal")
        self.declare_parameter("diagnostics_topic", "/local_avoidance/diagnostics")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("planner_id", "LocalAvoidance")
        self.declare_parameter("planning_rate_hz", 10.0)
        self.declare_parameter("goal_distance_m", 3.8)
        self.declare_parameter("maximum_cross_track_m", 0.8)
        self.declare_parameter("maximum_path_ratio", 1.25)
        self.declare_parameter("endpoint_tolerance_m", 0.2)
        self.declare_parameter("reverse_progress_tolerance_m", 0.05)
        self.declare_parameter("lookahead_distance_m", 0.8)
        self.declare_parameter("steering_gain", 1.2)
        self.declare_parameter("minimum_correction", 0.02)
        self.declare_parameter("maximum_steering", 0.577350269)
        self.declare_parameter("maximum_assist", 0.15)
        self.declare_parameter("discard_after_ms", 300.0)

        self._config = PlanningConfig(
            goal_distance_m=float(self.get_parameter("goal_distance_m").value),
            maximum_cross_track_m=float(
                self.get_parameter("maximum_cross_track_m").value
            ),
            maximum_path_ratio=float(
                self.get_parameter("maximum_path_ratio").value
            ),
            endpoint_tolerance_m=float(
                self.get_parameter("endpoint_tolerance_m").value
            ),
            reverse_progress_tolerance_m=float(
                self.get_parameter("reverse_progress_tolerance_m").value
            ),
            lookahead_distance_m=float(
                self.get_parameter("lookahead_distance_m").value
            ),
            steering_gain=float(self.get_parameter("steering_gain").value),
            minimum_correction=float(
                self.get_parameter("minimum_correction").value
            ),
            maximum_steering=float(
                self.get_parameter("maximum_steering").value
            ),
            maximum_assist=float(self.get_parameter("maximum_assist").value),
        )
        validate_config(self._config)
        self._discard_after_ms = float(
            self.get_parameter("discard_after_ms").value
        )
        rate_hz = float(self.get_parameter("planning_rate_hz").value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("planning_rate_hz must be positive")
        if not math.isfinite(self._discard_after_ms) or self._discard_after_ms <= 0.0:
            raise ValueError("discard_after_ms must be positive")

        self._latest_intent = None
        self._in_flight = False
        self._last_submitted_key = None
        self._side_hysteresis = StraightSideHysteresis()
        self._suggestion_pub = self.create_publisher(
            AvoidanceSuggestion,
            self.get_parameter("suggestion_topic").value,
            10,
        )
        self._path_pub = self.create_publisher(
            Path, self.get_parameter("accepted_path_topic").value, 1
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_topic").value, 1
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            self.get_parameter("diagnostics_topic").value,
            10,
        )
        self.create_subscription(
            OperatorIntent,
            self.get_parameter("operator_intent_topic").value,
            self._on_intent,
            1,
        )
        self._planner = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter("planner_action").value,
        )
        self.create_timer(1.0 / rate_hz, self._request_latest)
        self.get_logger().info(
            "Odometry-free local avoidance ready: goal=%.2fm budget=%.0fms "
            "assist<=%.2f (suggestions only)"
            % (
                self._config.goal_distance_m,
                self._discard_after_ms,
                self._config.maximum_assist,
            )
        )

    def _on_intent(self, msg: OperatorIntent) -> None:
        lateral = float(msg.lateral)
        longitudinal = float(msg.longitudinal)
        intent_class = int(msg.intent_class)
        eligible = (
            bool(msg.deadman)
            and intent_class in FORWARD_CLASSES
            and longitudinal > 0.0
            and all(math.isfinite(value) for value in (lateral, longitudinal))
        )
        if not eligible:
            self._latest_intent = None
            self._last_submitted_key = None
            self._side_hysteresis.reset()
            return
        source_steering = lateral / longitudinal
        max_assist = float(msg.max_steering_assist)
        if not math.isfinite(source_steering) or not math.isfinite(max_assist):
            self._latest_intent = None
            self._side_hysteresis.reset()
            return
        self._latest_intent = _IntentSnapshot(
            session_id=str(msg.session_id),
            sequence=int(msg.sequence),
            intent_class=intent_class,
            lateral=lateral,
            longitudinal=longitudinal,
            source_steering=source_steering,
        )

    def _request_latest(self) -> None:
        intent = self._latest_intent
        if intent is None or self._in_flight:
            return
        key = (intent.session_id, intent.sequence)
        if key == self._last_submitted_key:
            return
        if not self._planner.server_is_ready():
            self._publish_invalid(intent, "planner_unavailable", 0.0)
            return
        try:
            goal_x, goal_y, heading = temporary_goal(
                intent.lateral,
                intent.longitudinal,
                self._config.goal_distance_m,
            )
        except ValueError:
            self._publish_invalid(intent, "invalid_intent", 0.0)
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        self._goal_pub.publish(pose)
        request = ComputePathToPose.Goal()
        request.pose = pose
        request.planner_id = str(self.get_parameter("planner_id").value)
        self._in_flight = True
        self._last_submitted_key = key
        started = time.monotonic()
        future = self._planner.send_goal_async(request)
        future.add_done_callback(
            lambda completed: self._on_goal_response(completed, intent, started)
        )

    def _on_goal_response(self, future, intent: _IntentSnapshot, started: float) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # rclpy futures propagate action transport errors
            self._in_flight = False
            self._publish_invalid(intent, "planner_request_error", self._elapsed_ms(started))
            self.get_logger().warning("Planner request failed: %s" % exc)
            return
        if goal_handle is None or not goal_handle.accepted:
            self._in_flight = False
            self._publish_invalid(intent, "planner_rejected", self._elapsed_ms(started))
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(completed, intent, started)
        )

    def _on_result(self, future, intent: _IntentSnapshot, started: float) -> None:
        self._in_flight = False
        elapsed_ms = self._elapsed_ms(started)
        try:
            wrapped = future.result()
            path = wrapped.result.path
        except Exception as exc:
            self._publish_invalid(intent, "planner_result_error", elapsed_ms)
            self.get_logger().warning("Planner result failed: %s" % exc)
            return
        if elapsed_ms > self._discard_after_ms:
            self._publish_invalid(intent, "planner_late", elapsed_ms)
            return
        if str(path.header.frame_id) != "base_link":
            self._publish_invalid(intent, "path_frame_mismatch", elapsed_ms)
            return
        points = tuple(
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in path.poses
        )
        goal = temporary_goal(
            intent.lateral, intent.longitudinal, self._config.goal_distance_m
        )
        validation = validate_path(points, goal, self._config)
        if not validation.valid:
            self._publish_invalid(intent, validation.reason, elapsed_ms)
            return
        try:
            planned = path_steering(points, self._config)
            assisted, reason = bound_assisted_steering(
                intent.source_steering,
                intent.intent_class,
                planned,
                # Generate observable shadow suggestions even when the Pi
                # delegates zero authority. The supervisor independently
                # reapplies the current packet's advertised authority.
                self._config.maximum_assist,
                self._config,
            )
        except ValueError:
            self._publish_invalid(intent, "invalid_path_steering", elapsed_ms)
            return
        if intent.intent_class == FORWARD:
            assisted, confirmed = self._side_hysteresis.filter(assisted)
            if not confirmed:
                self._publish_invalid(intent, "side_switch_confirmation", elapsed_ms)
                return
        else:
            self._side_hysteresis.reset()
        if reason != "assisted":
            self._publish_invalid(intent, reason, elapsed_ms)
            return
        self._publish_suggestion(intent, assisted, True, "accepted", elapsed_ms)
        self._path_pub.publish(path)

    @staticmethod
    def _elapsed_ms(started: float) -> float:
        return (time.monotonic() - started) * 1000.0

    def _publish_invalid(
        self, intent: _IntentSnapshot, reason: str, elapsed_ms: float
    ) -> None:
        self._publish_suggestion(
            intent, intent.source_steering, False, reason, elapsed_ms
        )

    def _publish_suggestion(
        self,
        intent: _IntentSnapshot,
        steering: float,
        valid: bool,
        reason: str,
        elapsed_ms: float,
    ) -> None:
        now = self.get_clock().now().to_msg()
        suggestion = AvoidanceSuggestion()
        suggestion.header.stamp = now
        suggestion.header.frame_id = "base_link"
        suggestion.session_id = intent.session_id
        suggestion.intent_sequence = intent.sequence
        suggestion.intent_class = intent.intent_class
        suggestion.source_steering = intent.source_steering
        suggestion.suggested_steering = steering
        suggestion.valid = valid
        suggestion.reason = reason
        suggestion.planning_time_ms = elapsed_ms
        self._suggestion_pub.publish(suggestion)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now
        status = DiagnosticStatus()
        status.name = "wheelchair_obstacle_avoidance/local_planner"
        status.hardware_id = "nav2_planner"
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = reason
        status.values = [
            KeyValue(key="session_id", value=intent.session_id),
            KeyValue(key="intent_sequence", value=str(intent.sequence)),
            KeyValue(key="planning_time_ms", value="%.3f" % elapsed_ms),
            KeyValue(key="source_steering", value="%.4f" % intent.source_steering),
            KeyValue(key="suggested_steering", value="%.4f" % steering),
        ]
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = LocalAvoidancePlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
