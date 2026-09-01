"""ROS client that turns fresh forward intent into bounded Nav2 suggestions."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wheelchair_msgs.msg import AvoidanceSuggestion, OperatorIntent

from wheelchair_obstacle_avoidance.diagnostic_evidence import (
    CostGrid,
    PlanningEvidence,
    RegionEvidence,
    abort_hint,
    collect_planning_evidence,
)
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
from wheelchair_obstacle_avoidance.result_handling import (
    completed_result_failure_reason,
    duration_to_milliseconds,
    optional_milliseconds_text,
    planner_action_status_name,
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
    """Keep one planning request in flight and coalesce newer intent."""

    def __init__(self) -> None:
        super().__init__("local_avoidance_planner")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter(
            "suggestion_topic",
            "/shared_control/nav2_waypoint_suggestion",
        )
        self.declare_parameter("accepted_path_topic", "/local_avoidance/path")
        self.declare_parameter("goal_topic", "/local_avoidance/goal")
        self.declare_parameter(
            "diagnostics_topic", "/local_avoidance/diagnostics"
        )
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("planner_id", "LocalAvoidance")
        self.declare_parameter("costmap_topic", "/nav2_merged_costmap")
        self.declare_parameter(
            "footprint_topic", "/nav2_merged_costmap_footprint"
        )
        self.declare_parameter("planning_rate_hz", 2.0)
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
        self.declare_parameter("planner_search_budget_ms", 30.0)
        self.declare_parameter("path_display_steering_tolerance", 0.05)

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
        self._planner_search_budget_ms = float(
            self.get_parameter("planner_search_budget_ms").value
        )
        self._path_display_steering_tolerance = float(
            self.get_parameter("path_display_steering_tolerance").value
        )
        rate_hz = float(self.get_parameter("planning_rate_hz").value)
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("planning_rate_hz must be positive")
        if (
            not math.isfinite(self._discard_after_ms)
            or self._discard_after_ms <= 0.0
        ):
            raise ValueError("discard_after_ms must be positive")
        if (
            not math.isfinite(self._planner_search_budget_ms)
            or self._planner_search_budget_ms <= 0.0
        ):
            raise ValueError("planner_search_budget_ms must be positive")
        if (
            not math.isfinite(self._path_display_steering_tolerance)
            or self._path_display_steering_tolerance < 0.0
        ):
            raise ValueError(
                "path_display_steering_tolerance cannot be negative"
            )

        self._latest_intent = None
        self._in_flight = False
        self._in_flight_key = None
        self._last_submitted_key = None
        self._latest_costmap = None
        self._latest_footprint = None
        self._accepted_path_visible = False
        self._displayed_intent_class = None
        self._displayed_source_steering = None
        self._counters = {
            "received_intents": 0,
            "eligible_intents": 0,
            "coalesced_intents": 0,
            "submitted_requests": 0,
            "completed_requests": 0,
            "accepted_results": 0,
            "invalid_results": 0,
            "cleared_paths": 0,
        }
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
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            self._on_costmap,
            latched_qos,
        )
        self.create_subscription(
            PolygonStamped,
            self.get_parameter("footprint_topic").value,
            self._on_footprint,
            latched_qos,
        )
        self._planner = ActionClient(
            self,
            ComputePathToPose,
            self.get_parameter("planner_action").value,
        )
        self.create_timer(1.0 / rate_hz, self._request_latest)
        self.get_logger().info(
            "Nav2 waypoint shadow ready: goal=%.2fm "
            "result_age=%.0fms "
            "search_budget=%.0fms assist<=%.2f (suggestions only)"
            % (
                self._config.goal_distance_m,
                self._discard_after_ms,
                self._planner_search_budget_ms,
                self._config.maximum_assist,
            )
        )

    def _on_intent(self, msg: OperatorIntent) -> None:
        self._counters["received_intents"] += 1
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
            self._clear_accepted_path()
            return
        source_steering = lateral / longitudinal
        max_assist = float(msg.max_steering_assist)
        if not math.isfinite(source_steering) or not math.isfinite(max_assist):
            self._latest_intent = None
            self._side_hysteresis.reset()
            self._clear_accepted_path()
            return
        self._counters["eligible_intents"] += 1
        snapshot = _IntentSnapshot(
            session_id=str(msg.session_id),
            sequence=int(msg.sequence),
            intent_class=intent_class,
            lateral=lateral,
            longitudinal=longitudinal,
            source_steering=source_steering,
        )
        key = (snapshot.session_id, snapshot.sequence)
        if self._in_flight and key != self._in_flight_key:
            self._counters["coalesced_intents"] += 1
        self._latest_intent = snapshot
        if self._materially_differs_from_display(snapshot):
            self._clear_accepted_path()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        if str(msg.header.frame_id) != "base_link":
            self.get_logger().warning(
                "Ignoring diagnostic costmap in frame %s" % msg.header.frame_id
            )
            self._latest_costmap = None
            return
        orientation = msg.info.origin.orientation
        yaw = math.atan2(
            2.0
            * (
                float(orientation.w) * float(orientation.z)
                + float(orientation.x) * float(orientation.y)
            ),
            1.0
            - 2.0
            * (
                float(orientation.y) * float(orientation.y)
                + float(orientation.z) * float(orientation.z)
            ),
        )
        try:
            self._latest_costmap = CostGrid(
                width=int(msg.info.width),
                height=int(msg.info.height),
                resolution=float(msg.info.resolution),
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
                origin_yaw=yaw,
                data=tuple(int(value) for value in msg.data),
                stamp_sec=int(msg.header.stamp.sec),
                stamp_nanosec=int(msg.header.stamp.nanosec),
                received_monotonic=time.monotonic(),
            )
        except (TypeError, ValueError) as exc:
            self._latest_costmap = None
            self.get_logger().warning(
                "Ignoring malformed diagnostic costmap: %s" % exc
            )

    def _on_footprint(self, msg: PolygonStamped) -> None:
        if str(msg.header.frame_id) != "base_link":
            self._latest_footprint = None
            self.get_logger().warning(
                "Ignoring diagnostic footprint in frame %s"
                % msg.header.frame_id
            )
            return
        points = tuple(
            (float(point.x), float(point.y))
            for point in msg.polygon.points
        )
        if len(points) < 3 or not all(
            math.isfinite(value) for point in points for value in point
        ):
            self._latest_footprint = None
            self.get_logger().warning(
                "Ignoring malformed diagnostic footprint"
            )
            return
        self._latest_footprint = points

    def _materially_differs_from_display(
        self, intent: _IntentSnapshot
    ) -> bool:
        if not self._accepted_path_visible:
            return False
        return (
            intent.intent_class != self._displayed_intent_class
            or self._displayed_source_steering is None
            or abs(intent.source_steering - self._displayed_source_steering)
            > self._path_display_steering_tolerance
        )

    def _request_latest(self) -> None:
        intent = self._latest_intent
        if intent is None or self._in_flight:
            return
        key = (intent.session_id, intent.sequence)
        if key == self._last_submitted_key:
            return
        if not self._planner.server_is_ready():
            self._publish_invalid(
                intent,
                "planner_unavailable",
                0.0,
                planner_action_status="unavailable",
            )
            return
        try:
            goal_x, goal_y, heading = temporary_goal(
                intent.lateral,
                intent.longitudinal,
                self._config.goal_distance_m,
            )
        except ValueError:
            self._publish_invalid(
                intent,
                "invalid_intent",
                0.0,
                planner_action_status="not_requested",
            )
            return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)
        self._goal_pub.publish(pose)
        evidence = collect_planning_evidence(
            self._latest_costmap,
            self._latest_footprint,
            goal_x,
            goal_y,
            heading,
            now_monotonic=time.monotonic(),
        )
        request = ComputePathToPose.Goal()
        request.pose = pose
        request.planner_id = str(self.get_parameter("planner_id").value)
        self._in_flight = True
        self._in_flight_key = key
        self._last_submitted_key = key
        self._counters["submitted_requests"] += 1
        started = time.monotonic()
        future = self._planner.send_goal_async(request)
        future.add_done_callback(
            lambda completed: self._on_goal_response(
                completed, intent, started, evidence
            )
        )

    def _on_goal_response(
        self,
        future,
        intent: _IntentSnapshot,
        started: float,
        evidence: PlanningEvidence,
    ) -> None:
        try:
            goal_handle = future.result()
        # rclpy futures propagate action transport errors here.
        except Exception as exc:
            self._complete_request()
            self._publish_invalid(
                intent,
                "planner_request_error",
                self._elapsed_ms(started),
                evidence=evidence,
                planner_action_status="request_error",
            )
            self.get_logger().warning("Planner request failed: %s" % exc)
            return
        if goal_handle is None or not goal_handle.accepted:
            self._complete_request()
            self._publish_invalid(
                intent,
                "planner_rejected",
                self._elapsed_ms(started),
                evidence=evidence,
                planner_action_status="rejected",
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._on_result(
                completed, intent, started, evidence
            )
        )

    def _on_result(
        self,
        future,
        intent: _IntentSnapshot,
        started: float,
        evidence: PlanningEvidence,
    ) -> None:
        self._complete_request()
        elapsed_ms = self._elapsed_ms(started)
        try:
            wrapped = future.result()
            result = wrapped.result
            path = result.path
            action_status = planner_action_status_name(wrapped.status)
            nav2_planning_time_ms = duration_to_milliseconds(
                result.planning_time
            )
        except Exception as exc:
            self._publish_invalid(
                intent,
                "planner_result_error",
                elapsed_ms,
                evidence=evidence,
                planner_action_status="result_error",
            )
            self.get_logger().warning("Planner result failed: %s" % exc)
            return
        failure_reason = completed_result_failure_reason(
            status=wrapped.status,
            pose_count=len(path.poses),
            frame_id=str(path.header.frame_id),
            elapsed_ms=elapsed_ms,
            discard_after_ms=self._discard_after_ms,
        )
        if failure_reason is not None:
            self._publish_invalid(
                intent,
                failure_reason,
                elapsed_ms,
                evidence=evidence,
                nav2_planning_time_ms=nav2_planning_time_ms,
                planner_action_status=action_status,
            )
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
            self._publish_invalid(
                intent,
                validation.reason,
                elapsed_ms,
                evidence=evidence,
                nav2_planning_time_ms=nav2_planning_time_ms,
                planner_action_status=action_status,
            )
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
            self._publish_invalid(
                intent,
                "invalid_path_steering",
                elapsed_ms,
                evidence=evidence,
                nav2_planning_time_ms=nav2_planning_time_ms,
                planner_action_status=action_status,
            )
            return
        if intent.intent_class == FORWARD:
            assisted, confirmed = self._side_hysteresis.filter(assisted)
            if not confirmed:
                self._publish_invalid(
                    intent,
                    "side_switch_confirmation",
                    elapsed_ms,
                    evidence=evidence,
                    nav2_planning_time_ms=nav2_planning_time_ms,
                    planner_action_status=action_status,
                )
                return
        else:
            self._side_hysteresis.reset()
        if reason != "assisted":
            self._publish_invalid(
                intent,
                reason,
                elapsed_ms,
                evidence=evidence,
                nav2_planning_time_ms=nav2_planning_time_ms,
                planner_action_status=action_status,
            )
            return
        self._counters["accepted_results"] += 1
        self._publish_suggestion(
            intent,
            assisted,
            True,
            "accepted",
            elapsed_ms,
            evidence=evidence,
            nav2_planning_time_ms=nav2_planning_time_ms,
            planner_action_status=action_status,
        )
        if not self._result_path_is_superseded(intent):
            self._path_pub.publish(path)
            self._accepted_path_visible = True
            self._displayed_intent_class = intent.intent_class
            self._displayed_source_steering = intent.source_steering

    @staticmethod
    def _elapsed_ms(started: float) -> float:
        return (time.monotonic() - started) * 1000.0

    def _complete_request(self) -> None:
        self._in_flight = False
        self._in_flight_key = None
        self._counters["completed_requests"] += 1

    def _result_path_is_superseded(self, intent: _IntentSnapshot) -> bool:
        latest = self._latest_intent
        if latest is None or latest.session_id != intent.session_id:
            return True
        if (latest.session_id, latest.sequence) == (
            intent.session_id,
            intent.sequence,
        ):
            return False
        return (
            latest.intent_class != intent.intent_class
            or abs(latest.source_steering - intent.source_steering)
            > self._path_display_steering_tolerance
        )

    def _clear_accepted_path(self) -> None:
        if not self._accepted_path_visible:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "base_link"
        self._path_pub.publish(path)
        self._accepted_path_visible = False
        self._displayed_intent_class = None
        self._displayed_source_steering = None
        self._counters["cleared_paths"] += 1

    def _publish_invalid(
        self,
        intent: _IntentSnapshot,
        reason: str,
        elapsed_ms: float,
        *,
        evidence: PlanningEvidence | None = None,
        nav2_planning_time_ms: float | None = None,
        planner_action_status: str = "unavailable",
    ) -> None:
        self._clear_accepted_path()
        self._counters["invalid_results"] += 1
        self._publish_suggestion(
            intent,
            intent.source_steering,
            False,
            reason,
            elapsed_ms,
            evidence=evidence,
            nav2_planning_time_ms=nav2_planning_time_ms,
            planner_action_status=planner_action_status,
        )

    def _publish_suggestion(
        self,
        intent: _IntentSnapshot,
        steering: float,
        valid: bool,
        reason: str,
        elapsed_ms: float,
        *,
        evidence: PlanningEvidence | None = None,
        nav2_planning_time_ms: float | None = None,
        planner_action_status: str = "unavailable",
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
            KeyValue(
                key="nav2_planning_time_ms",
                value=optional_milliseconds_text(nav2_planning_time_ms),
            ),
            KeyValue(
                key="planner_action_status",
                value=planner_action_status,
            ),
            KeyValue(
                key="source_steering",
                value="%.4f" % intent.source_steering,
            ),
            KeyValue(key="suggested_steering", value="%.4f" % steering),
            KeyValue(
                key="abort_hint",
                value=abort_hint(
                    planner_action_status,
                    evidence,
                    nav2_planning_time_ms,
                    self._planner_search_budget_ms,
                ),
            ),
        ]
        status.values.extend(self._evidence_values(evidence))
        status.values.extend(
            KeyValue(key=key, value=str(value))
            for key, value in self._counters.items()
        )
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)

    @staticmethod
    def _region_values(prefix: str, region: RegionEvidence | None):
        if region is None:
            return [
                KeyValue(key=prefix + "_state", value="unavailable"),
                KeyValue(key=prefix + "_checked_cells", value="unavailable"),
                KeyValue(key=prefix + "_max_cost", value="unavailable"),
                KeyValue(key=prefix + "_unknown_cells", value="unavailable"),
                KeyValue(key=prefix + "_collision_cells", value="unavailable"),
                KeyValue(key=prefix + "_inflated_cells", value="unavailable"),
                KeyValue(key=prefix + "_outside_cells", value="unavailable"),
            ]
        maximum = "none" if region.max_cost is None else str(region.max_cost)
        return [
            KeyValue(key=prefix + "_state", value=region.state),
            KeyValue(
                key=prefix + "_checked_cells",
                value=str(region.checked_cells),
            ),
            KeyValue(key=prefix + "_max_cost", value=maximum),
            KeyValue(
                key=prefix + "_unknown_cells",
                value=str(region.unknown_cells),
            ),
            KeyValue(
                key=prefix + "_collision_cells",
                value=str(region.collision_cells),
            ),
            KeyValue(
                key=prefix + "_inflated_cells",
                value=str(region.inflated_cells),
            ),
            KeyValue(
                key=prefix + "_outside_cells",
                value=str(region.outside_cells),
            ),
        ]

    def _evidence_values(self, evidence: PlanningEvidence | None):
        if evidence is None:
            available = "false"
            goal_x = goal_y = goal_heading = "unavailable"
            map_age = map_stamp = "unavailable"
            goal_state = goal_cost = "unavailable"
            start = goal = None
            footprint_available = "false"
        else:
            available = str(evidence.costmap_available).lower()
            footprint_available = str(evidence.footprint_available).lower()
            goal_x = "%.3f" % evidence.goal_x
            goal_y = "%.3f" % evidence.goal_y
            goal_heading = "%.4f" % evidence.goal_heading
            map_age = optional_milliseconds_text(evidence.costmap_age_ms)
            if evidence.costmap_stamp_sec is None:
                map_stamp = "unavailable"
            else:
                map_stamp = "%d.%09d" % (
                    evidence.costmap_stamp_sec,
                    evidence.costmap_stamp_nanosec,
                )
            if evidence.goal_center is None:
                goal_state = goal_cost = "unavailable"
            else:
                goal_state = evidence.goal_center.state
                goal_cost = (
                    "none"
                    if evidence.goal_center.cost is None
                    else str(evidence.goal_center.cost)
                )
            start = evidence.start_footprint
            goal = evidence.goal_footprint
        values = [
            KeyValue(key="costmap_available", value=available),
            KeyValue(key="footprint_available", value=footprint_available),
            KeyValue(key="costmap_age_ms", value=map_age),
            KeyValue(key="costmap_stamp", value=map_stamp),
            KeyValue(key="goal_x_m", value=goal_x),
            KeyValue(key="goal_y_m", value=goal_y),
            KeyValue(key="goal_heading_rad", value=goal_heading),
            KeyValue(key="goal_center_state", value=goal_state),
            KeyValue(key="goal_center_cost", value=goal_cost),
        ]
        values.extend(self._region_values("start_footprint", start))
        values.extend(self._region_values("goal_footprint", goal))
        return values


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
