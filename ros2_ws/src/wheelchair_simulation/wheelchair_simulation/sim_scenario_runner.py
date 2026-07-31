"""Deterministic supervised-control scenarios for Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import uuid

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope


SCENARIOS = (
    "missing_intent",
    "clear_forward",
    "obstacle_slow",
    "obstacle_stop",
    "right_sweep_blocked",
    "moving_dummy_stop",
    "left_unobserved",
    "reverse_disabled",
    "stale_intent",
)


@dataclass(frozen=True)
class ScenarioObservation:
    decision: int | None
    reason: str
    linear_mps: float
    angular_radps: float
    displacement_x_m: float


def scenario_passes(name: str, observation: ScenarioObservation) -> bool:
    """Return whether one integrated scenario reached its safe outcome."""

    stopped = (
        abs(observation.linear_mps) <= 1e-4
        and abs(observation.angular_radps) <= 1e-4
    )
    if name == "missing_intent":
        return observation.reason == "missing_intent" and stopped
    if name == "clear_forward":
        return (
            observation.decision == SafetyEnvelope.CLEAR
            and observation.reason == "clear"
            and observation.linear_mps > 0.0
        )
    if name == "obstacle_slow":
        return (
            observation.decision == SafetyEnvelope.SLOW
            and observation.reason == "obstacle_slow"
            and observation.linear_mps > 0.0
        )
    if name in (
        "obstacle_stop",
        "right_sweep_blocked",
        "moving_dummy_stop",
    ):
        return (
            observation.decision == SafetyEnvelope.STOP
            and observation.reason == "obstacle_stop"
            and stopped
        )
    if name == "left_unobserved":
        return observation.reason == "left_turn_unobserved" and stopped
    if name == "reverse_disabled":
        return observation.reason == "reverse_disabled" and stopped
    if name == "stale_intent":
        return observation.reason == "stale_intent" and stopped
    raise ValueError("unknown simulation scenario: %s" % name)


def scenario_command(name: str) -> tuple[float, float, bool] | None:
    """Return steering, forward, and deadman for a scenario."""

    if name == "missing_intent":
        return None
    if name == "right_sweep_blocked":
        return -0.2, 0.5, True
    if name == "left_unobserved":
        return 0.2, 0.5, True
    if name == "reverse_disabled":
        return 0.0, -0.5, True
    return 0.0, 0.5, True


class SimScenarioRunner(Node):
    """Position fixtures, issue intent, and assert fail-closed outcomes."""

    def __init__(self) -> None:
        super().__init__("sim_scenario_runner")
        self.declare_parameter("scenario", "all")
        self.declare_parameter("startup_timeout_s", 90.0)
        self.declare_parameter("robot_spawn_wait_s", 15.0)
        self.declare_parameter("scenario_timeout_s", 8.0)
        self.declare_parameter("settle_time_s", 1.5)
        self.declare_parameter("intent_topic", "/operator_intent")
        self.declare_parameter("envelope_topic", "/safety_envelope")
        self.declare_parameter("cmd_vel_topic", "/sim/safe_cmd_vel")
        self.declare_parameter("odom_topic", "/sim/odom")

        requested = str(self.get_parameter("scenario").value)
        if requested != "all" and requested not in SCENARIOS:
            raise ValueError("scenario must be 'all' or one of %s" % (SCENARIOS,))
        self._scenarios = list(SCENARIOS if requested == "all" else (requested,))
        self._index = 0
        self._phase = "waiting_for_gazebo"
        self._phase_started_s = time.monotonic()
        self._sequence = 0
        self._session_id = "sim-scenario-" + str(uuid.uuid4())
        self._latest_envelope: SafetyEnvelope | None = None
        self._latest_cmd = Twist()
        self._latest_odom_x: float | None = None
        self._start_odom_x: float | None = None
        self._consecutive_passes = 0
        self._stale_publish_stopped = False
        self._shutdown_requested = False
        self.failed = False

        self._intent_pub = self.create_publisher(
            OperatorIntent,
            str(self.get_parameter("intent_topic").value),
            10,
        )
        self.create_subscription(
            SafetyEnvelope,
            str(self.get_parameter("envelope_topic").value),
            self._on_envelope,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_cmd,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self._set_state = self.create_client(
            SetEntityState, "/set_entity_state"
        )
        self.create_timer(0.05, self._tick)

    @property
    def _scenario(self) -> str:
        return self._scenarios[self._index]

    def _on_envelope(self, msg: SafetyEnvelope) -> None:
        self._latest_envelope = msg

    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom_x = float(msg.pose.pose.position.x)

    def _tick(self) -> None:
        now_s = time.monotonic()
        if self._phase == "waiting_for_gazebo":
            if self._set_state.service_is_ready():
                self._phase = "waiting_for_robot"
                self._phase_started_s = now_s
            elif now_s - self._phase_started_s > float(
                self.get_parameter("startup_timeout_s").value
            ):
                self._fail_startup()
            return

        if self._phase == "waiting_for_robot":
            if now_s - self._phase_started_s >= float(
                self.get_parameter("robot_spawn_wait_s").value
            ):
                self.get_logger().info(
                    "Gazebo startup grace period completed."
                )
                self._configure_scene()
                self._phase = "settling"
                self._phase_started_s = now_s
            elif now_s - self._phase_started_s > float(
                self.get_parameter("startup_timeout_s").value
            ):
                self._fail_startup()
            return

        if self._phase == "settling":
            if self._scenario != "missing_intent":
                self._publish_release()
            if now_s - self._phase_started_s >= float(
                self.get_parameter("settle_time_s").value
            ):
                self._start_odom_x = self._latest_odom_x
                self._latest_envelope = None
                self._consecutive_passes = 0
                self._stale_publish_stopped = False
                self._phase = "running"
                self._phase_started_s = now_s
                self.get_logger().info(
                    "Running simulation scenario: %s" % self._scenario
                )
            return

        elapsed_s = now_s - self._phase_started_s
        command = scenario_command(self._scenario)
        if self._scenario == "stale_intent" and elapsed_s >= 0.8:
            self._stale_publish_stopped = True
        if command is not None and not self._stale_publish_stopped:
            self._publish_intent(*command)
        if self._scenario == "moving_dummy_stop":
            crossing_y = min(-1.2 + elapsed_s * 0.5, 1.2)
            self._move_entity("moving_dummy", 1.6, crossing_y, 0.75)

        observation = self._observation()
        if scenario_passes(self._scenario, observation):
            self._consecutive_passes += 1
        else:
            self._consecutive_passes = 0
        if self._consecutive_passes >= 5:
            self.get_logger().info(
                "PASS %s: reason=%s linear=%.3f displacement=%.3f"
                % (
                    self._scenario,
                    observation.reason,
                    observation.linear_mps,
                    observation.displacement_x_m,
                )
            )
            self._advance()
            return

        if elapsed_s > float(
            self.get_parameter("scenario_timeout_s").value
        ):
            self.get_logger().error(
                "FAIL %s: decision=%s reason=%s linear=%.3f angular=%.3f "
                "displacement=%.3f"
                % (
                    self._scenario,
                    observation.decision,
                    observation.reason,
                    observation.linear_mps,
                    observation.angular_radps,
                    observation.displacement_x_m,
                )
            )
            self.failed = True
            self._request_shutdown()

    def _fail_startup(self) -> None:
        self.get_logger().error(
            "FAIL: /set_entity_state and wheelchair state "
            "did not become ready"
        )
        self.failed = True
        self._request_shutdown()

    def _observation(self) -> ScenarioObservation:
        envelope = self._latest_envelope
        displacement = 0.0
        if self._start_odom_x is not None and self._latest_odom_x is not None:
            displacement = self._latest_odom_x - self._start_odom_x
        return ScenarioObservation(
            decision=int(envelope.decision) if envelope is not None else None,
            reason=envelope.reason if envelope is not None else "",
            linear_mps=float(self._latest_cmd.linear.x),
            angular_radps=float(self._latest_cmd.angular.z),
            displacement_x_m=displacement,
        )

    def _advance(self) -> None:
        self._publish_release()
        self._index += 1
        if self._index >= len(self._scenarios):
            self.get_logger().info("All requested simulation scenarios passed.")
            self._request_shutdown()
            return
        self._configure_scene()
        self._phase = "settling"
        self._phase_started_s = time.monotonic()

    def _configure_scene(self) -> None:
        for name, x_m in (
            ("box_obstacle", 1.5),
            ("low_block", 1.8),
            ("pole_obstacle", 2.0),
            ("moving_dummy", 2.2),
        ):
            self._move_entity(name, x_m, 5.0, 0.75 if name == "moving_dummy" else 0.35)
        self._move_entity("wheelchair", 0.0, 0.0, 0.0)
        if self._scenario == "obstacle_slow":
            self._move_entity("box_obstacle", 2.0, 0.0, 0.35)
        elif self._scenario == "obstacle_stop":
            self._move_entity("box_obstacle", 1.4, 0.0, 0.35)
        elif self._scenario == "right_sweep_blocked":
            self._move_entity("box_obstacle", 1.4, -0.35, 0.35)
        elif self._scenario == "moving_dummy_stop":
            self._move_entity("moving_dummy", 1.6, -1.2, 0.75)

    def _move_entity(
        self, name: str, x_m: float, y_m: float, z_m: float
    ) -> None:
        state = EntityState()
        state.name = name
        state.reference_frame = "world"
        state.pose.position.x = x_m
        state.pose.position.y = y_m
        state.pose.position.z = z_m
        state.pose.orientation.w = 1.0
        request = SetEntityState.Request()
        request.state = state
        self._set_state.call_async(request)

    def _publish_release(self) -> None:
        self._publish_intent(0.0, 0.0, False)

    def _publish_intent(
        self, steering: float, forward: float, deadman: bool
    ) -> None:
        self._sequence += 1
        msg = OperatorIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.session_id = self._session_id
        msg.sequence = self._sequence
        msg.steering = steering
        msg.forward = forward
        msg.deadman = deadman
        self._intent_pub.publish(msg)

    def _request_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        threading.Thread(target=rclpy.shutdown, daemon=True).start()


def main() -> int:
    rclpy.init()
    node = SimScenarioRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.failed = True
    finally:
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
