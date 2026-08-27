"""Gazebo-free operator-intent source for recorded-map validation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
import uuid

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from wheelchair_msgs.msg import OperatorIntent

from wheelchair_shared_control.operator_intent import (
    FORWARD as FORWARD_INTENT,
    LEFT_TURN as LEFT_TURN_INTENT,
    RELEASED as RELEASED_INTENT,
    RIGHT_TURN as RIGHT_TURN_INTENT,
)


RELEASED = "released"
FORWARD = "forward"
LEFT_TURN = "left_turn"
RIGHT_TURN = "right_turn"
COMMANDS = (RELEASED, FORWARD, LEFT_TURN, RIGHT_TURN)


@dataclass(frozen=True)
class InjectedCommand:
    lateral: float
    longitudinal: float
    intent_class: int
    deadman: bool


def command_for_preset(
    name: str,
    forward_request: float,
    turn_request: float = 0.5,
) -> InjectedCommand:
    """Return the normalized intent represented by one operator preset."""

    validate_injector_config(
        command=name,
        forward_request=forward_request,
        publish_rate_hz=1.0,
        motion_timeout_s=1.0,
        turn_request=turn_request,
    )
    if name == RELEASED:
        return InjectedCommand(0.0, 0.0, RELEASED_INTENT, False)
    if name == LEFT_TURN:
        return InjectedCommand(
            float(turn_request), 0.0, LEFT_TURN_INTENT, True
        )
    if name == RIGHT_TURN:
        return InjectedCommand(
            -float(turn_request), 0.0, RIGHT_TURN_INTENT, True
        )
    return InjectedCommand(
        0.0,
        float(forward_request),
        FORWARD_INTENT,
        True,
    )


def validate_injector_config(
    *,
    command: str,
    forward_request: float,
    publish_rate_hz: float,
    motion_timeout_s: float,
    turn_request: float = 0.5,
) -> None:
    """Validate startup values without depending on a running ROS graph."""

    if command not in COMMANDS:
        raise ValueError("command must be one of %s" % (COMMANDS,))
    if not math.isfinite(forward_request) or not 0.0 < forward_request <= 1.0:
        raise ValueError("forward_request must be finite and in (0, 1]")
    if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be finite and positive")
    if not math.isfinite(motion_timeout_s) or motion_timeout_s <= 0.0:
        raise ValueError("motion_timeout_s must be finite and positive")
    if not math.isfinite(turn_request) or not 0.0 < turn_request <= 1.0:
        raise ValueError("turn_request must be finite and in (0, 1]")


def motion_lease_expired(
    command: str,
    activated_at_s: float | None,
    now_s: float,
    timeout_s: float,
) -> bool:
    """Return whether an active motion preset has exhausted its wall-time lease."""

    return (
        command != RELEASED
        and activated_at_s is not None
        and now_s - activated_at_s >= timeout_s
    )


class OperatorIntentInjectorNode(Node):
    """Publish test-only operator intent; never publish velocity or access hardware."""

    def __init__(self) -> None:
        super().__init__("operator_intent_injector")
        self.declare_parameter("intent_topic", "/operator_intent")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("command", RELEASED)
        self.declare_parameter("forward_request", 0.5)
        self.declare_parameter("turn_request", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("motion_timeout_s", 30.0)

        self._command = str(self.get_parameter("command").value)
        self._forward_request = float(
            self.get_parameter("forward_request").value
        )
        self._publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self._turn_request = float(self.get_parameter("turn_request").value)
        self._motion_timeout_s = float(
            self.get_parameter("motion_timeout_s").value
        )
        validate_injector_config(
            command=self._command,
            forward_request=self._forward_request,
            publish_rate_hz=self._publish_rate_hz,
            motion_timeout_s=self._motion_timeout_s,
            turn_request=self._turn_request,
        )

        self._publisher = self.create_publisher(
            OperatorIntent,
            str(self.get_parameter("intent_topic").value),
            1,
        )
        self._session_id = "replay-injector-" + str(uuid.uuid4())
        self._sequence = 0
        self._activated_at_s = (
            time.monotonic() if self._command != RELEASED else None
        )
        self._last_published_command = None
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(1.0 / self._publish_rate_hz, self._tick)
        self.get_logger().info(
            "Recorded-map intent injector started. It publishes OperatorIntent "
            "only and has no actuator interface. command=%s lease=%.1f s"
            % (self._command, self._motion_timeout_s)
        )

    def _on_set_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "command":
                if parameter.type_ != Parameter.Type.STRING:
                    return SetParametersResult(
                        successful=False,
                        reason="command must be a string",
                    )
                if str(parameter.value) not in COMMANDS:
                    return SetParametersResult(
                        successful=False,
                        reason="command must be one of %s" % (COMMANDS,),
                    )
            elif parameter.name in (
                "intent_topic",
                "frame_id",
                "forward_request",
                "turn_request",
                "publish_rate_hz",
                "motion_timeout_s",
            ):
                return SetParametersResult(
                    successful=False,
                    reason="restart the injector to change %s" % parameter.name,
                )

        for parameter in parameters:
            if parameter.name == "command":
                self._command = str(parameter.value)
                self._activated_at_s = (
                    time.monotonic() if self._command != RELEASED else None
                )
        return SetParametersResult(successful=True)

    def _tick(self) -> None:
        now_s = time.monotonic()
        if motion_lease_expired(
            self._command,
            self._activated_at_s,
            now_s,
            self._motion_timeout_s,
        ):
            result = self.set_parameters(
                [Parameter("command", Parameter.Type.STRING, RELEASED)]
            )[0]
            if result.successful:
                self.get_logger().info(
                    "Motion intent lease expired; returned to released."
                )
            else:
                self._command = RELEASED
                self._activated_at_s = None
                self.get_logger().error(
                    "Could not synchronize the released parameter: %s"
                    % result.reason
                )
        self._publish_current()

    def _publish_current(self) -> None:
        command = command_for_preset(
            self._command,
            self._forward_request,
            self._turn_request,
        )
        self._sequence += 1
        msg = OperatorIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.session_id = self._session_id
        msg.sequence = self._sequence
        msg.steering = command.lateral
        msg.forward = max(0.0, command.longitudinal)
        msg.lateral = command.lateral
        msg.longitudinal = command.longitudinal
        msg.intent_class = command.intent_class
        msg.deadman = command.deadman
        self._publisher.publish(msg)
        if self._command != self._last_published_command:
            self.get_logger().info(
                "Injected operator command: %s "
                "(lateral=%.3f longitudinal=%.3f deadman=%s)"
                % (
                    self._command,
                    command.lateral,
                    command.longitudinal,
                    command.deadman,
                )
            )
            self._last_published_command = self._command

    def publish_release(self) -> None:
        """Publish an explicit release before orderly shutdown."""

        self._command = RELEASED
        self._activated_at_s = None
        self._publish_current()


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = OperatorIntentInjectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_release()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
