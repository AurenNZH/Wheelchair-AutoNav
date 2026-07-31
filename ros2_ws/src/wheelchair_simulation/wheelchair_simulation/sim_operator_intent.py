"""Keyboard operator-intent source for simulation only."""

from __future__ import annotations

from dataclasses import dataclass
import select
import sys
import termios
import threading
import time
import tty
import uuid

import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import OperatorIntent


@dataclass(frozen=True)
class IntentCommand:
    steering: float
    forward: float
    deadman: bool
    quit_requested: bool = False


def command_for_key(
    key: str,
    *,
    forward_request: float = 0.5,
    turn_request: float = 0.2,
) -> IntentCommand | None:
    """Translate one simulation key into normalized operator intent."""

    commands = {
        "w": IntentCommand(0.0, forward_request, True),
        "d": IntentCommand(-turn_request, forward_request, True),
        "a": IntentCommand(turn_request, forward_request, True),
        "s": IntentCommand(0.0, -forward_request, True),
        " ": IntentCommand(0.0, 0.0, False),
        "x": IntentCommand(0.0, 0.0, False),
        "q": IntentCommand(0.0, 0.0, False, True),
    }
    return commands.get(key.lower())


class SimOperatorIntentNode(Node):
    """Publish fresh intent while keyboard input satisfies a short deadman."""

    def __init__(self) -> None:
        super().__init__("sim_operator_intent")
        self.declare_parameter("intent_topic", "/operator_intent")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("deadman_timeout_s", 0.35)
        self.declare_parameter("forward_request", 0.5)
        self.declare_parameter("turn_request", 0.2)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._deadman_timeout_s = float(
            self.get_parameter("deadman_timeout_s").value
        )
        if rate_hz <= 0.0 or self._deadman_timeout_s <= 0.0:
            raise ValueError("publish rate and deadman timeout must be positive")

        self._publisher = self.create_publisher(
            OperatorIntent,
            str(self.get_parameter("intent_topic").value),
            10,
        )
        self._session_id = "sim-keyboard-" + str(uuid.uuid4())
        self._sequence = 0
        self._command = IntentCommand(0.0, 0.0, False)
        self._last_motion_key_s = 0.0
        self._quit_requested = False
        self._shutdown_requested = False
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            "Simulation controls: W straight, D right, A unsupported-left "
            "test, S reverse-disabled test, Space/X stop, Q quit."
        )

    def _tick(self) -> None:
        key = read_available_key()
        if key:
            command = command_for_key(
                key,
                forward_request=float(
                    self.get_parameter("forward_request").value
                ),
                turn_request=float(
                    self.get_parameter("turn_request").value
                ),
            )
            if command is not None:
                self._command = command
                self._quit_requested = command.quit_requested
                if command.deadman:
                    self._last_motion_key_s = time.monotonic()

        command = self._command
        if (
            command.deadman
            and time.monotonic() - self._last_motion_key_s
            > self._deadman_timeout_s
        ):
            command = IntentCommand(0.0, 0.0, False)
            self._command = command
        self._publish(command)
        if self._quit_requested and not self._shutdown_requested:
            self._shutdown_requested = True
            threading.Thread(target=rclpy.shutdown, daemon=True).start()

    def _publish(self, command: IntentCommand) -> None:
        self._sequence += 1
        msg = OperatorIntent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.session_id = self._session_id
        msg.sequence = self._sequence
        msg.steering = command.steering
        msg.forward = command.forward
        msg.deadman = command.deadman
        self._publisher.publish(msg)


def read_available_key() -> str:
    """Read one terminal character without blocking the ROS executor."""

    if not sys.stdin.isatty():
        return ""
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    return sys.stdin.read(1) if readable else ""


def main() -> int:
    if not sys.stdin.isatty():
        print("sim_operator_intent requires an interactive terminal")
        return 2
    previous = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = SimOperatorIntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, previous)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
