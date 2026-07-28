"""Move the Gazebo padded dummy across the test corridor."""

import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.node import Node


class MovingDummyNode(Node):
    def __init__(self) -> None:
        super().__init__("moving_dummy")
        self.declare_parameter("entity_name", "moving_dummy")
        self.declare_parameter("x_m", 2.2)
        self.declare_parameter("centre_y_m", 0.0)
        self.declare_parameter("amplitude_m", 1.5)
        self.declare_parameter("crossing_speed_mps", 0.5)
        self._client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self._started_s = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(0.1, self._move)

    def _move(self) -> None:
        if not self._client.service_is_ready():
            return
        amplitude = float(self.get_parameter("amplitude_m").value)
        speed = float(self.get_parameter("crossing_speed_mps").value)
        if amplitude <= 0.0 or speed <= 0.0:
            return
        angular_rate = speed / amplitude
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._started_s
        phase = angular_rate * elapsed

        state = EntityState()
        state.name = str(self.get_parameter("entity_name").value)
        state.reference_frame = "world"
        state.pose.position.x = float(self.get_parameter("x_m").value)
        state.pose.position.y = (
            float(self.get_parameter("centre_y_m").value)
            + amplitude * math.sin(phase)
        )
        state.pose.position.z = 0.75
        state.pose.orientation.w = 1.0
        state.twist.linear.y = speed * math.cos(phase)
        request = SetEntityState.Request()
        request.state = state
        self._client.call_async(request)


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = MovingDummyNode()
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
