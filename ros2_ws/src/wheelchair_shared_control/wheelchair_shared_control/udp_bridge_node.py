"""Disabled-by-default UDP adapter between the Raspberry Pi and ROS."""

from __future__ import annotations

import socket

import rclpy
from rclpy.node import Node
from wheelchair_msgs.msg import OperatorIntent, SafetyEnvelope

from wheelchair_shared_control.protocol import (
    EnvelopePacket,
    ProtocolError,
    decode_intent,
    encode_envelope,
)


class UdpBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("shared_control_udp_bridge")
        self.declare_parameter("enable_udp", False)
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("intent_port", 45450)
        self.declare_parameter("envelope_port", 45451)
        self.declare_parameter("pi_address", "")
        self.declare_parameter("allowed_pi_address", "")
        self.declare_parameter("operator_intent_topic", "/operator_intent")
        self.declare_parameter("safety_envelope_topic", "/safety_envelope")

        self._enabled = bool(self.get_parameter("enable_udp").value)
        self._socket = None
        self._last_sequence_by_session = {}
        self._latest_pi_address = None
        self._intent_pub = self.create_publisher(
            OperatorIntent, self.get_parameter("operator_intent_topic").value, 10
        )
        self.create_subscription(
            SafetyEnvelope,
            self.get_parameter("safety_envelope_topic").value,
            self._on_envelope,
            10,
        )

        if self._enabled:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setblocking(False)
            self._socket.bind(
                (
                    str(self.get_parameter("bind_address").value),
                    int(self.get_parameter("intent_port").value),
                )
            )
            self.create_timer(0.01, self._poll_intents)
            self.get_logger().warn(
                "UDP shared-control bridge enabled; supervisor remains fail-closed "
                "unless motion and calibrated geometry are explicitly enabled."
            )
        else:
            self.get_logger().info("UDP shared-control bridge disabled.")

    def _poll_intents(self) -> None:
        for _ in range(32):
            try:
                data, peer = self._socket.recvfrom(1025)
            except BlockingIOError:
                return
            allowed = str(self.get_parameter("allowed_pi_address").value)
            if allowed and peer[0] != allowed:
                self.get_logger().warn(
                    "Ignored intent from unapproved address %s" % peer[0],
                    throttle_duration_sec=5.0,
                )
                continue
            try:
                packet = decode_intent(data)
            except ProtocolError as exc:
                self.get_logger().warn(
                    "Ignored invalid intent packet: %s" % exc,
                    throttle_duration_sec=5.0,
                )
                continue
            previous = self._last_sequence_by_session.get(packet.session_id, -1)
            if packet.sequence <= previous:
                continue
            self._last_sequence_by_session[packet.session_id] = packet.sequence
            self._latest_pi_address = peer[0]

            msg = OperatorIntent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.session_id = packet.session_id
            msg.sequence = packet.sequence
            msg.steering = packet.steering
            msg.forward = packet.forward
            msg.deadman = packet.deadman
            self._intent_pub.publish(msg)

    def _on_envelope(self, msg: SafetyEnvelope) -> None:
        if not self._enabled or self._socket is None:
            return
        configured_pi = str(self.get_parameter("pi_address").value)
        destination = configured_pi or self._latest_pi_address
        if not destination or msg.session_id in ("no-session", "invalid-session"):
            return
        try:
            data = encode_envelope(
                EnvelopePacket(
                    session_id=msg.session_id,
                    intent_sequence=int(msg.intent_sequence),
                    decision=int(msg.decision),
                    permitted_forward=float(msg.permitted_forward),
                    permitted_steering=float(msg.permitted_steering),
                    reason=msg.reason,
                    map_age_ms=float(msg.map_age_ms),
                )
            )
            self._socket.sendto(
                data,
                (destination, int(self.get_parameter("envelope_port").value)),
            )
        except (OSError, ProtocolError) as exc:
            self.get_logger().error(
                "Failed to send safety envelope: %s" % exc,
                throttle_duration_sec=5.0,
            )

    def destroy_node(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node = None
    try:
        node = UdpBridgeNode()
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
