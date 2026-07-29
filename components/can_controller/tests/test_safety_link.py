import json
from pathlib import Path
import sys
import unittest


COMPONENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPONENT / "src"))
sys.path.insert(
    0,
    str(
        COMPONENT.parents[1]
        / "ros2_ws"
        / "src"
        / "wheelchair_shared_control"
    ),
)

from wheelchair_shared_control.protocol import EnvelopePacket, encode_envelope
from wheelchair_teleop.safety_link import (
    SafetyLink,
    decode_envelope,
    pi_x_to_ros_steering,
    ros_steering_to_pi_x,
)


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.received = []
        self.closed = False

    def setblocking(self, _enabled):
        pass

    def sendto(self, data, destination):
        self.sent.append((data, destination))

    def recvfrom(self, _size):
        if not self.received:
            raise BlockingIOError
        return self.received.pop(0)

    def close(self):
        self.closed = True


class SafetyLinkTests(unittest.TestCase):
    def test_disabled_link_preserves_manual_command(self):
        link = SafetyLink(enabled=False)
        self.assertEqual(link.apply(10, 20, True), (10, 20))

    def test_pi_right_positive_is_ros_right_negative_and_round_trips(self):
        self.assertEqual(pi_x_to_ros_steering(25), -0.25)
        self.assertEqual(pi_x_to_ros_steering(-25), 0.25)
        self.assertEqual(ros_steering_to_pi_x(-0.25), 25)
        self.assertEqual(ros_steering_to_pi_x(0.25), -25)

    def test_enabled_link_fails_closed_until_matching_envelope(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            allowed_jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            heartbeat_hz=1.0,
            udp_socket=udp,
        )

        self.assertEqual(link.apply(10, 50, True), (0, 0))
        sent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        session_id=sent["session"],
                        intent_sequence=sent["seq"],
                        decision=2,
                        permitted_forward=0.5,
                        permitted_steering=-0.1,
                        reason="clear",
                        map_age_ms=120.0,
                    )
                ),
                ("192.0.2.10", 45450),
            )
        )

        self.assertEqual(sent["steering"], -0.1)
        self.assertEqual(link.apply(10, 50, True), (10, 20))
        self.assertEqual(link.get_status()["reason"], "clear")

    def test_wrong_sender_and_malformed_packet_are_ignored(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            allowed_jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            udp_socket=udp,
        )
        udp.received.extend(
            [
                (b"not-json", ("192.0.2.10", 45450)),
                (b"{}", ("192.0.2.11", 45450)),
            ]
        )
        self.assertEqual(link.apply(0, 50, True), (0, 0))

    def test_clear_handshake_counts_distinct_intent_sequences(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=2,
            command_cap=0.20,
            heartbeat_hz=1.0,
            udp_socket=udp,
        )

        self.assertEqual(link.apply(0, 50, True), (0, 0))
        first_intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        session_id=first_intent["session"],
                        intent_sequence=first_intent["seq"],
                        decision=2,
                        permitted_forward=0.5,
                        permitted_steering=0.0,
                        reason="clear",
                        map_age_ms=10.0,
                    )
                ),
                ("192.0.2.10", 45450),
            )
        )
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertEqual(link.get_status()["clear_count"], 1)

        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertEqual(link.get_status()["clear_count"], 1)

        link._last_send_monotonic = 0.0
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        second_intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        session_id=second_intent["session"],
                        intent_sequence=second_intent["seq"],
                        decision=2,
                        permitted_forward=0.5,
                        permitted_steering=0.0,
                        reason="clear",
                        map_age_ms=10.0,
                    )
                ),
                ("192.0.2.10", 45450),
            )
        )
        self.assertEqual(link.apply(0, 50, True), (0, 20))
        self.assertEqual(link.get_status()["clear_count"], 2)

    def test_shared_protocol_is_wire_compatible_with_pi_decoder(self):
        packet = EnvelopePacket("session", 4, 1, 0.2, -0.1, "slow", 150.0)
        decoded = decode_envelope(encode_envelope(packet))
        self.assertEqual(decoded.session_id, packet.session_id)
        self.assertEqual(decoded.intent_sequence, packet.intent_sequence)
        self.assertEqual(decoded.permitted_forward, packet.permitted_forward)

    def test_operator_release_clears_stop_latch_but_does_not_move(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            udp_socket=udp,
        )
        link.emergency_stop()
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertTrue(link.get_status()["stop_latched"])
        self.assertEqual(link.apply(0, 0, False), (0, 0))
        self.assertFalse(link.get_status()["stop_latched"])


if __name__ == "__main__":
    unittest.main()
