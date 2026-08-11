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

from wheelchair_shared_control.protocol import (
    EnvelopePacket,
    decode_intent,
    encode_envelope,
)
from wheelchair_teleop.safety_link import (
    SafetyLink,
    decode_envelope,
    pi_x_to_ros_steering,
    ros_steering_to_pi_x,
    steering_ratio_to_pi_x,
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


class FakeClock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class SafetyLinkTests(unittest.TestCase):
    def test_disabled_link_preserves_manual_command(self):
        link = SafetyLink(enabled=False)
        self.assertEqual(link.apply(10, 20, True), (10, 20))

    def test_pi_right_positive_is_ros_right_negative_and_round_trips(self):
        self.assertEqual(pi_x_to_ros_steering(25), -0.25)
        self.assertEqual(pi_x_to_ros_steering(-25), 0.25)
        self.assertEqual(ros_steering_to_pi_x(-0.25), 25)
        self.assertEqual(ros_steering_to_pi_x(0.25), -25)
        self.assertEqual(steering_ratio_to_pi_x(0.25, 20), -5)

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
                        permitted_steering=-0.2,
                        reason="clear",
                        map_age_ms=120.0,
                    )
                ),
                ("192.0.2.10", 45450),
            )
        )

        self.assertEqual(sent["lateral"], -0.1)
        self.assertEqual(sent["longitudinal"], 0.5)
        self.assertEqual(sent["intent_class"], 3)
        self.assertEqual(link.apply(10, 50, True), (4, 20))
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

    def test_pi_v2_intent_is_wire_compatible_with_jetson_decoder(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            udp_socket=udp,
        )

        link.apply(-14, 99, True)
        decoded = decode_intent(udp.sent[-1][0])

        self.assertAlmostEqual(decoded.lateral, 0.14)
        self.assertAlmostEqual(decoded.longitudinal, 0.99)
        self.assertEqual(decoded.intent_class, 2)
        self.assertTrue(decoded.deadman)

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

    def test_slow_has_distinct_local_cap(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            slow_command_cap=0.15,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        self.assertEqual(link.apply(0, 80, True), (0, 0))
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 1, 0.35, 0.0,
                        "obstacle_slow", 50.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 80, True), (0, 15))
        self.assertEqual(link.get_status()["latest_decision"], 1)

    def test_default_slow_cap_preserves_existing_keyboard_limit(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, 80, True)
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 1, 0.35, 0.0,
                        "obstacle_slow", 50.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 80, True), (0, 20))

    def test_forward_jitter_uses_same_corridor_and_never_exceeds_current_input(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, 50, True)
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.50, 0.0,
                        "clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 51, True), (0, 20))
        self.assertEqual(link.apply(0, 10, True), (0, 10))

    def test_correction_cap_preserves_direction(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            slow_command_cap=0.15,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        self.assertEqual(link.apply(-14, 99, True), (0, 0))
        intent = json.loads(udp.sent[-1][0].decode())
        ratio = intent["lateral"] / intent["longitudinal"]
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.99,
                        ratio, "clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(-14, 99, True), (-3, 20))

    def test_opposite_correction_uses_authorized_straight_path(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.20,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(-14, 99, True)
        intent = json.loads(udp.sent[-1][0].decode())
        ratio = intent["lateral"] / intent["longitudinal"]
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.99,
                        ratio, "clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(14, 99, True), (0, 20))

    def test_release_prevents_old_clear_from_authorizing_new_motion(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, 50, True)
        first = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        first["session"], first["seq"], 2, 0.50, 0.0,
                        "clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 50, True), (0, 20))
        self.assertEqual(link.apply(0, 0, False), (0, 0))
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertEqual(
            link.get_status()["reason"], "envelope_precedes_current_input"
        )


if __name__ == "__main__":
    unittest.main()
