import json
import math
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


def queue_latest_envelope(
    udp,
    link,
    decision,
    permitted_forward,
    permitted_steering,
    reason,
):
    intent = json.loads(udp.sent[-1][0].decode())
    udp.received.append(
        (
            encode_envelope(
                EnvelopePacket(
                    intent["session"],
                    intent["seq"],
                    decision,
                    permitted_forward,
                    permitted_steering,
                    reason,
                    10.0,
                )
            ),
            ("192.0.2.10", 45451),
        )
    )
    return intent


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

    def test_pi_decoder_rejects_mixed_v2_envelope(self):
        packet = json.loads(
            encode_envelope(
                EnvelopePacket("session", 4, 1, 0.2, 0.0, "slow", 10.0)
            )
        )
        packet["v"] = 2
        with self.assertRaises(ValueError):
            decode_envelope(json.dumps(packet).encode())

    def test_pi_v3_intent_is_wire_compatible_with_jetson_decoder(self):
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
        self.assertEqual(decoded.max_steering_assist, 0.0)
        self.assertTrue(decoded.deadman)

    def test_explicit_full_assist_allows_thirty_degree_slow_steering(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.90,
            slow_command_cap=0.60,
            max_steering_assist=0.577350269,
            udp_socket=udp,
            monotonic_clock=clock,
        )

        self.assertEqual(link.apply(0, 100, True), (0, 0))
        intent = json.loads(udp.sent[-1][0].decode())
        self.assertEqual(intent["v"], 3)
        self.assertEqual(intent["max_steering_assist"], 0.577350269)
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 1, 0.60,
                        0.577350269, "nav2_cost_slow", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(0, 100, True), (-35, 60))

    def test_assist_over_advertised_authority_latches_fail_closed(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            max_steering_assist=0.30,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, 100, True)
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 1.0, 0.31,
                        "bad_assist", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(0, 100, True), (0, 0))
        self.assertTrue(link.get_status()["stop_latched"])
        self.assertEqual(
            link.get_status()["reason"], "invalid_safety_envelope_limit"
        )

    def test_assist_configuration_above_supported_ceiling_is_rejected(self):
        with self.assertRaisesRegex(ValueError, r"\[0, 0\.577350269\]"):
            SafetyLink(
                enabled=True,
                jetson_address="192.0.2.10",
                max_steering_assist=0.58,
                udp_socket=FakeSocket(),
            )

    def test_operator_release_clears_stop_latch_but_does_not_move(self):
        udp = FakeSocket()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            auto_resume_obstacle_stops=True,
            udp_socket=udp,
        )
        link.emergency_stop()
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertTrue(link.get_status()["stop_latched"])
        self.assertEqual(link.apply(0, 0, False), (0, 0))
        self.assertFalse(link.get_status()["stop_latched"])

    def test_obstacle_stop_auto_resumes_held_forward_after_five_envelopes(self):
        for decision, reason, permitted, expected in (
            (1, "nav2_cost_slow", 0.75, (0, 60)),
            (2, "nav2_cost_clear", 1.00, (0, 90)),
        ):
            with self.subTest(decision=decision):
                udp = FakeSocket()
                clock = FakeClock()
                link = SafetyLink(
                    enabled=True,
                    jetson_address="192.0.2.10",
                    required_clear_envelopes=5,
                    command_cap=0.90,
                    slow_command_cap=0.60,
                    heartbeat_hz=20.0,
                    auto_resume_obstacle_stops=True,
                    udp_socket=udp,
                    monotonic_clock=clock,
                )

                self.assertEqual(link.apply(0, 100, True), (0, 0))
                queue_latest_envelope(
                    udp, link, 0, 0.0, 0.0, "nav2_cost_stop"
                )
                clock.advance(0.01)
                self.assertEqual(link.apply(0, 100, True), (0, 0))
                self.assertFalse(link.get_status()["stop_latched"])

                for index in range(5):
                    clock.advance(0.05)
                    self.assertEqual(link.apply(0, 100, True), (0, 0))
                    queue_latest_envelope(
                        udp, link, decision, permitted, 0.0, reason
                    )
                    clock.advance(0.01)
                    output = link.apply(0, 100, True)
                    self.assertEqual(output, expected if index == 4 else (0, 0))

                self.assertEqual(link.get_status()["clear_count"], 5)

    def test_obstacle_stop_resets_auto_resume_progress(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=5,
            command_cap=0.90,
            heartbeat_hz=20.0,
            auto_resume_obstacle_stops=True,
            udp_socket=udp,
            monotonic_clock=clock,
        )

        link.apply(0, 100, True)
        queue_latest_envelope(udp, link, 0, 0.0, 0.0, "nav2_cost_stop")
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 100, True), (0, 0))

        for _ in range(3):
            clock.advance(0.05)
            link.apply(0, 100, True)
            queue_latest_envelope(
                udp, link, 2, 1.0, 0.0, "nav2_cost_clear"
            )
            clock.advance(0.01)
            self.assertEqual(link.apply(0, 100, True), (0, 0))
        self.assertEqual(link.get_status()["clear_count"], 3)

        clock.advance(0.05)
        link.apply(0, 100, True)
        queue_latest_envelope(udp, link, 0, 0.0, 0.0, "nav2_cost_stop")
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 100, True), (0, 0))
        self.assertEqual(link.get_status()["clear_count"], 0)

        for index in range(5):
            clock.advance(0.05)
            self.assertEqual(link.apply(0, 100, True), (0, 0))
            queue_latest_envelope(
                udp, link, 2, 1.0, 0.0, "nav2_cost_clear"
            )
            clock.advance(0.01)
            output = link.apply(0, 100, True)
            self.assertEqual(output, (0, 90) if index == 4 else (0, 0))

    def test_obstacle_resume_uses_current_forward_input_and_slow_caps(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=5,
            command_cap=0.90,
            slow_command_cap=0.60,
            heartbeat_hz=20.0,
            auto_resume_obstacle_stops=True,
            udp_socket=udp,
            monotonic_clock=clock,
        )

        link.apply(0, 100, True)
        queue_latest_envelope(udp, link, 0, 0.0, 0.0, "nav2_cost_stop")
        clock.advance(0.01)
        link.apply(-14, 80, True)

        for index in range(5):
            clock.advance(0.05)
            self.assertEqual(link.apply(-14, 80, True), (0, 0))
            intent = json.loads(udp.sent[-1][0].decode())
            self.assertAlmostEqual(intent["longitudinal"], 0.80)
            steering_ratio = intent["lateral"] / intent["longitudinal"]
            queue_latest_envelope(
                udp,
                link,
                1,
                0.80,
                steering_ratio,
                "nav2_cost_slow",
            )
            clock.advance(0.01)
            output = link.apply(-14, 80, True)
            self.assertEqual(output, (-11, 60) if index == 4 else (0, 0))

    def test_left_and_right_turn_obstacle_stops_auto_resume_with_caps(self):
        for x_pos, steering in ((-100, 1.0), (100, -1.0)):
            for decision, permitted_steering, expected_x in (
                (1, 0.80 * steering, 60),
                (2, 1.00 * steering, 90),
            ):
                with self.subTest(x_pos=x_pos, decision=decision):
                    udp = FakeSocket()
                    clock = FakeClock()
                    link = SafetyLink(
                        enabled=True,
                        jetson_address="192.0.2.10",
                        required_clear_envelopes=5,
                        turn_command_cap=0.90,
                        slow_turn_command_cap=0.60,
                        turn_longitudinal_cap=0.15,
                        heartbeat_hz=20.0,
                        auto_resume_obstacle_stops=True,
                        udp_socket=udp,
                        monotonic_clock=clock,
                    )

                    link.apply(x_pos, 30, True)
                    queue_latest_envelope(
                        udp,
                        link,
                        0,
                        0.0,
                        0.0,
                        "nav2_turn_cost_stop",
                    )
                    clock.advance(0.01)
                    self.assertEqual(link.apply(x_pos, 30, True), (0, 0))
                    self.assertFalse(link.get_status()["stop_latched"])

                    for index in range(5):
                        clock.advance(0.05)
                        self.assertEqual(link.apply(x_pos, 30, True), (0, 0))
                        queue_latest_envelope(
                            udp,
                            link,
                            decision,
                            0.30,
                            permitted_steering,
                            (
                                "nav2_turn_cost_slow"
                                if decision == 1
                                else "nav2_turn_cost_clear"
                            ),
                        )
                        clock.advance(0.01)
                        output = link.apply(x_pos, 30, True)
                        expected = (
                            int(math.copysign(expected_x, x_pos)),
                            15,
                        )
                        self.assertEqual(
                            output, expected if index == 4 else (0, 0)
                        )

    def test_auto_resume_allowlist_keeps_other_stops_latched(self):
        for reason in (
            "stale_source",
            "missing_intent",
            "invalid_intent",
            "intent_class_mismatch",
            "unsupported_intent",
        ):
            with self.subTest(reason=reason):
                udp = FakeSocket()
                clock = FakeClock()
                link = SafetyLink(
                    enabled=True,
                    jetson_address="192.0.2.10",
                    required_clear_envelopes=1,
                    auto_resume_obstacle_stops=True,
                    udp_socket=udp,
                    monotonic_clock=clock,
                )
                link.apply(0, 50, True)
                queue_latest_envelope(udp, link, 0, 0.0, 0.0, reason)
                clock.advance(0.01)
                self.assertEqual(link.apply(0, 50, True), (0, 0))
                self.assertTrue(link.get_status()["stop_latched"])

                clock.advance(0.05)
                link.apply(0, 50, True)
                queue_latest_envelope(
                    udp, link, 2, 0.50, 0.0, "nav2_cost_clear"
                )
                clock.advance(0.01)
                self.assertEqual(link.apply(0, 50, True), (0, 0))
                self.assertTrue(link.get_status()["stop_latched"])
                self.assertEqual(link.apply(0, 0, False), (0, 0))
                self.assertFalse(link.get_status()["stop_latched"])

    def test_disabled_auto_resume_requires_release_for_keyboard_and_rollback(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            auto_resume_obstacle_stops=False,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, 50, True)
        queue_latest_envelope(udp, link, 0, 0.0, 0.0, "nav2_cost_stop")
        clock.advance(0.01)
        self.assertEqual(link.apply(0, 50, True), (0, 0))
        self.assertTrue(link.get_status()["stop_latched"])

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

    def test_reverse_slow_cap_preserves_signed_direction_and_correction(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=0.90,
            slow_command_cap=0.60,
            reverse_command_cap=0.65,
            max_steering_assist=0.30,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        self.assertEqual(link.apply(20, -100, True), (0, 0))
        intent = json.loads(udp.sent[-1][0].decode())
        ratio = intent["lateral"] / abs(intent["longitudinal"])
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 1, 0.65,
                        ratio, "reverse_unmonitored_slow", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(20, -100, True), (13, -65))
        self.assertEqual(
            link.get_status()["reason"], "reverse_unmonitored_slow"
        )

    def test_reverse_rejects_non_slow_envelope(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            command_cap=1.00,
            slow_command_cap=0.65,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(0, -100, True)
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.70, 0.0,
                        "unexpected_reverse_clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(0, -100, True), (0, 0))
        self.assertEqual(
            link.get_status()["reason"], "reverse_requires_slow_envelope"
        )
        self.assertTrue(link.get_status()["stop_latched"])

    def test_turn_envelope_uses_direct_lateral_and_local_caps(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            turn_command_cap=0.90,
            slow_turn_command_cap=0.60,
            turn_longitudinal_cap=0.15,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        self.assertEqual(link.apply(-100, 30, True), (0, 0))
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.15,
                        0.90, "nav2_turn_cost_clear", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(-100, 30, True), (-90, 15))

    def test_invalid_turn_envelope_stays_latched_with_auto_resume(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=1,
            auto_resume_obstacle_stops=True,
            udp_socket=udp,
            monotonic_clock=clock,
        )
        link.apply(-100, 0, True)
        intent = json.loads(udp.sent[-1][0].decode())
        udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"], intent["seq"], 2, 0.0,
                        -0.90, "wrong_turn_direction", 10.0
                    )
                ),
                ("192.0.2.10", 45451),
            )
        )
        clock.advance(0.01)

        self.assertEqual(link.apply(-100, 0, True), (0, 0))
        self.assertTrue(link.get_status()["stop_latched"])
        self.assertEqual(
            link.get_status()["reason"], "invalid_safety_envelope_limit"
        )

    def test_direction_family_change_requires_five_new_envelopes(self):
        udp = FakeSocket()
        clock = FakeClock()
        link = SafetyLink(
            enabled=True,
            jetson_address="192.0.2.10",
            required_clear_envelopes=5,
            command_cap=1.00,
            slow_command_cap=0.65,
            heartbeat_hz=1.0,
            udp_socket=udp,
            monotonic_clock=clock,
        )

        def accept_latest(decision, permitted, reason, x_pos, y_pos):
            intent = json.loads(udp.sent[-1][0].decode())
            ratio = intent["lateral"] / abs(intent["longitudinal"])
            udp.received.append(
                (
                    encode_envelope(
                        EnvelopePacket(
                            intent["session"], intent["seq"], decision,
                            permitted, ratio, reason, 10.0
                        )
                    ),
                    ("192.0.2.10", 45451),
                )
            )
            clock.advance(0.01)
            return link.apply(x_pos, y_pos, True)

        link.apply(0, -100, True)
        for index in range(5):
            output = accept_latest(
                1, 0.65, "reverse_unmonitored_slow", 0, -100
            )
            self.assertEqual(output, (0, -65) if index == 4 else (0, 0))
            if index < 4:
                link._last_send_monotonic = 0.0
                link.apply(0, -100, True)

        link._last_send_monotonic = 0.0
        self.assertEqual(link.apply(0, 100, True), (0, 0))
        self.assertEqual(link.get_status()["clear_count"], 0)
        for index in range(5):
            output = accept_latest(2, 1.0, "nav2_cost_clear", 0, 100)
            self.assertEqual(output, (0, 100) if index == 4 else (0, 0))
            if index < 4:
                link._last_send_monotonic = 0.0
                link.apply(0, 100, True)

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
