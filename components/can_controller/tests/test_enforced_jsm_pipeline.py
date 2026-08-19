from collections import deque
import json
from pathlib import Path
import struct
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

from wheelchair_shared_control.protocol import (  # noqa: E402
    EnvelopePacket,
    encode_envelope,
)
from wheelchair_teleop.jsm_observer import (  # noqa: E402
    CAN_EFF_FLAG,
    CAN_FRAME_FORMAT,
    PhysicalJsmGatewayObserver,
)
from wheelchair_teleop.physical_shared_control import (  # noqa: E402
    PhysicalJsmSharedControl,
)
from wheelchair_teleop.safety_link import (  # noqa: E402
    CLEAR,
    SLOW,
    STOP,
    SafetyLink,
)


JETSON_ADDRESS = "192.0.2.10"


class FakeClock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeUdpSocket:
    def __init__(self):
        self.sent = []
        self.received = deque()

    def setblocking(self, _enabled):
        pass

    def sendto(self, data, destination):
        self.sent.append((data, destination))

    def recvfrom(self, _size):
        if not self.received:
            raise BlockingIOError
        return self.received.popleft()

    def close(self):
        pass


class FakeCanSocket:
    def __init__(self, received=()):
        self.received = deque(received)
        self.sent = []

    def bind(self, _address):
        pass

    def recv(self, _size):
        return self.received.popleft()

    def send(self, frame):
        self.sent.append(frame)
        return len(frame)

    def close(self):
        pass


def jsm_frame(x_raw, y_raw):
    payload = bytes((x_raw & 0xFF, y_raw & 0xFF)) + b"safety"
    return struct.pack(
        CAN_FRAME_FORMAT,
        CAN_EFF_FLAG | 0x02000200,
        2,
        payload,
    )


def forwarded_axes(frame):
    can_id, payload_length, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
    axes = struct.unpack("=bb", payload[:2])
    return can_id, payload_length, axes, payload[2:]


class EnforcedJsmPipeline:
    def __init__(self, frames):
        self.clock = FakeClock()
        self.udp = FakeUdpSocket()
        self.link = SafetyLink(
            enabled=True,
            jetson_address=JETSON_ADDRESS,
            allowed_jetson_address=JETSON_ADDRESS,
            required_clear_envelopes=1,
            command_cap=0.70,
            slow_command_cap=0.40,
            heartbeat_hz=20.0,
            envelope_timeout_s=0.20,
            neutral_deadzone=4,
            udp_socket=self.udp,
            monotonic_clock=self.clock,
        )
        self.control = PhysicalJsmSharedControl(
            self.link,
            mode="enforce",
            neutral_deadzone=4,
            forward_cone_half_angle_deg=25.0,
        )
        self.controller = FakeCanSocket()
        self.joystick = FakeCanSocket(frames)
        sockets = iter((self.controller, self.joystick))
        self.gateway = PhysicalJsmGatewayObserver(
            "can0",
            "can1",
            device_slot=2,
            socket_factory=lambda *_args: next(sockets),
            select_function=lambda _readers, _writes, _errors, _timeout: (
                [self.joystick],
                [],
                [],
            ),
            jsm_transform=self.control.transform,
            monotonic_clock=self.clock,
        )
        self.gateway.open()

    def forward_once(self):
        self.gateway.receive()
        return forwarded_axes(self.controller.sent[-1])

    def latest_intent(self):
        return json.loads(self.udp.sent[-1][0].decode())

    def queue_envelope(
        self,
        decision,
        permitted_forward,
        permitted_steering,
        reason,
        *,
        peer=JETSON_ADDRESS,
    ):
        intent = self.latest_intent()
        self.udp.received.append(
            (
                encode_envelope(
                    EnvelopePacket(
                        intent["session"],
                        intent["seq"],
                        decision,
                        permitted_forward,
                        permitted_steering,
                        reason,
                        25.0,
                    )
                ),
                (peer, 45451),
            )
        )


class EnforcedJsmPipelineTests(unittest.TestCase):
    def test_clear_slow_and_stop_rewrite_slot_two_frame_axes(self):
        cases = (
            (CLEAR, 0.80, "nav2_cost_clear", (0, 70)),
            (SLOW, 0.40, "nav2_cost_slow", (0, 40)),
            (STOP, 0.0, "nav2_cost_stop", (0, 0)),
        )
        for decision, permitted, reason, expected in cases:
            with self.subTest(decision=decision):
                pipeline = EnforcedJsmPipeline(
                    (jsm_frame(0, 80), jsm_frame(0, 80))
                )

                _, _, waiting_axes, _ = pipeline.forward_once()
                self.assertEqual(waiting_axes, (0, 0))
                pipeline.queue_envelope(decision, permitted, 0.0, reason)
                pipeline.clock.advance(0.01)

                can_id, dlc, axes, trailing = pipeline.forward_once()
                self.assertEqual(can_id, CAN_EFF_FLAG | 0x02000200)
                self.assertEqual(dlc, 2)
                self.assertEqual(axes, expected)
                self.assertEqual(trailing, b"safety")

    def test_clear_correction_preserves_direction_at_reduced_forward_cap(self):
        pipeline = EnforcedJsmPipeline(
            (jsm_frame(-14, 80), jsm_frame(-14, 80))
        )

        pipeline.forward_once()
        intent = pipeline.latest_intent()
        steering_ratio = intent["lateral"] / intent["longitudinal"]
        pipeline.queue_envelope(
            CLEAR,
            0.80,
            steering_ratio,
            "nav2_cost_clear",
        )
        pipeline.clock.advance(0.01)

        _, _, axes, _ = pipeline.forward_once()
        self.assertEqual(axes, (-12, 70))

    def test_reverse_correction_rewrites_signed_slot_two_axes(self):
        pipeline = EnforcedJsmPipeline(
            (jsm_frame(20, -100), jsm_frame(20, -100))
        )

        pipeline.forward_once()
        intent = pipeline.latest_intent()
        steering_ratio = intent["lateral"] / abs(intent["longitudinal"])
        pipeline.queue_envelope(
            SLOW,
            0.40,
            steering_ratio,
            "reverse_unmonitored_slow",
        )
        pipeline.clock.advance(0.01)

        can_id, dlc, axes, trailing = pipeline.forward_once()
        self.assertEqual(can_id, CAN_EFF_FLAG | 0x02000200)
        self.assertEqual(dlc, 2)
        self.assertEqual(axes, (8, -40))
        self.assertEqual(trailing, b"safety")

    def test_wrong_sender_and_expired_envelope_never_forward_raw_motion(self):
        pipeline = EnforcedJsmPipeline(
            (
                jsm_frame(0, 80),
                jsm_frame(0, 80),
                jsm_frame(0, 80),
                jsm_frame(0, 80),
            )
        )

        pipeline.forward_once()
        pipeline.queue_envelope(
            CLEAR,
            0.80,
            0.0,
            "nav2_cost_clear",
            peer="192.0.2.11",
        )
        pipeline.clock.advance(0.01)
        _, _, wrong_peer_axes, _ = pipeline.forward_once()
        self.assertEqual(wrong_peer_axes, (0, 0))

        pipeline.queue_envelope(CLEAR, 0.80, 0.0, "nav2_cost_clear")
        pipeline.clock.advance(0.01)
        _, _, authorized_axes, _ = pipeline.forward_once()
        self.assertEqual(authorized_axes, (0, 70))

        pipeline.clock.advance(0.21)
        _, _, expired_axes, _ = pipeline.forward_once()
        self.assertEqual(expired_axes, (0, 0))

    def test_stop_latches_until_neutral_and_requires_new_authorization(self):
        pipeline = EnforcedJsmPipeline(
            (
                jsm_frame(0, 80),
                jsm_frame(0, 80),
                jsm_frame(0, 0),
                jsm_frame(0, 80),
            )
        )

        pipeline.forward_once()
        pipeline.queue_envelope(STOP, 0.0, 0.0, "nav2_cost_stop")
        pipeline.clock.advance(0.01)
        _, _, stopped_axes, _ = pipeline.forward_once()
        self.assertEqual(stopped_axes, (0, 0))
        self.assertTrue(pipeline.link.get_status()["stop_latched"])

        pipeline.clock.advance(0.01)
        _, _, released_axes, _ = pipeline.forward_once()
        self.assertEqual(released_axes, (0, 0))
        self.assertFalse(pipeline.link.get_status()["stop_latched"])

        pipeline.clock.advance(0.01)
        _, _, unarmed_axes, _ = pipeline.forward_once()
        self.assertEqual(unarmed_axes, (0, 0))


if __name__ == "__main__":
    unittest.main()
