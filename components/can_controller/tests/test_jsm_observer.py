from collections import deque
from pathlib import Path
import socket
import struct
import sys
import unittest


COMPONENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(COMPONENT / "src"))

from wheelchair_teleop.jsm_observer import (
    CAN_EFF_FLAG,
    CAN_FRAME_FORMAT,
    CAN_FRAME_SIZE,
    CAN_RAW_FILTER,
    CAN_RTR_FLAG,
    JsmFrameError,
    JsmSample,
    PhysicalJsmObserver,
    SOL_CAN_RAW,
    decode_jsm_frame,
    direction_label,
    joystick_frame_id,
)


class FakeReceiveSocket:
    """Receive-only fake: intentionally provides no send method."""

    def __init__(self, received=()):
        self.received = deque(received)
        self.options = []
        self.timeout = None
        self.bound = None
        self.closed = False

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))

    def settimeout(self, timeout):
        self.timeout = timeout

    def bind(self, address):
        self.bound = address

    def recvmsg(self, _size):
        if not self.received:
            raise socket.timeout
        return self.received.popleft()

    def close(self):
        self.closed = True


def can_frame(can_id, x_value, y_value, payload_length=2):
    payload = bytes((x_value & 0xFF, y_value & 0xFF))
    return struct.pack(
        CAN_FRAME_FORMAT,
        can_id,
        payload_length,
        payload.ljust(8, b"\x00"),
    )


class JsmDecoderTests(unittest.TestCase):
    def test_frame_id_uses_configured_device_slot(self):
        self.assertEqual(joystick_frame_id(1), 0x02000100)
        self.assertEqual(joystick_frame_id(0xA), 0x02000A00)
        for invalid in (-1, 16, True, 1.0):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    joystick_frame_id(invalid)

    def test_signed_axes_and_ros_normalization(self):
        sample = decode_jsm_frame(
            CAN_EFF_FLAG | 0x02000100,
            2,
            bytes((25, 0xCE)),
            device_slot=1,
        )
        self.assertEqual((sample.x_raw, sample.y_raw), (25, -50))
        self.assertEqual(sample.ros_steering, -0.25)
        self.assertEqual(sample.forward, 0.0)
        self.assertEqual(sample.reverse, 0.5)

    def test_forward_position_is_positive_y(self):
        sample = decode_jsm_frame(
            CAN_EFF_FLAG | 0x02000100,
            2,
            bytes((0, 100)),
            device_slot=1,
        )
        self.assertEqual(sample.forward, 1.0)
        self.assertEqual(sample.reverse, 0.0)

    def test_rejects_non_matching_or_malformed_frames(self):
        cases = (
            (0x02000100, 2, b"\x00\x00"),
            (CAN_EFF_FLAG | CAN_RTR_FLAG | 0x02000100, 2, b"\x00\x00"),
            (CAN_EFF_FLAG | 0x02000200, 2, b"\x00\x00"),
            (CAN_EFF_FLAG | 0x02000100, 1, b"\x00"),
            (CAN_EFF_FLAG | 0x02000100, 2, b"\x65\x00"),
        )
        for can_id, length, payload in cases:
            with self.subTest(can_id=can_id, length=length, payload=payload):
                with self.assertRaises(JsmFrameError):
                    decode_jsm_frame(
                        can_id,
                        length,
                        payload,
                        device_slot=1,
                    )

    def test_direction_deadzone_does_not_change_raw_values(self):
        sample = JsmSample(0.0, 0.0, None, 0x02000100, 3, 50, -0.03, 0.5, 0.0)
        self.assertEqual(direction_label(sample, deadzone=0), "forward_right")
        self.assertEqual(direction_label(sample, deadzone=3), "forward")
        self.assertEqual((sample.x_raw, sample.y_raw), (3, 50))


class PhysicalJsmObserverTests(unittest.TestCase):
    def test_opens_exact_receive_filter_and_binds_selected_interface(self):
        fake = FakeReceiveSocket()
        observer = PhysicalJsmObserver(
            "can1",
            device_slot=1,
            socket_factory=lambda *_args: fake,
        )
        observer.open()

        self.assertEqual(fake.bound, ("can1",))
        self.assertEqual(fake.timeout, 0.25)
        self.assertEqual(len(fake.options), 1)
        level, option, packed_filter = fake.options[0]
        self.assertEqual((level, option), (SOL_CAN_RAW, CAN_RAW_FILTER))
        filter_id, filter_mask = struct.unpack("=II", packed_filter)
        self.assertEqual(filter_id, CAN_EFF_FLAG | 0x02000100)
        self.assertTrue(filter_mask & CAN_EFF_FLAG)

        observer.close()
        self.assertTrue(fake.closed)

    def test_ignores_locally_generated_frame_and_returns_physical_frame(self):
        local_frame = can_frame(CAN_EFF_FLAG | 0x02000100, 10, 10)
        physical_frame = can_frame(CAN_EFF_FLAG | 0x02000100, 20, 30)
        fake = FakeReceiveSocket(
            (
                (local_frame, [], getattr(socket, "MSG_DONTROUTE", 4), ("can1",)),
                (physical_frame, [], 0, ("can1",)),
            )
        )
        times = iter((10.0,))
        observer = PhysicalJsmObserver(
            "can1",
            socket_factory=lambda *_args: fake,
            wall_clock=lambda: 100.0,
            monotonic_clock=lambda: next(times),
        )
        observer.open()

        sample = observer.receive()

        self.assertEqual((sample.x_raw, sample.y_raw), (20, 30))
        self.assertEqual(sample.wall_time_s, 100.0)
        self.assertFalse(hasattr(fake, "send"))

    def test_reports_inter_frame_interval_and_timeout(self):
        first = can_frame(CAN_EFF_FLAG | 0x02000100, 0, 0)
        second = can_frame(CAN_EFF_FLAG | 0x02000100, 0, 1)
        fake = FakeReceiveSocket(
            ((first, [], 0, ("can1",)), (second, [], 0, ("can1",)))
        )
        times = iter((1.0, 1.01))
        observer = PhysicalJsmObserver(
            "can1",
            socket_factory=lambda *_args: fake,
            monotonic_clock=lambda: next(times),
        )
        observer.open()

        self.assertIsNone(observer.receive().interval_s)
        self.assertAlmostEqual(observer.receive().interval_s, 0.01)
        self.assertIsNone(observer.receive())

    def test_rejects_incomplete_kernel_frame(self):
        fake = FakeReceiveSocket(((b"short", [], 0, ("can1",)),))
        observer = PhysicalJsmObserver(
            "can1",
            socket_factory=lambda *_args: fake,
        )
        observer.open()
        with self.assertRaises(JsmFrameError):
            observer.receive()


if __name__ == "__main__":
    unittest.main()
