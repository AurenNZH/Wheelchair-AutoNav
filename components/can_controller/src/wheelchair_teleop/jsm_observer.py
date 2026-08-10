"""Decode physical R-Net JSM input with passive or transparent observation."""

from __future__ import annotations

from dataclasses import dataclass
import select
import socket
import struct
import time
from typing import Callable, Optional


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FILTER = getattr(socket, "CAN_RAW_FILTER", 1)
CAN_RAW_PROTOCOL = getattr(socket, "CAN_RAW", 1)


class JsmFrameError(ValueError):
    """Raised when a CAN frame is not a valid physical-JSM position sample."""


@dataclass(frozen=True)
class JsmSample:
    """One decoded R-Net joystick-position frame."""

    wall_time_s: float
    monotonic_s: float
    interval_s: Optional[float]
    can_id: int
    x_raw: int
    y_raw: int
    ros_steering: float
    forward: float
    reverse: float


@dataclass(frozen=True)
class GatewayStats:
    """Frame counters for the transparent observer gateway."""

    forwarded_to_controller: int
    forwarded_to_joystick: int


def joystick_frame_id(device_slot: int) -> int:
    """Return the 29-bit R-Net joystick-position identifier for a device slot."""

    if isinstance(device_slot, bool) or not isinstance(device_slot, int):
        raise ValueError("device_slot must be an integer")
    if not 0 <= device_slot <= 0xF:
        raise ValueError("device_slot must be in [0, 15]")
    return 0x02000000 | (device_slot << 8)


def decode_jsm_frame(
    can_id_with_flags: int,
    payload_length: int,
    payload: bytes,
    *,
    device_slot: int,
    wall_time_s: float = 0.0,
    monotonic_s: float = 0.0,
    interval_s: Optional[float] = None,
) -> JsmSample:
    """Decode one exact ``02000X00#XxYy`` R-Net position frame."""

    expected_id = joystick_frame_id(device_slot)
    if not can_id_with_flags & CAN_EFF_FLAG:
        raise JsmFrameError("joystick position frame must use an extended CAN ID")
    if can_id_with_flags & (CAN_RTR_FLAG | CAN_ERR_FLAG):
        raise JsmFrameError("RTR and error frames are not joystick samples")
    if can_id_with_flags & CAN_EFF_MASK != expected_id:
        raise JsmFrameError("CAN ID does not match the configured JSM slot")
    if payload_length != 2 or len(payload) < 2:
        raise JsmFrameError("joystick position payload must contain exactly two bytes")

    x_raw, y_raw = struct.unpack("=bb", payload[:2])
    if abs(x_raw) > 100 or abs(y_raw) > 100:
        raise JsmFrameError("joystick position is outside the documented [-100, 100] range")

    return JsmSample(
        wall_time_s=float(wall_time_s),
        monotonic_s=float(monotonic_s),
        interval_s=interval_s,
        can_id=expected_id,
        x_raw=x_raw,
        y_raw=y_raw,
        # The Pi/R-Net convention is right-positive; ROS steering is left-positive.
        ros_steering=-float(x_raw) / 100.0,
        forward=max(0.0, float(y_raw) / 100.0),
        reverse=max(0.0, -float(y_raw) / 100.0),
    )


def direction_label(sample: JsmSample, deadzone: int = 0) -> str:
    """Return a diagnostic direction label without modifying the raw sample."""

    if isinstance(deadzone, bool) or not isinstance(deadzone, int):
        raise ValueError("deadzone must be an integer")
    if not 0 <= deadzone <= 99:
        raise ValueError("deadzone must be in [0, 99]")

    x_value = 0 if abs(sample.x_raw) <= deadzone else sample.x_raw
    y_value = 0 if abs(sample.y_raw) <= deadzone else sample.y_raw
    if x_value == 0 and y_value == 0:
        return "neutral"

    longitudinal = "forward" if y_value > 0 else "reverse" if y_value < 0 else ""
    lateral = "right" if x_value > 0 else "left" if x_value < 0 else ""
    return "_".join(part for part in (longitudinal, lateral) if part)


class PhysicalJsmObserver:
    """Listen for physical JSM frames without exposing a CAN transmit operation."""

    def __init__(
        self,
        can_interface: str,
        *,
        device_slot: int = 1,
        receive_timeout_s: float = 0.25,
        socket_factory: Optional[Callable[..., object]] = None,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not can_interface or not isinstance(can_interface, str):
            raise ValueError("can_interface is required")
        if receive_timeout_s <= 0.0:
            raise ValueError("receive_timeout_s must be positive")

        self.can_interface = can_interface
        self.device_slot = device_slot
        self.receive_timeout_s = float(receive_timeout_s)
        self._socket_factory = socket_factory or socket.socket
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._socket = None
        self._last_sample_monotonic = None

        # Validate the slot before opening any operating-system resource.
        joystick_frame_id(self.device_slot)

    def open(self) -> None:
        if self._socket is not None:
            return
        if not hasattr(socket, "AF_CAN"):
            raise OSError("SocketCAN is not available on this platform")

        can_socket = self._socket_factory(
            socket.AF_CAN,
            socket.SOCK_RAW,
            CAN_RAW_PROTOCOL,
        )
        expected = joystick_frame_id(self.device_slot) | CAN_EFF_FLAG
        mask = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG | CAN_EFF_MASK
        can_filter = struct.pack("=II", expected, mask)
        try:
            can_socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, can_filter)
            can_socket.settimeout(self.receive_timeout_s)
            can_socket.bind((self.can_interface,))
        except Exception:
            can_socket.close()
            raise
        self._socket = can_socket

    def receive(self) -> Optional[JsmSample]:
        """Return the next physical JSM sample, or ``None`` after the timeout."""

        if self._socket is None:
            raise RuntimeError("observer is not open")

        while True:
            try:
                frame, _ancillary, message_flags, _address = self._socket.recvmsg(
                    CAN_FRAME_SIZE
                )
            except socket.timeout:
                return None

            # SocketCAN marks frames created by another local socket with
            # MSG_DONTROUTE. Ignoring them keeps this observer focused on the
            # physical JSM rather than keyboard/injection traffic on the Pi.
            if message_flags & getattr(socket, "MSG_DONTROUTE", 4):
                continue
            if len(frame) != CAN_FRAME_SIZE:
                raise JsmFrameError("received an incomplete classical CAN frame")

            can_id, payload_length, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
            now_monotonic = self._monotonic_clock()
            interval_s = None
            if self._last_sample_monotonic is not None:
                interval_s = now_monotonic - self._last_sample_monotonic
            sample = decode_jsm_frame(
                can_id,
                payload_length,
                payload,
                device_slot=self.device_slot,
                wall_time_s=self._wall_clock(),
                monotonic_s=now_monotonic,
                interval_s=interval_s,
            )
            self._last_sample_monotonic = now_monotonic
            return sample

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def __enter__(self) -> "PhysicalJsmObserver":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


class PhysicalJsmGatewayObserver:
    """Bridge an in-line R-Net connection and observe its physical JSM side.

    Every received classical CAN frame is forwarded byte-for-byte to the other
    interface. JSM samples are decoded only from frames that arrived on the
    physical-joystick interface, before those frames are forwarded to the
    controller interface.
    """

    def __init__(
        self,
        controller_interface: str,
        joystick_interface: str,
        *,
        device_slot: int = 1,
        receive_timeout_s: float = 0.25,
        socket_factory: Optional[Callable[..., object]] = None,
        select_function: Callable = select.select,
        wall_clock: Callable[[], float] = time.time,
        monotonic_clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not controller_interface or not isinstance(controller_interface, str):
            raise ValueError("controller_interface is required")
        if not joystick_interface or not isinstance(joystick_interface, str):
            raise ValueError("joystick_interface is required")
        if controller_interface == joystick_interface:
            raise ValueError("controller and joystick interfaces must differ")
        if receive_timeout_s <= 0.0:
            raise ValueError("receive_timeout_s must be positive")

        joystick_frame_id(device_slot)
        self.controller_interface = controller_interface
        self.joystick_interface = joystick_interface
        self.device_slot = device_slot
        self.receive_timeout_s = float(receive_timeout_s)
        self._socket_factory = socket_factory or socket.socket
        self._select = select_function
        self._wall_clock = wall_clock
        self._monotonic_clock = monotonic_clock
        self._controller_socket = None
        self._joystick_socket = None
        self._last_sample_monotonic = None
        self._forwarded_to_controller = 0
        self._forwarded_to_joystick = 0

    def _open_socket(self, interface_name: str):
        can_socket = self._socket_factory(
            socket.AF_CAN,
            socket.SOCK_RAW,
            CAN_RAW_PROTOCOL,
        )
        try:
            can_socket.bind((interface_name,))
        except Exception:
            can_socket.close()
            raise
        return can_socket

    def open(self) -> None:
        if self._controller_socket is not None:
            return
        if not hasattr(socket, "AF_CAN"):
            raise OSError("SocketCAN is not available on this platform")

        controller_socket = self._open_socket(self.controller_interface)
        try:
            joystick_socket = self._open_socket(self.joystick_interface)
        except Exception:
            controller_socket.close()
            raise

        self._controller_socket = controller_socket
        self._joystick_socket = joystick_socket

    def receive(self) -> Optional[JsmSample]:
        """Forward ready traffic and return a physical JSM sample if present."""

        if self._controller_socket is None or self._joystick_socket is None:
            raise RuntimeError("observer gateway is not open")

        readable, _, _ = self._select(
            (self._controller_socket, self._joystick_socket),
            (),
            (),
            self.receive_timeout_s,
        )
        sample = None
        for source_socket in readable:
            if source_socket is self._controller_socket:
                destination_socket = self._joystick_socket
                observe = False
            elif source_socket is self._joystick_socket:
                destination_socket = self._controller_socket
                observe = True
            else:
                continue

            frame = source_socket.recv(CAN_FRAME_SIZE)
            if len(frame) != CAN_FRAME_SIZE:
                raise JsmFrameError("received an incomplete classical CAN frame")

            # Preserve the complete kernel can_frame. The observer never
            # constructs, edits, suppresses, or replaces a bus message.
            destination_socket.send(frame)
            if observe:
                self._forwarded_to_controller += 1
            else:
                self._forwarded_to_joystick += 1

            if not observe:
                continue

            can_id, payload_length, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
            now_monotonic = self._monotonic_clock()
            interval_s = None
            if self._last_sample_monotonic is not None:
                interval_s = now_monotonic - self._last_sample_monotonic
            try:
                candidate = decode_jsm_frame(
                    can_id,
                    payload_length,
                    payload,
                    device_slot=self.device_slot,
                    wall_time_s=self._wall_clock(),
                    monotonic_s=now_monotonic,
                    interval_s=interval_s,
                )
            except JsmFrameError:
                continue

            self._last_sample_monotonic = now_monotonic
            sample = candidate

        return sample

    @property
    def stats(self) -> GatewayStats:
        return GatewayStats(
            forwarded_to_controller=self._forwarded_to_controller,
            forwarded_to_joystick=self._forwarded_to_joystick,
        )

    def close(self) -> None:
        for can_socket in (self._controller_socket, self._joystick_socket):
            if can_socket is not None:
                can_socket.close()
        self._controller_socket = None
        self._joystick_socket = None

    def __enter__(self) -> "PhysicalJsmGatewayObserver":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


__all__ = [
    "JsmFrameError",
    "JsmSample",
    "GatewayStats",
    "PhysicalJsmGatewayObserver",
    "PhysicalJsmObserver",
    "decode_jsm_frame",
    "direction_label",
    "joystick_frame_id",
]
