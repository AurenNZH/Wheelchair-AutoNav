"""Fail-safe UDP safety-envelope client used only when explicitly enabled."""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
import math
import socket
import time
import uuid


logger = logging.getLogger(__name__)
PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 1024
STOP = 0
SLOW = 1
CLEAR = 2


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Envelope:
    session_id: str
    intent_sequence: int
    decision: int
    permitted_forward: float
    permitted_steering: float
    reason: str
    map_age_ms: float


class SafetyLink:
    """Exchange intent/envelopes and return a fail-safe joystick limit."""

    def __init__(
        self,
        *,
        enabled: bool = False,
        jetson_address: str = "",
        allowed_jetson_address: str = "",
        intent_port: int = 45450,
        envelope_port: int = 45451,
        heartbeat_hz: float = 20.0,
        envelope_timeout_s: float = 0.20,
        required_clear_envelopes: int = 5,
        command_cap: float = 0.20,
        udp_socket=None,
    ):
        self.enabled = bool(enabled)
        self.jetson_address = jetson_address
        self.allowed_jetson_address = allowed_jetson_address or jetson_address
        self.intent_port = int(intent_port)
        self.envelope_port = int(envelope_port)
        self.heartbeat_period_s = 1.0 / float(heartbeat_hz)
        self.envelope_timeout_s = float(envelope_timeout_s)
        self.required_clear_envelopes = int(required_clear_envelopes)
        self.command_cap = float(command_cap)
        self.session_id = str(uuid.uuid4())
        self._socket = udp_socket
        self._owns_socket = udp_socket is None
        self._sequence = 0
        self._last_send_monotonic = 0.0
        self._last_envelope_monotonic = 0.0
        self._last_envelope = None
        self._intent_by_sequence = {}
        self._clear_count = 0
        self._last_counted_intent_sequence = None
        self._stop_latched = False
        self._reason = "shared_control_disabled"

        if self.enabled:
            self._validate_config()
            if self._socket is None:
                self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self._socket.bind(("0.0.0.0", self.envelope_port))
            self._socket.setblocking(False)
            self._reason = "waiting_for_safety_envelope"
            logger.warning(
                "Shared-control UDP enforcement enabled; commands fail closed "
                "until valid envelopes arrive."
            )

    def _validate_config(self):
        if not self.jetson_address:
            raise ValueError("jetson_address is required when shared control is enabled")
        if self.heartbeat_period_s <= 0.0 or self.envelope_timeout_s <= 0.0:
            raise ValueError("heartbeat and timeout must be positive")
        if self.required_clear_envelopes < 1:
            raise ValueError("required_clear_envelopes must be at least one")
        if not 0.0 < self.command_cap <= 1.0:
            raise ValueError("command_cap must be in (0, 1]")

    def apply(self, x_pos: int, y_pos: int, deadman: bool) -> tuple[int, int]:
        """Return steering/forward values permitted by a fresh matching envelope."""

        if not self.enabled:
            return int(x_pos), int(y_pos)

        now = time.monotonic()
        steering = max(-1.0, min(1.0, float(x_pos) / 100.0))
        forward = max(0.0, min(1.0, float(y_pos) / 100.0))
        command = (steering, forward, bool(deadman))
        if now - self._last_send_monotonic >= self.heartbeat_period_s:
            self._send_intent(command, now)
        self._receive_envelopes(now)

        if y_pos <= 0 or not deadman:
            self._stop_latched = False
            self._clear_count = 0
            self._reason = "operator_released"
            return 0, 0
        if self._stop_latched:
            self._reason = "automatic_stop_latched"
            return 0, 0
        if (
            self._last_envelope is None
            or now - self._last_envelope_monotonic > self.envelope_timeout_s
        ):
            self._clear_count = 0
            self._reason = "safety_envelope_timeout"
            return 0, 0

        envelope = self._last_envelope
        matching_intent = self._intent_by_sequence.get(envelope.intent_sequence)
        if matching_intent != command:
            self._clear_count = 0
            self._reason = "envelope_does_not_match_current_intent"
            return 0, 0
        if envelope.decision == STOP:
            self._stop_latched = True
            self._clear_count = 0
            self._last_counted_intent_sequence = envelope.intent_sequence
            self._reason = envelope.reason
            return 0, 0

        if envelope.intent_sequence != self._last_counted_intent_sequence:
            self._clear_count += 1
            self._last_counted_intent_sequence = envelope.intent_sequence
        if self._clear_count < self.required_clear_envelopes:
            self._reason = "waiting_for_clear_envelopes"
            return 0, 0

        permitted_forward = min(envelope.permitted_forward, self.command_cap)
        permitted_steering = max(
            -self.command_cap,
            min(self.command_cap, envelope.permitted_steering),
        )
        self._reason = envelope.reason
        return int(round(permitted_steering * 100.0)), int(
            round(permitted_forward * 100.0)
        )

    def _send_intent(self, command: tuple[float, float, bool], now: float) -> None:
        self._sequence += 1
        steering, forward, deadman = command
        payload = {
            "v": PROTOCOL_VERSION,
            "type": "intent",
            "session": self.session_id,
            "seq": self._sequence,
            "steering": steering,
            "forward": forward,
            "deadman": deadman,
        }
        data = json.dumps(
            payload, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
        self._socket.sendto(data, (self.jetson_address, self.intent_port))
        self._last_send_monotonic = now
        self._intent_by_sequence[self._sequence] = command
        for sequence in list(self._intent_by_sequence):
            if sequence < self._sequence - 32:
                del self._intent_by_sequence[sequence]

    def _receive_envelopes(self, now: float) -> None:
        for _ in range(32):
            try:
                data, peer = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                return
            if self.allowed_jetson_address and peer[0] != self.allowed_jetson_address:
                continue
            try:
                envelope = decode_envelope(data)
            except ProtocolError:
                continue
            if envelope.session_id != self.session_id:
                continue
            if (
                self._last_envelope is not None
                and envelope.intent_sequence < self._last_envelope.intent_sequence
            ):
                continue
            self._last_envelope = envelope
            self._last_envelope_monotonic = now

    def emergency_stop(self) -> None:
        if self.enabled:
            self._stop_latched = True
            self._clear_count = 0
            self._reason = "operator_emergency_stop"

    def close(self) -> None:
        if self._socket is not None and self._owns_socket:
            self._socket.close()
        self._socket = None

    def get_status(self) -> dict:
        return {
            "enabled": self.enabled,
            "session_id": self.session_id,
            "reason": self._reason,
            "stop_latched": self._stop_latched,
            "clear_count": self._clear_count,
        }


def decode_envelope(data: bytes) -> Envelope:
    if not data or len(data) > MAX_PACKET_BYTES:
        raise ProtocolError("invalid packet size")
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("packet is not valid JSON") from exc
    if not isinstance(payload, dict):
        raise ProtocolError("packet root must be an object")
    if payload.get("v") != PROTOCOL_VERSION or payload.get("type") != "envelope":
        raise ProtocolError("unsupported packet version or type")

    session = payload.get("session")
    reason = payload.get("reason")
    sequence = payload.get("intent_seq")
    decision = payload.get("decision")
    forward = payload.get("permitted_forward")
    steering = payload.get("permitted_steering")
    map_age_ms = payload.get("map_age_ms")
    if not isinstance(session, str) or not session or len(session) > 64:
        raise ProtocolError("invalid session")
    if not isinstance(reason, str) or len(reason) > 96:
        raise ProtocolError("invalid reason")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ProtocolError("invalid sequence")
    if isinstance(decision, bool) or decision not in (STOP, SLOW, CLEAR):
        raise ProtocolError("invalid decision")
    for name, value in (
        ("permitted_forward", forward),
        ("permitted_steering", steering),
        ("map_age_ms", map_age_ms),
    ):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError("invalid %s" % name)
        if not math.isfinite(float(value)):
            raise ProtocolError("non-finite %s" % name)
    forward = float(forward)
    steering = float(steering)
    map_age_ms = float(map_age_ms)
    if not 0.0 <= forward <= 1.0 or not -1.0 <= steering <= 1.0:
        raise ProtocolError("envelope limit outside normalized range")
    if map_age_ms < 0.0:
        raise ProtocolError("invalid map age")
    if decision == STOP and (forward != 0.0 or steering != 0.0):
        raise ProtocolError("STOP envelope permits motion")
    return Envelope(
        session,
        sequence,
        decision,
        forward,
        steering,
        reason,
        map_age_ms,
    )


__all__ = [
    "CLEAR",
    "Envelope",
    "ProtocolError",
    "SLOW",
    "STOP",
    "SafetyLink",
    "decode_envelope",
]
