"""Strict, versioned JSON datagrams for the Jetson-to-Pi safety boundary."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math


PROTOCOL_VERSION = 1
MAX_PACKET_BYTES = 1024


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class IntentPacket:
    session_id: str
    sequence: int
    steering: float
    forward: float
    deadman: bool


@dataclass(frozen=True)
class EnvelopePacket:
    session_id: str
    intent_sequence: int
    decision: int
    permitted_forward: float
    permitted_steering: float
    reason: str
    map_age_ms: float


def encode_intent(packet: IntentPacket) -> bytes:
    _validate_intent(packet)
    return _encode(
        {
            "v": PROTOCOL_VERSION,
            "type": "intent",
            "session": packet.session_id,
            "seq": packet.sequence,
            "steering": packet.steering,
            "forward": packet.forward,
            "deadman": packet.deadman,
        }
    )


def decode_intent(data: bytes) -> IntentPacket:
    payload = _decode(data, "intent")
    packet = IntentPacket(
        session_id=_string(payload, "session", max_length=64),
        sequence=_integer(payload, "seq"),
        steering=_number(payload, "steering"),
        forward=_number(payload, "forward"),
        deadman=_boolean(payload, "deadman"),
    )
    _validate_intent(packet)
    return packet


def encode_envelope(packet: EnvelopePacket) -> bytes:
    _validate_envelope(packet)
    return _encode(
        {
            "v": PROTOCOL_VERSION,
            "type": "envelope",
            "session": packet.session_id,
            "intent_seq": packet.intent_sequence,
            "decision": packet.decision,
            "permitted_forward": packet.permitted_forward,
            "permitted_steering": packet.permitted_steering,
            "reason": packet.reason,
            "map_age_ms": packet.map_age_ms,
        }
    )


def decode_envelope(data: bytes) -> EnvelopePacket:
    payload = _decode(data, "envelope")
    packet = EnvelopePacket(
        session_id=_string(payload, "session", max_length=64),
        intent_sequence=_integer(payload, "intent_seq"),
        decision=_integer(payload, "decision"),
        permitted_forward=_number(payload, "permitted_forward"),
        permitted_steering=_number(payload, "permitted_steering"),
        reason=_string(payload, "reason", max_length=96),
        map_age_ms=_number(payload, "map_age_ms"),
    )
    _validate_envelope(packet)
    return packet


def _encode(payload: dict) -> bytes:
    data = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    if len(data) > MAX_PACKET_BYTES:
        raise ProtocolError("packet is too large")
    return data


def _decode(data: bytes, expected_type: str) -> dict:
    if not data or len(data) > MAX_PACKET_BYTES:
        raise ProtocolError("invalid packet size")
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("packet is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ProtocolError("packet root must be an object")
    if payload.get("v") != PROTOCOL_VERSION or payload.get("type") != expected_type:
        raise ProtocolError("unsupported packet version or type")
    return payload


def _validate_intent(packet: IntentPacket) -> None:
    if not packet.session_id or len(packet.session_id) > 64:
        raise ProtocolError("invalid session")
    if packet.sequence < 0:
        raise ProtocolError("invalid sequence")
    if not -1.0 <= packet.steering <= 1.0:
        raise ProtocolError("steering outside [-1, 1]")
    if not 0.0 <= packet.forward <= 1.0:
        raise ProtocolError("forward outside [0, 1]")
    if not math.isfinite(packet.steering) or not math.isfinite(packet.forward):
        raise ProtocolError("non-finite intent")
    if not isinstance(packet.deadman, bool):
        raise ProtocolError("deadman must be boolean")


def _validate_envelope(packet: EnvelopePacket) -> None:
    if not packet.session_id or len(packet.session_id) > 64:
        raise ProtocolError("invalid session")
    if packet.intent_sequence < 0 or packet.decision not in (0, 1, 2):
        raise ProtocolError("invalid envelope sequence or decision")
    if not 0.0 <= packet.permitted_forward <= 1.0:
        raise ProtocolError("permitted_forward outside [0, 1]")
    if not -1.0 <= packet.permitted_steering <= 1.0:
        raise ProtocolError("permitted_steering outside [-1, 1]")
    if not math.isfinite(packet.map_age_ms) or packet.map_age_ms < 0.0:
        raise ProtocolError("invalid map age")
    if len(packet.reason) > 96:
        raise ProtocolError("reason is too long")
    if packet.decision == 0 and (
        packet.permitted_forward != 0.0 or packet.permitted_steering != 0.0
    ):
        raise ProtocolError("STOP envelope must permit zero motion")


def _string(payload: dict, key: str, max_length: int) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value or len(value) > max_length:
        raise ProtocolError("invalid %s" % key)
    return value


def _integer(payload: dict, key: str) -> int:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError("invalid %s" % key)
    return value


def _number(payload: dict, key: str) -> float:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError("invalid %s" % key)
    return float(value)


def _boolean(payload: dict, key: str) -> bool:
    value = payload.get(key)
    if not isinstance(value, bool):
        raise ProtocolError("invalid %s" % key)
    return value


__all__ = [
    "EnvelopePacket",
    "IntentPacket",
    "MAX_PACKET_BYTES",
    "PROTOCOL_VERSION",
    "ProtocolError",
    "decode_envelope",
    "decode_intent",
    "encode_envelope",
    "encode_intent",
]
