"""Fail-safe UDP safety-envelope client used only when explicitly enabled."""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
import math
import socket
import time
import uuid

from .operator_intent import (
    FORWARD_CLASSES,
    LEFT_TURN,
    RELEASED,
    REVERSE_CLASSES,
    RIGHT_TURN,
    ClassifiedIntent,
    classify_raw_axes,
    intent_label,
)


logger = logging.getLogger(__name__)
PROTOCOL_VERSION = 3
MAX_PACKET_BYTES = 1024
STOP = 0
SLOW = 1
CLEAR = 2
MAX_STEERING_ASSIST = 0.15
RECOVERABLE_OBSTACLE_STOP_REASONS = frozenset(
    ("nav2_cost_stop", "nav2_turn_cost_stop")
)


def pi_x_to_ros_steering(x_pos: int | float) -> float:
    """Convert Pi joystick right-positive X to ROS left-positive steering."""

    return -max(-1.0, min(1.0, float(x_pos) / 100.0))


def ros_steering_to_pi_x(steering: float) -> int:
    """Convert ROS left-positive steering back to Pi right-positive X."""

    bounded = max(-1.0, min(1.0, float(steering)))
    return int(round(-bounded * 100.0))


def steering_ratio_to_pi_x(steering_ratio: float, y_pos: int) -> int:
    """Preserve a left-positive steering ratio at the permitted forward Y."""

    bounded_ratio = max(-1.0, min(1.0, float(steering_ratio)))
    bounded_y = max(0, min(100, int(y_pos)))
    return int(round(-bounded_ratio * bounded_y))


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
        slow_command_cap: float | None = None,
        reverse_command_cap: float = 0.65,
        turn_command_cap: float = 0.90,
        slow_turn_command_cap: float = 0.60,
        turn_longitudinal_cap: float = 0.15,
        neutral_deadzone: int = 5,
        forward_cone_half_angle_deg: float = 30.0,
        max_steering_assist: float = 0.0,
        auto_resume_obstacle_stops: bool = False,
        udp_socket=None,
        monotonic_clock=time.monotonic,
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
        self.slow_command_cap = (
            self.command_cap
            if slow_command_cap is None
            else float(slow_command_cap)
        )
        self.reverse_command_cap = float(reverse_command_cap)
        self.turn_command_cap = float(turn_command_cap)
        self.slow_turn_command_cap = float(slow_turn_command_cap)
        self.turn_longitudinal_cap = float(turn_longitudinal_cap)
        self.neutral_deadzone = int(neutral_deadzone)
        self.forward_cone_half_angle_deg = float(
            forward_cone_half_angle_deg
        )
        self.max_steering_assist = float(max_steering_assist)
        self.auto_resume_obstacle_stops = bool(auto_resume_obstacle_stops)
        self.session_id = str(uuid.uuid4())
        self._socket = udp_socket
        self._owns_socket = udp_socket is None
        self._monotonic_clock = monotonic_clock
        self._sequence = 0
        self._last_send_monotonic = 0.0
        self._last_envelope_monotonic = 0.0
        self._last_envelope = None
        self._intent_by_sequence = {}
        self._sent_monotonic_by_sequence = {}
        self._clear_count = 0
        self._last_counted_intent_sequence = None
        self._minimum_acceptable_sequence = 0
        self._last_command_class = None
        self._stop_latched = False
        self._reason = "shared_control_disabled"
        self._latest_decision = None
        self._latest_map_age_ms = None
        self._latest_round_trip_ms = None

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
        if not 0.0 < self.slow_command_cap <= self.command_cap:
            raise ValueError("slow_command_cap must be in (0, command_cap]")
        if not 0.0 < self.reverse_command_cap <= 1.0:
            raise ValueError("reverse_command_cap must be in (0, 1]")
        if not 0.0 < self.turn_command_cap <= 1.0:
            raise ValueError("turn_command_cap must be in (0, 1]")
        if not 0.0 < self.slow_turn_command_cap <= self.turn_command_cap:
            raise ValueError(
                "slow_turn_command_cap must be in (0, turn_command_cap]"
            )
        if not 0.0 <= self.turn_longitudinal_cap <= 1.0:
            raise ValueError("turn_longitudinal_cap must be in [0, 1]")
        if not math.isfinite(self.max_steering_assist) or not (
            0.0 <= self.max_steering_assist <= MAX_STEERING_ASSIST
        ):
            raise ValueError("max_steering_assist must be in [0, 0.15]")
        classify_raw_axes(
            0,
            0,
            neutral_deadzone=self.neutral_deadzone,
            forward_cone_half_angle_deg=self.forward_cone_half_angle_deg,
        )

    def apply(self, x_pos: int, y_pos: int, deadman: bool) -> tuple[int, int]:
        """Return joystick axes permitted by a fresh matching envelope."""

        if not self.enabled:
            return int(x_pos), int(y_pos)

        now = self._monotonic_clock()
        command = classify_raw_axes(
            int(x_pos) if deadman else 0,
            int(y_pos) if deadman else 0,
            neutral_deadzone=self.neutral_deadzone,
            forward_cone_half_angle_deg=self.forward_cone_half_angle_deg,
        )
        command_class = self._authorization_family(command.intent_class)
        if command_class != self._last_command_class:
            # Forward, correction-left, and correction-right share one arming
            # family. Each envelope still authorizes only the checked steering
            # interval, but normal correction changes do not restart arming.
            self._clear_count = 0
            self._last_counted_intent_sequence = None
            self._minimum_acceptable_sequence = self._sequence + 1
            self._last_command_class = command_class
        if now - self._last_send_monotonic >= self.heartbeat_period_s:
            self._send_intent(command, now)
        self._receive_envelopes(now)

        if command.intent_class == RELEASED or not deadman:
            self._stop_latched = False
            self._clear_count = 0
            self._reason = "operator_released"
            return 0, 0
        if command.intent_class not in (
            FORWARD_CLASSES + REVERSE_CLASSES + (LEFT_TURN, RIGHT_TURN)
        ):
            self._clear_count = 0
            self._reason = "%s_not_enabled" % intent_label(
                command.intent_class
            )
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
        if envelope.intent_sequence < self._minimum_acceptable_sequence:
            self._clear_count = 0
            self._reason = "envelope_precedes_current_input"
            return 0, 0
        matching_intent = self._intent_by_sequence.get(envelope.intent_sequence)
        if not self._intent_is_compatible(matching_intent, command):
            self._clear_count = 0
            self._reason = "envelope_does_not_match_current_intent"
            return 0, 0
        if envelope.decision == STOP:
            self._stop_latched = not (
                self.auto_resume_obstacle_stops
                and envelope.reason in RECOVERABLE_OBSTACLE_STOP_REASONS
            )
            self._clear_count = 0
            self._last_counted_intent_sequence = envelope.intent_sequence
            self._reason = envelope.reason
            return 0, 0
        if command.is_reverse and envelope.decision != SLOW:
            self._stop_latched = True
            self._clear_count = 0
            self._reason = "reverse_requires_slow_envelope"
            return 0, 0

        if envelope.intent_sequence != self._last_counted_intent_sequence:
            self._clear_count += 1
            self._last_counted_intent_sequence = envelope.intent_sequence
        if self._clear_count < self.required_clear_envelopes:
            self._reason = "waiting_for_clear_envelopes"
            return 0, 0

        if not self._envelope_limit_is_valid(envelope, matching_intent):
            self._stop_latched = True
            self._clear_count = 0
            self._reason = "invalid_safety_envelope_limit"
            return 0, 0

        is_turn = command.intent_class in (LEFT_TURN, RIGHT_TURN)
        if command.is_reverse:
            decision_cap = self.reverse_command_cap
        elif is_turn and envelope.decision == SLOW:
            decision_cap = self.slow_turn_command_cap
        elif is_turn:
            decision_cap = self.turn_command_cap
        elif envelope.decision == SLOW:
            decision_cap = self.slow_command_cap
        else:
            decision_cap = self.command_cap
        permitted_forward = min(
            envelope.permitted_forward,
            decision_cap,
            abs(command.longitudinal),
        )
        if is_turn:
            permitted_forward = min(
                permitted_forward,
                self.turn_longitudinal_cap,
            )
            permitted_lateral = self._steering_inside_authorized_interval(
                command.lateral,
                envelope.permitted_steering,
            )
            if permitted_lateral == 0.0:
                self._reason = "turn_direction_not_authorized"
                return 0, 0
            permitted_lateral = math.copysign(
                min(abs(permitted_lateral), decision_cap),
                permitted_lateral,
            )
            output_y = int(round(permitted_forward * 100.0))
            if command.longitudinal < 0.0:
                output_y = -output_y
            self._reason = envelope.reason
            return ros_steering_to_pi_x(permitted_lateral), output_y
        if self.max_steering_assist > 0.0:
            if not self._forward_assist_is_valid(
                envelope.permitted_steering, command
            ):
                self._reason = "assist_does_not_match_current_intent"
                return 0, 0
            permitted_steering = envelope.permitted_steering
        else:
            permitted_steering = self._steering_inside_authorized_interval(
                command.steering_ratio,
                envelope.permitted_steering,
            )
        self._reason = envelope.reason
        output_magnitude = int(round(permitted_forward * 100.0))
        output_y = (
            -output_magnitude if command.is_reverse else output_magnitude
        )
        return (
            steering_ratio_to_pi_x(permitted_steering, output_magnitude),
            output_y,
        )

    @staticmethod
    def _authorization_family(intent_class: int) -> str:
        if intent_class == RELEASED:
            return "released"
        if intent_class in FORWARD_CLASSES:
            return "forward_cone"
        if intent_class in REVERSE_CLASSES:
            return "reverse_cone"
        if intent_class in (LEFT_TURN, RIGHT_TURN):
            return "turn_disc"
        return "unsupported"

    @staticmethod
    def _steering_inside_authorized_interval(
        current_ratio: float,
        authorized_ratio: float,
    ) -> float:
        """Clamp to the checked straight-to-requested steering interval."""

        if current_ratio == 0.0 or authorized_ratio == 0.0:
            return 0.0
        if math.copysign(1.0, current_ratio) != math.copysign(
            1.0, authorized_ratio
        ):
            return 0.0
        return math.copysign(
            min(abs(current_ratio), abs(authorized_ratio)),
            current_ratio,
        )

    @staticmethod
    def _intent_is_compatible(
        sent_command: ClassifiedIntent | None,
        current_command: ClassifiedIntent,
    ) -> bool:
        """Allow a fresh envelope only within the active motion family."""

        if sent_command is None:
            return False
        return (
            sent_command.is_forward
            and current_command.is_forward
        ) or (
            sent_command.is_reverse
            and current_command.is_reverse
        ) or (
            sent_command.intent_class in (LEFT_TURN, RIGHT_TURN)
            and sent_command.intent_class == current_command.intent_class
        )

    def _envelope_limit_is_valid(
        self,
        envelope: Envelope,
        sent_command: ClassifiedIntent,
    ) -> bool:
        if envelope.permitted_forward > abs(sent_command.longitudinal) + 1e-6:
            return False
        if sent_command.intent_class in (LEFT_TURN, RIGHT_TURN):
            permitted = envelope.permitted_steering
            requested = sent_command.lateral
            if permitted == 0.0:
                return True
            if requested == 0.0:
                return False
            return (
                math.copysign(1.0, permitted)
                == math.copysign(1.0, requested)
                and abs(permitted) <= abs(requested) + 1e-6
            )
        requested = sent_command.steering_ratio
        permitted = envelope.permitted_steering
        if sent_command.is_forward and self.max_steering_assist > 0.0:
            return self._forward_assist_is_valid(permitted, sent_command)
        if permitted == 0.0:
            return True
        if requested == 0.0:
            return False
        if math.copysign(1.0, permitted) != math.copysign(1.0, requested):
            return False
        return abs(permitted) <= abs(requested) + 1e-6

    def _forward_assist_is_valid(
        self,
        permitted: float,
        command: ClassifiedIntent,
    ) -> bool:
        """Validate planner steering against authority advertised by this Pi."""

        if not command.is_forward or not math.isfinite(permitted):
            return False
        requested = command.steering_ratio
        authority = self.max_steering_assist
        if command.intent_class == FORWARD_CLASSES[0]:
            return abs(permitted) <= authority + 1e-6
        if requested > 0.0:
            return (
                -1e-6 <= permitted <= requested + 1e-6
                and requested - permitted <= authority + 1e-6
            )
        if requested < 0.0:
            return (
                requested - 1e-6 <= permitted <= 1e-6
                and permitted - requested <= authority + 1e-6
            )
        return abs(permitted) <= authority + 1e-6

    def _send_intent(self, command: ClassifiedIntent, now: float) -> None:
        self._sequence += 1
        payload = {
            "v": PROTOCOL_VERSION,
            "type": "intent",
            "session": self.session_id,
            "seq": self._sequence,
            "lateral": command.lateral,
            "longitudinal": command.longitudinal,
            "max_steering_assist": self.max_steering_assist,
            "intent_class": command.intent_class,
            "deadman": command.deadman,
        }
        data = json.dumps(
            payload, separators=(",", ":"), sort_keys=True
        ).encode("utf-8")
        self._socket.sendto(data, (self.jetson_address, self.intent_port))
        self._last_send_monotonic = now
        self._intent_by_sequence[self._sequence] = command
        self._sent_monotonic_by_sequence[self._sequence] = now
        for sequence in list(self._intent_by_sequence):
            if sequence < self._sequence - 32:
                del self._intent_by_sequence[sequence]
                self._sent_monotonic_by_sequence.pop(sequence, None)

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
            self._latest_decision = envelope.decision
            self._latest_map_age_ms = envelope.map_age_ms
            sent_at = self._sent_monotonic_by_sequence.get(
                envelope.intent_sequence
            )
            self._latest_round_trip_ms = (
                None if sent_at is None else max(0.0, (now - sent_at) * 1000.0)
            )

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
        envelope_age_ms = None
        if self._last_envelope is not None:
            envelope_age_ms = max(
                0.0,
                (self._monotonic_clock() - self._last_envelope_monotonic)
                * 1000.0,
            )
        return {
            "enabled": self.enabled,
            "session_id": self.session_id,
            "reason": self._reason,
            "stop_latched": self._stop_latched,
            "clear_count": self._clear_count,
            "latest_decision": self._latest_decision,
            "map_age_ms": self._latest_map_age_ms,
            "round_trip_ms": self._latest_round_trip_ms,
            "envelope_age_ms": envelope_age_ms,
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
    "MAX_STEERING_ASSIST",
    "ProtocolError",
    "RECOVERABLE_OBSTACLE_STOP_REASONS",
    "SLOW",
    "STOP",
    "SafetyLink",
    "decode_envelope",
    "pi_x_to_ros_steering",
    "ros_steering_to_pi_x",
    "steering_ratio_to_pi_x",
]
