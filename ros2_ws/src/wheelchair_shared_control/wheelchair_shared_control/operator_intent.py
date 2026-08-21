"""Pure normalized operator-intent classification for shared control."""

from __future__ import annotations

from dataclasses import dataclass
import math


RELEASED = 0
FORWARD = 1
FORWARD_LEFT = 2
FORWARD_RIGHT = 3
LEFT_TURN = 4
RIGHT_TURN = 5
REVERSE = 6
REVERSE_LEFT = 7
REVERSE_RIGHT = 8

INTENT_LABELS = {
    RELEASED: "released",
    FORWARD: "forward",
    FORWARD_LEFT: "forward_left",
    FORWARD_RIGHT: "forward_right",
    LEFT_TURN: "left_turn",
    RIGHT_TURN: "right_turn",
    REVERSE: "reverse",
    REVERSE_LEFT: "reverse_left",
    REVERSE_RIGHT: "reverse_right",
}
INTENT_CLASSES = tuple(INTENT_LABELS)
FORWARD_CLASSES = (FORWARD, FORWARD_LEFT, FORWARD_RIGHT)
REVERSE_CLASSES = (REVERSE, REVERSE_LEFT, REVERSE_RIGHT)


@dataclass(frozen=True)
class ClassifiedIntent:
    intent_class: int
    lateral: float
    longitudinal: float
    deadman: bool
    heading_deg: float | None

    @property
    def label(self) -> str:
        return INTENT_LABELS[self.intent_class]

    @property
    def is_forward(self) -> bool:
        return self.intent_class in FORWARD_CLASSES

    @property
    def is_reverse(self) -> bool:
        return self.intent_class in REVERSE_CLASSES

    @property
    def steering_ratio(self) -> float:
        moving = self.is_forward or self.is_reverse
        if not moving or self.longitudinal == 0.0:
            return 0.0
        return self.lateral / abs(self.longitudinal)


def classify_normalized_axes(
    lateral: float,
    longitudinal: float,
    *,
    neutral_deadzone: float = 0.05,
    forward_cone_half_angle_deg: float = 30.0,
) -> ClassifiedIntent:
    lateral = float(lateral)
    longitudinal = float(longitudinal)
    cone = float(forward_cone_half_angle_deg)
    deadzone = float(neutral_deadzone)
    if not math.isfinite(lateral) or not math.isfinite(longitudinal):
        raise ValueError("intent axes must be finite")
    if not -1.0 <= lateral <= 1.0 or not -1.0 <= longitudinal <= 1.0:
        raise ValueError("intent axes must be in [-1, 1]")
    if not math.isfinite(deadzone) or not 0.0 <= deadzone < 1.0:
        raise ValueError("neutral_deadzone must be in [0, 1)")
    if not math.isfinite(cone) or not 0.0 < cone < 90.0:
        raise ValueError("forward cone half-angle must be in (0, 90)")

    if abs(lateral) <= deadzone and abs(longitudinal) <= deadzone:
        return ClassifiedIntent(RELEASED, lateral, longitudinal, False, None)

    heading_deg = math.degrees(math.atan2(lateral, longitudinal))
    if longitudinal > 0.0 and abs(heading_deg) <= cone:
        if abs(lateral) <= deadzone:
            intent_class = FORWARD
        elif lateral > 0.0:
            intent_class = FORWARD_LEFT
        else:
            intent_class = FORWARD_RIGHT
    elif abs(heading_deg) >= 180.0 - cone:
        if abs(lateral) <= deadzone:
            intent_class = REVERSE
        elif lateral > 0.0:
            intent_class = REVERSE_LEFT
        else:
            intent_class = REVERSE_RIGHT
    elif lateral > 0.0:
        intent_class = LEFT_TURN
    elif lateral < 0.0:
        intent_class = RIGHT_TURN
    else:
        intent_class = REVERSE

    return ClassifiedIntent(
        intent_class,
        lateral,
        longitudinal,
        True,
        heading_deg,
    )


def intent_label(intent_class: int) -> str:
    try:
        return INTENT_LABELS[int(intent_class)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("unknown intent class") from exc


__all__ = [
    "ClassifiedIntent",
    "FORWARD",
    "FORWARD_CLASSES",
    "FORWARD_LEFT",
    "FORWARD_RIGHT",
    "INTENT_CLASSES",
    "INTENT_LABELS",
    "LEFT_TURN",
    "RELEASED",
    "REVERSE",
    "REVERSE_CLASSES",
    "REVERSE_LEFT",
    "REVERSE_RIGHT",
    "RIGHT_TURN",
    "classify_normalized_axes",
    "intent_label",
]
