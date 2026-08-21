"""Motion-cone shared control for a physical in-line R-Net JSM."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .jsm_observer import JsmSample
from .operator_intent import (
    RELEASED,
    classify_raw_axes,
    intent_label,
)
from .safety_link import STOP, SafetyLink


@dataclass(frozen=True)
class PhysicalControlResult:
    """One physical request, its safe limit, and the frame actually forwarded."""

    input_x: int
    input_y: int
    intent_class: int
    intent_label: str
    heading_deg: Optional[float]
    would_output_x: int
    would_output_y: int
    forwarded_x: int
    forwarded_y: int
    reason: str
    supervisor_decision: Optional[int]
    map_age_ms: Optional[float]
    round_trip_ms: Optional[float]
    envelope_age_ms: Optional[float]
    local_stop_latched: bool


class PhysicalJsmSharedControl:
    """Convert physical JSM samples into shadowed or enforced safe commands."""

    def __init__(
        self,
        safety_link: SafetyLink,
        *,
        mode: str = "shadow",
        neutral_deadzone: int = 5,
        forward_cone_half_angle_deg: float = 30.0,
    ) -> None:
        if mode not in ("shadow", "enforce"):
            raise ValueError("mode must be 'shadow' or 'enforce'")
        if isinstance(neutral_deadzone, bool) or not isinstance(
            neutral_deadzone, int
        ):
            raise ValueError("neutral_deadzone must be an integer")
        if not 0 <= neutral_deadzone <= 99:
            raise ValueError("neutral_deadzone must be in [0, 99]")
        if not safety_link.enabled:
            raise ValueError("physical shared control requires an enabled safety link")

        self.safety_link = safety_link
        self.mode = mode
        self.neutral_deadzone = neutral_deadzone
        self.forward_cone_half_angle_deg = float(
            forward_cone_half_angle_deg
        )
        classify_raw_axes(
            0,
            0,
            neutral_deadzone=self.neutral_deadzone,
            forward_cone_half_angle_deg=self.forward_cone_half_angle_deg,
        )
        self._local_stop_latched = False
        self.last_result = None

    def transform(self, sample: JsmSample) -> tuple[int, int]:
        """Return axes to forward for one physical slot-specific JSM frame."""

        input_x = int(sample.x_raw)
        input_y = int(sample.y_raw)
        intent = classify_raw_axes(
            input_x,
            input_y,
            neutral_deadzone=self.neutral_deadzone,
            forward_cone_half_angle_deg=self.forward_cone_half_angle_deg,
        )
        local_reason = None

        try:
            if intent.intent_class == RELEASED:
                self._local_stop_latched = False
                self.safety_link.apply(0, 0, False)
                # Neutral is an unconditional local stop even if a malformed
                # or stale safety-link implementation returned motion.
                would_x, would_y = 0, 0
            elif not (intent.is_forward or intent.is_reverse):
                self._local_stop_latched = True
                self.safety_link.apply(input_x, input_y, True)
                would_x, would_y = 0, 0
                local_reason = "%s_not_enabled" % intent_label(
                    intent.intent_class
                )
            else:
                would_x, would_y = self.safety_link.apply(
                    input_x, input_y, True
                )
                if self._local_stop_latched:
                    would_x = 0
                    would_y = 0
                    local_reason = "local_stop_latched"
        except Exception as exc:
            would_x, would_y = 0, 0
            local_reason = "safety_link_error:%s" % exc

        try:
            status = self.safety_link.get_status()
        except Exception as exc:
            would_x, would_y = 0, 0
            local_reason = "safety_status_error:%s" % exc
            status = {
                "reason": local_reason,
                "latest_decision": STOP,
                "map_age_ms": None,
                "round_trip_ms": None,
                "envelope_age_ms": None,
            }
        reason = local_reason or str(status["reason"])
        decision = status.get("latest_decision")
        if local_reason is not None:
            decision = STOP

        if self.mode == "shadow":
            forwarded_x, forwarded_y = input_x, input_y
        else:
            forwarded_x, forwarded_y = would_x, would_y

        self.last_result = PhysicalControlResult(
            input_x=input_x,
            input_y=input_y,
            intent_class=intent.intent_class,
            intent_label=intent.label,
            heading_deg=intent.heading_deg,
            would_output_x=int(would_x),
            would_output_y=int(would_y),
            forwarded_x=int(forwarded_x),
            forwarded_y=int(forwarded_y),
            reason=reason,
            supervisor_decision=decision,
            map_age_ms=status.get("map_age_ms"),
            round_trip_ms=status.get("round_trip_ms"),
            envelope_age_ms=status.get("envelope_age_ms"),
            local_stop_latched=self._local_stop_latched,
        )
        return int(forwarded_x), int(forwarded_y)


# Keep the previous import name for local callers while the v2 implementation
# and operator documentation migrate together.
StraightPhysicalJsmControl = PhysicalJsmSharedControl


__all__ = [
    "PhysicalControlResult",
    "PhysicalJsmSharedControl",
    "StraightPhysicalJsmControl",
]
