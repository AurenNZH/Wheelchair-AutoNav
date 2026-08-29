"""Pure fail-closed arbitration helpers for local avoidance suggestions."""

from __future__ import annotations

import math

from wheelchair_shared_control.operator_intent import (
    FORWARD,
    FORWARD_CLASSES,
    FORWARD_LEFT,
)


def suggestion_compatibility_reason(
    *,
    current_session: str,
    current_sequence: int,
    current_intent_class: int,
    current_steering: float,
    suggestion_session: str,
    suggestion_sequence: int,
    suggestion_intent_class: int,
    source_steering: float,
    suggested_steering: float,
    valid: bool,
    source_tolerance: float,
) -> str | None:
    """Return None only when a suggestion matches the current forward intent."""

    if not valid:
        return "invalid_suggestion"
    if (
        suggestion_session != current_session
        or suggestion_sequence > current_sequence
        or suggestion_intent_class not in FORWARD_CLASSES
        or current_intent_class not in FORWARD_CLASSES
    ):
        return "suggestion_intent_mismatch"
    values = (current_steering, source_steering, suggested_steering)
    if not all(math.isfinite(value) for value in values):
        return "invalid_suggestion"
    if abs(source_steering - current_steering) > source_tolerance:
        return "source_steering_mismatch"
    return None


def suggestion_age_is_fresh(age_s: float, maximum_age_s: float) -> bool:
    """Validate monotonic suggestion age without accepting future samples."""

    return (
        math.isfinite(age_s)
        and math.isfinite(maximum_age_s)
        and maximum_age_s > 0.0
        and 0.0 <= age_s <= maximum_age_s
    )


def bound_current_assist(
    *,
    requested: float,
    intent_class: int,
    planned: float,
    authority: float,
    system_maximum: float,
    minimum_steering: float,
    maximum_steering: float,
) -> float | None:
    """Apply the current intent's authority independently of the planner."""

    values = (
        requested,
        planned,
        authority,
        system_maximum,
        minimum_steering,
        maximum_steering,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    if intent_class not in FORWARD_CLASSES:
        return None
    maximum = min(system_maximum, abs(minimum_steering), maximum_steering)
    authority = min(max(0.0, authority), maximum)
    if authority <= 0.0:
        return None
    if intent_class == FORWARD:
        bounded = max(-authority, min(authority, planned))
    elif intent_class == FORWARD_LEFT:
        bounded = max(
            max(0.0, requested - authority), min(requested, planned)
        )
    else:
        bounded = min(
            min(0.0, requested + authority), max(requested, planned)
        )
    return max(minimum_steering, min(maximum_steering, bounded))


def candidate_is_no_worse(direct_decision: int, candidate_decision: int) -> bool:
    """Never apply a STOP suggestion or one less permissive than direct."""

    return candidate_decision != 0 and candidate_decision >= direct_decision


__all__ = [
    "bound_current_assist",
    "candidate_is_no_worse",
    "suggestion_compatibility_reason",
    "suggestion_age_is_fresh",
]
