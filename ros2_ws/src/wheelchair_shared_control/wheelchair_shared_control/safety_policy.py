"""Pure STOP, SLOW, and CLEAR shared-control policy."""

from __future__ import annotations

import math

from wheelchair_shared_control.operator_intent import (
    FORWARD_CLASSES,
    LEFT_TURN,
    RELEASED,
    REVERSE_CLASSES,
    RIGHT_TURN,
    classify_normalized_axes,
)
from wheelchair_shared_control.models import (
    CLEAR,
    SLOW,
    STOP,
    OperatorIntentData,
    SafetyConfig,
    SafetyDecision,
    WeightedCostmap,
)
from wheelchair_shared_control.trajectory import (
    PathCostSummary,
    swept_path_costs,
)


def evaluate_safety(
    intent: OperatorIntentData,
    front_costmap: WeightedCostmap,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    """Limit forward motion by Nav2 costs and cap unmonitored reverse."""

    if not config.enable_motion:
        return stop_decision("live_control_disabled")
    if not config.geometry_calibrated:
        return stop_decision("uncalibrated_geometry")
    if not math.isfinite(intent.lateral) or not math.isfinite(
        intent.longitudinal
    ):
        return stop_decision("invalid_intent")
    if abs(intent.lateral) > 1.0 or abs(intent.longitudinal) > 1.0:
        return stop_decision("invalid_intent")
    try:
        classified = classify_normalized_axes(
            intent.lateral,
            intent.longitudinal,
            neutral_deadzone=config.neutral_deadzone,
            forward_cone_half_angle_deg=config.forward_cone_half_angle_deg,
        )
    except ValueError:
        return stop_decision("invalid_intent")
    if (
        int(intent.intent_class) != classified.intent_class
        or bool(intent.deadman) != classified.deadman
    ):
        return stop_decision("intent_class_mismatch")
    if not intent.deadman or intent.intent_class == RELEASED:
        return stop_decision("deadman_released")
    if intent.intent_class == LEFT_TURN:
        return stop_decision("left_turn_not_enabled")
    if intent.intent_class == RIGHT_TURN:
        return stop_decision("right_turn_not_enabled")
    if intent.intent_class not in FORWARD_CLASSES + REVERSE_CLASSES:
        return stop_decision("unsupported_intent")

    steering = classified.steering_ratio
    if steering > config.max_steering:
        return stop_decision("left_correction_limit_exceeded")
    if steering < config.min_steering:
        return stop_decision("right_correction_limit_exceeded")

    if intent.intent_class in REVERSE_CLASSES:
        return SafetyDecision(
            SLOW,
            min(abs(float(intent.longitudinal)), config.reverse_limit),
            steering,
            "reverse_unmonitored_slow",
        )

    summary = swept_path_costs(front_costmap, steering, config)
    if not summary.valid:
        return _stop_from_costs(
            summary.failure_reason or "invalid_costmap", summary
        )
    if (
        summary.nearest_stop_distance_m is not None
        and summary.nearest_stop_distance_m <= config.stop_distance_m
    ):
        return _stop_from_costs("nav2_cost_stop", summary)
    if (
        summary.nearest_slow_distance_m is not None
        and summary.nearest_slow_distance_m <= config.slow_distance_m
    ):
        return SafetyDecision(
            SLOW,
            min(float(intent.longitudinal), config.slow_forward_limit),
            steering,
            "nav2_cost_slow",
            summary.nearest_slow_distance_m,
            summary.maximum_cost,
            summary.nearest_slow_distance_m,
            summary.nearest_stop_distance_m,
            True,
        )
    return SafetyDecision(
        CLEAR,
        float(intent.longitudinal),
        steering,
        "nav2_cost_clear",
        summary.nearest_slow_distance_m,
        summary.maximum_cost,
        summary.nearest_slow_distance_m,
        summary.nearest_stop_distance_m,
        True,
    )


def stop_decision(
    reason: str,
    nearest: float | None = None,
) -> SafetyDecision:
    """Return a fail-closed decision with zero permitted motion."""

    return SafetyDecision(STOP, 0.0, 0.0, reason, nearest)


def _stop_from_costs(
    reason: str, summary: PathCostSummary
) -> SafetyDecision:
    nearest = (
        summary.nearest_stop_distance_m
        if reason == "nav2_cost_stop"
        else summary.nearest_slow_distance_m
    )
    return SafetyDecision(
        STOP,
        0.0,
        0.0,
        reason,
        nearest,
        summary.maximum_cost,
        summary.nearest_slow_distance_m,
        summary.nearest_stop_distance_m,
        summary.valid,
    )


__all__ = [
    "evaluate_safety",
    "stop_decision",
]
