"""Pure shared-control geometry and fail-safe policy."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

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
    validate_safety_config,
    weighted_costmap_from_grid,
)


@dataclass(frozen=True)
class PathCostSummary:
    """Cost evidence sampled across the requested trajectory union."""

    maximum_cost: int | None = None
    nearest_slow_distance_m: float | None = None
    nearest_stop_distance_m: float | None = None
    valid: bool = False
    failure_reason: str | None = None


def evaluate_safety(
    intent: OperatorIntentData,
    front_costmap: WeightedCostmap,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    """Limit forward motion by Nav2 costs and cap unmonitored reverse."""

    if not config.enable_motion:
        return _stop("live_control_disabled")
    if not config.geometry_calibrated:
        return _stop("uncalibrated_geometry")
    if not math.isfinite(intent.lateral) or not math.isfinite(
        intent.longitudinal
    ):
        return _stop("invalid_intent")
    if abs(intent.lateral) > 1.0 or abs(intent.longitudinal) > 1.0:
        return _stop("invalid_intent")
    try:
        classified = classify_normalized_axes(
            intent.lateral,
            intent.longitudinal,
            neutral_deadzone=config.neutral_deadzone,
            forward_cone_half_angle_deg=config.forward_cone_half_angle_deg,
        )
    except ValueError:
        return _stop("invalid_intent")
    if (
        int(intent.intent_class) != classified.intent_class
        or bool(intent.deadman) != classified.deadman
    ):
        return _stop("intent_class_mismatch")
    if not intent.deadman or intent.intent_class == RELEASED:
        return _stop("deadman_released")
    if intent.intent_class == LEFT_TURN:
        return _stop("left_turn_not_enabled")
    if intent.intent_class == RIGHT_TURN:
        return _stop("right_turn_not_enabled")
    if intent.intent_class not in FORWARD_CLASSES + REVERSE_CLASSES:
        return _stop("unsupported_intent")

    steering = classified.steering_ratio
    if steering > config.max_steering:
        return _stop("left_correction_limit_exceeded")
    if steering < config.min_steering:
        return _stop("right_correction_limit_exceeded")

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


def swept_path_costs(
    costmap: WeightedCostmap,
    steering: float,
    config: SafetyConfig,
) -> PathCostSummary:
    """Sample Nav2 costs from straight through the requested correction."""

    validate_safety_config(config)
    steering_step = float(config.steering_sample_step)
    count = max(1, int(math.ceil(abs(float(steering)) / steering_step)))
    candidates = np.linspace(0.0, float(steering), count + 1)
    if abs(float(steering)) < 1e-9:
        candidates = np.array([0.0])

    maximum_cost = None
    nearest_slow = None
    nearest_stop = None
    for candidate in candidates:
        summary = _costs_for_steering(costmap, float(candidate), config)
        if not summary.valid:
            return summary
        maximum_cost = _maximum(maximum_cost, summary.maximum_cost)
        nearest_slow = _nearest(
            nearest_slow, summary.nearest_slow_distance_m
        )
        nearest_stop = _nearest(
            nearest_stop, summary.nearest_stop_distance_m
        )
    return PathCostSummary(
        maximum_cost=maximum_cost,
        nearest_slow_distance_m=nearest_slow,
        nearest_stop_distance_m=nearest_stop,
        valid=True,
    )


def _costs_for_steering(
    costmap: WeightedCostmap,
    steering: float,
    config: SafetyConfig,
) -> PathCostSummary:
    points = trajectory_points(steering, config)
    step = float(config.path_sample_step_m)
    samples = max(1, int(math.ceil(config.slow_distance_m / step)))
    distances = np.linspace(0.0, config.slow_distance_m, samples + 1)

    maximum_cost = None
    nearest_slow = None
    nearest_stop = None
    for distance, (x_m, y_m) in zip(distances, points):
        col = math.floor(
            (float(x_m) - costmap.origin_x_m) / costmap.resolution_m
        )
        row = math.floor(
            (float(y_m) - costmap.origin_y_m) / costmap.resolution_m
        )
        if (
            col < 0
            or col >= costmap.width
            or row < 0
            or row >= costmap.height
        ):
            return PathCostSummary(
                maximum_cost=maximum_cost,
                nearest_slow_distance_m=nearest_slow,
                nearest_stop_distance_m=nearest_stop,
                valid=False,
                failure_reason="trajectory_outside_costmap",
            )
        cost = int(costmap.costs[row, col])
        if cost < 0:
            return PathCostSummary(
                maximum_cost=maximum_cost,
                nearest_slow_distance_m=nearest_slow,
                nearest_stop_distance_m=nearest_stop,
                valid=False,
                failure_reason="unknown_nav2_cost",
            )
        maximum_cost = (
            cost if maximum_cost is None else max(maximum_cost, cost)
        )
        if nearest_slow is None and cost >= config.slow_cost_threshold:
            nearest_slow = float(distance)
        if nearest_stop is None and cost >= config.stop_cost_threshold:
            nearest_stop = float(distance)

    return PathCostSummary(
        maximum_cost=maximum_cost,
        nearest_slow_distance_m=nearest_slow,
        nearest_stop_distance_m=nearest_stop,
        valid=True,
    )


def trajectory_points(
    steering: float,
    config: SafetyConfig,
) -> tuple[tuple[float, float], ...]:
    """Return the exact robot-centre samples used for one path check."""

    step = float(config.path_sample_step_m)
    samples = max(1, int(math.ceil(config.slow_distance_m / step)))
    distances = np.linspace(0.0, config.slow_distance_m, samples + 1)
    curvature = float(steering) / config.min_turn_radius_m
    if abs(curvature) < 1e-6:
        xs = distances
        ys = np.zeros_like(distances)
    else:
        yaws = curvature * distances
        xs = np.sin(yaws) / curvature
        ys = (1.0 - np.cos(yaws)) / curvature
    return tuple(
        (float(x_m), float(y_m)) for x_m, y_m in zip(xs, ys)
    )


def _nearest(first: float | None, second: float | None) -> float | None:
    if first is None:
        return second
    if second is None:
        return first
    return min(first, second)


def _maximum(first: int | None, second: int | None) -> int | None:
    if first is None:
        return second
    if second is None:
        return first
    return max(first, second)


def _stop(reason: str, nearest: float | None = None) -> SafetyDecision:
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
    "CLEAR",
    "PathCostSummary",
    "SLOW",
    "STOP",
    "OperatorIntentData",
    "SafetyConfig",
    "SafetyDecision",
    "WeightedCostmap",
    "evaluate_safety",
    "swept_path_costs",
    "trajectory_points",
    "validate_safety_config",
    "weighted_costmap_from_grid",
]
