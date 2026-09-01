"""Trajectory geometry and weighted-cost sampling for shared control."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from wheelchair_shared_control.models import (
    SafetyConfig,
    WeightedCostmap,
    validate_safety_config,
)


@dataclass(frozen=True)
class PathCostSummary:
    """Cost evidence sampled across the requested trajectory union."""

    maximum_cost: int | None = None
    nearest_slow_distance_m: float | None = None
    nearest_stop_distance_m: float | None = None
    accumulated_cost: int = 0
    valid: bool = False
    failure_reason: str | None = None


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
        summary = individual_path_costs(
            costmap, float(candidate), config
        )
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


def turn_disc_costs(
    costmap: WeightedCostmap,
    config: SafetyConfig,
) -> PathCostSummary:
    """Sample cells whose centres lie in the base-centred turn disc."""

    validate_safety_config(config)
    radius = float(config.turn_clearance_radius_m)
    map_min_x = float(costmap.origin_x_m)
    map_min_y = float(costmap.origin_y_m)
    map_max_x = map_min_x + costmap.width * costmap.resolution_m
    map_max_y = map_min_y + costmap.height * costmap.resolution_m
    tolerance = 1e-9
    if (
        map_min_x > -radius + tolerance
        or map_max_x < radius - tolerance
        or map_min_y > -radius + tolerance
        or map_max_y < radius - tolerance
    ):
        return PathCostSummary(
            valid=False,
            failure_reason="turn_disc_outside_costmap",
        )

    xs = costmap.origin_x_m + (
        np.arange(costmap.width, dtype=np.float64) + 0.5
    ) * costmap.resolution_m
    ys = costmap.origin_y_m + (
        np.arange(costmap.height, dtype=np.float64) + 0.5
    ) * costmap.resolution_m
    squared_distances = ys[:, None] ** 2 + xs[None, :] ** 2
    checked = squared_distances <= radius ** 2 + tolerance
    checked_costs = costmap.costs[checked]
    if checked_costs.size == 0:
        return PathCostSummary(
            valid=False,
            failure_reason="empty_turn_disc",
        )
    if np.any(checked_costs < 0):
        return PathCostSummary(
            maximum_cost=int(np.max(checked_costs)),
            valid=False,
            failure_reason="unknown_nav2_turn_cost",
        )

    checked_distances = np.sqrt(squared_distances[checked])
    slow_mask = checked_costs >= config.slow_cost_threshold
    stop_mask = checked_costs >= config.stop_cost_threshold
    return PathCostSummary(
        maximum_cost=int(np.max(checked_costs)),
        nearest_slow_distance_m=(
            float(np.min(checked_distances[slow_mask]))
            if np.any(slow_mask)
            else None
        ),
        nearest_stop_distance_m=(
            float(np.min(checked_distances[stop_mask]))
            if np.any(stop_mask)
            else None
        ),
        valid=True,
    )


def _costs_for_steering(
    costmap: WeightedCostmap,
    steering: float,
    config: SafetyConfig,
) -> PathCostSummary:
    """Backward-compatible private alias for one trajectory's costs."""

    return individual_path_costs(costmap, steering, config)


def individual_path_costs(
    costmap: WeightedCostmap,
    steering: float,
    config: SafetyConfig,
    *,
    horizon_m: float | None = None,
    sample_step_m: float | None = None,
) -> PathCostSummary:
    """Sample one arc without the direct policy's swept steering union."""

    validate_safety_config(config)
    horizon = (
        float(config.slow_distance_m)
        if horizon_m is None
        else float(horizon_m)
    )
    step = (
        float(config.path_sample_step_m)
        if sample_step_m is None
        else float(sample_step_m)
    )
    if (
        not math.isfinite(horizon)
        or horizon <= 0.0
        or not math.isfinite(step)
        or step <= 0.0
        or not math.isfinite(float(steering))
    ):
        raise ValueError("trajectory horizon, step, and steering must be valid")
    samples = max(1, int(math.ceil(horizon / step)))
    distances = np.linspace(0.0, horizon, samples + 1)
    points = _trajectory_points_at_distances(
        float(steering), distances, config
    )

    maximum_cost = None
    nearest_slow = None
    nearest_stop = None
    accumulated_cost = 0
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
                accumulated_cost=accumulated_cost,
                valid=False,
                failure_reason="trajectory_outside_costmap",
            )
        cost = int(costmap.costs[row, col])
        if cost < 0:
            return PathCostSummary(
                maximum_cost=maximum_cost,
                nearest_slow_distance_m=nearest_slow,
                nearest_stop_distance_m=nearest_stop,
                accumulated_cost=accumulated_cost,
                valid=False,
                failure_reason="unknown_nav2_cost",
            )
        accumulated_cost += cost
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
        accumulated_cost=accumulated_cost,
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
    return _trajectory_points_at_distances(steering, distances, config)


def trajectory_points_for_horizon(
    steering: float,
    config: SafetyConfig,
    *,
    horizon_m: float,
    sample_step_m: float,
) -> tuple[tuple[float, float], ...]:
    """Return one arc with caller-selected reactive horizon and spacing."""

    horizon = float(horizon_m)
    step = float(sample_step_m)
    if not math.isfinite(horizon) or horizon <= 0.0:
        raise ValueError("trajectory horizon must be finite and positive")
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("trajectory sample step must be finite and positive")
    samples = max(1, int(math.ceil(horizon / step)))
    distances = np.linspace(0.0, horizon, samples + 1)
    return _trajectory_points_at_distances(steering, distances, config)


def _trajectory_points_at_distances(
    steering: float,
    distances: np.ndarray,
    config: SafetyConfig,
) -> tuple[tuple[float, float], ...]:
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


__all__ = [
    "PathCostSummary",
    "individual_path_costs",
    "swept_path_costs",
    "trajectory_points",
    "trajectory_points_for_horizon",
    "turn_disc_costs",
]
