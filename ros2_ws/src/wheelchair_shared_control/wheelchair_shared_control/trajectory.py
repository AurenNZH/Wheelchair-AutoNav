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


__all__ = ["PathCostSummary", "swept_path_costs", "trajectory_points"]
