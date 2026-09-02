"""Pure data models and validation for shared-control decisions."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


STOP = 0
SLOW = 1
CLEAR = 2


@dataclass(frozen=True)
class OperatorIntentData:
    session_id: str
    sequence: int
    lateral: float
    longitudinal: float
    intent_class: int
    deadman: bool


@dataclass(frozen=True)
class SafetyConfig:
    enable_motion: bool = False
    geometry_calibrated: bool = False
    stop_distance_m: float = 0.70
    slow_distance_m: float = 1.20
    min_turn_radius_m: float = 1.20
    min_steering: float = -0.577350269
    max_steering: float = 0.577350269
    slow_forward_limit: float = 0.60
    reverse_limit: float = 0.65
    turn_clearance_radius_m: float = 0.45
    clear_turn_limit: float = 0.90
    slow_turn_limit: float = 0.60
    turn_longitudinal_limit: float = 0.15
    path_sample_step_m: float = 0.05
    steering_sample_step: float = 0.05
    neutral_deadzone: float = 0.05
    forward_cone_half_angle_deg: float = 30.0
    slow_cost_threshold: int = 1
    stop_cost_threshold: int = 99


@dataclass(frozen=True)
class WeightedCostmap:
    """Validated robot-relative Nav2 costs with an axis-aligned origin."""

    costs: np.ndarray
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float


@dataclass(frozen=True)
class SafetyDecision:
    decision: int
    permitted_forward: float
    permitted_steering: float
    reason: str
    nearest_path_distance_m: float | None = None
    maximum_path_cost: int | None = None
    nearest_slow_cost_distance_m: float | None = None
    nearest_stop_cost_distance_m: float | None = None
    path_cost_valid: bool = False


def weighted_costmap_from_grid(
    data,
    *,
    frame_id: str,
    width: int,
    height: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    origin_orientation_xyzw: tuple[float, float, float, float],
) -> WeightedCostmap:
    """Validate and retain every Nav2 OccupancyGrid cost."""

    if frame_id != "base_link":
        raise ValueError("costmap frame must be base_link")
    if width <= 0 or height <= 0:
        raise ValueError("costmap dimensions must be positive")
    if not math.isfinite(resolution_m) or resolution_m <= 0.0:
        raise ValueError("costmap resolution must be finite and positive")
    if not math.isfinite(origin_x_m) or not math.isfinite(origin_y_m):
        raise ValueError("costmap origin must be finite")
    orientation = tuple(float(value) for value in origin_orientation_xyzw)
    if len(orientation) != 4 or not all(map(math.isfinite, orientation)):
        raise ValueError("costmap origin orientation must be finite")
    x, y, z, w = orientation
    if (
        abs(x) > 1e-6
        or abs(y) > 1e-6
        or abs(z) > 1e-6
        or abs(abs(w) - 1.0) > 1e-6
    ):
        raise ValueError("costmap origin orientation must be identity")

    values = np.asarray(data, dtype=np.int16)
    if values.size != width * height:
        raise ValueError("costmap dimensions do not match its data")
    if np.any(values < -1) or np.any(values > 100):
        raise ValueError("costmap values must be within [-1, 100]")
    costs = values.reshape(height, width).copy()
    costs.setflags(write=False)
    return WeightedCostmap(
        costs=costs,
        width=width,
        height=height,
        resolution_m=float(resolution_m),
        origin_x_m=float(origin_x_m),
        origin_y_m=float(origin_y_m),
    )


def validate_safety_config(config: SafetyConfig) -> None:
    """Validate trajectory geometry, motion limits, and cost thresholds."""

    finite_positive = (
        config.path_sample_step_m,
        config.steering_sample_step,
        config.min_turn_radius_m,
        config.stop_distance_m,
        config.slow_distance_m,
        config.turn_clearance_radius_m,
    )
    if not all(
        math.isfinite(value) and value > 0.0 for value in finite_positive
    ):
        raise ValueError("cost-policy geometry must be finite and positive")
    if config.stop_distance_m > config.slow_distance_m:
        raise ValueError("stop distance must not exceed slow distance")
    if not (
        math.isfinite(config.slow_forward_limit)
        and 0.0 < config.slow_forward_limit <= 1.0
    ):
        raise ValueError("slow forward limit must be in (0, 1]")
    if not (
        math.isfinite(config.reverse_limit)
        and 0.0 < config.reverse_limit <= 1.0
    ):
        raise ValueError("reverse limit must be in (0, 1]")
    if not (
        math.isfinite(config.clear_turn_limit)
        and 0.0 < config.clear_turn_limit <= 1.0
    ):
        raise ValueError("clear turn limit must be in (0, 1]")
    if not (
        math.isfinite(config.slow_turn_limit)
        and 0.0 < config.slow_turn_limit <= config.clear_turn_limit
    ):
        raise ValueError("slow turn limit must be in (0, clear turn limit]")
    if not (
        math.isfinite(config.turn_longitudinal_limit)
        and 0.0 <= config.turn_longitudinal_limit <= 1.0
    ):
        raise ValueError("turn longitudinal limit must be in [0, 1]")
    if not (
        1
        <= config.slow_cost_threshold
        < config.stop_cost_threshold
        <= 100
    ):
        raise ValueError(
            "cost thresholds must satisfy 1 <= slow < stop <= 100"
        )


__all__ = [
    "CLEAR",
    "SLOW",
    "STOP",
    "OperatorIntentData",
    "SafetyConfig",
    "SafetyDecision",
    "WeightedCostmap",
    "validate_safety_config",
    "weighted_costmap_from_grid",
]
