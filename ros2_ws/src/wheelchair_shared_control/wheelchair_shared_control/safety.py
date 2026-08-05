"""Pure shared-control geometry and fail-safe policy."""

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
    steering: float
    forward: float
    deadman: bool


@dataclass(frozen=True)
class SafetyConfig:
    enable_motion: bool = False
    geometry_calibrated: bool = False
    chair_width_m: float = 0.70
    front_extent_m: float = 0.80
    rear_extent_m: float = 0.40
    lateral_margin_m: float = 0.15
    stop_distance_m: float = 0.70
    slow_distance_m: float = 1.20
    min_turn_radius_m: float = 1.20
    min_steering: float = -0.35
    max_steering: float = 0.0
    slow_forward_limit: float = 0.35
    path_sample_step_m: float = 0.05
    max_map_age_s: float = 0.30


@dataclass(frozen=True)
class SafetyDecision:
    decision: int
    permitted_forward: float
    permitted_steering: float
    reason: str
    nearest_path_distance_m: float | None = None


def occupied_points_from_grid(
    data,
    *,
    width: int,
    height: int,
    resolution_m: float,
    origin_x_m: float,
    origin_y_m: float,
    occupied_threshold: int = 100,
) -> np.ndarray:
    """Return XY centres for occupied cells from an OccupancyGrid-like array."""

    values = np.asarray(data, dtype=np.int16)
    if width <= 0 or height <= 0 or values.size != width * height:
        raise ValueError("occupancy grid dimensions do not match its data")
    grid = values.reshape(height, width)
    rows, cols = np.nonzero(grid >= occupied_threshold)
    return np.column_stack(
        (
            origin_x_m + (cols.astype(np.float32) + 0.5) * resolution_m,
            origin_y_m + (rows.astype(np.float32) + 0.5) * resolution_m,
        )
    ).astype(np.float32, copy=False)


def evaluate_safety(
    intent: OperatorIntentData,
    front_obstacles_xy: np.ndarray,
    map_age_s: float,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    """Limit one operator request without selecting or initiating a path."""

    if not config.enable_motion:
        return _stop("live_control_disabled")
    if not config.geometry_calibrated:
        return _stop("uncalibrated_geometry")
    if not math.isfinite(map_age_s) or map_age_s < 0.0:
        return _stop("invalid_map_age")
    if map_age_s > config.max_map_age_s:
        return _stop("stale_map")
    if not intent.deadman:
        return _stop("deadman_released")
    if not math.isfinite(intent.forward) or not math.isfinite(intent.steering):
        return _stop("invalid_intent")
    if intent.forward < 0.0:
        return _stop("reverse_disabled")
    if intent.forward > 1.0 or abs(intent.steering) > 1.0:
        return _stop("invalid_intent")
    if intent.forward == 0.0:
        return _stop("no_forward_intent")
    if intent.steering > config.max_steering:
        return _stop("left_turn_unobserved")
    if intent.steering < config.min_steering:
        return _stop("right_turn_limit_exceeded")

    steering = float(intent.steering)
    nearest = nearest_swept_obstacle_distance(
        front_obstacles_xy, steering, config
    )
    if nearest is not None and nearest <= config.stop_distance_m:
        return _stop("obstacle_stop", nearest)
    if nearest is not None and nearest <= config.slow_distance_m:
        return SafetyDecision(
            SLOW,
            min(float(intent.forward), config.slow_forward_limit),
            steering,
            "obstacle_slow",
            nearest,
        )
    return SafetyDecision(
        CLEAR,
        float(intent.forward),
        steering,
        "clear",
        nearest,
    )


def nearest_swept_obstacle_distance(
    obstacles_xy: np.ndarray,
    steering: float,
    config: SafetyConfig,
) -> float | None:
    """Return the earliest centreline distance whose footprint hits an obstacle."""

    obstacles = np.asarray(obstacles_xy, dtype=np.float32)
    if obstacles.size == 0:
        return None
    if obstacles.ndim != 2 or obstacles.shape[1] != 2:
        raise ValueError("obstacles_xy must have shape (N, 2)")

    step = config.path_sample_step_m
    samples = max(1, int(np.ceil(config.slow_distance_m / step)))
    distances = np.linspace(0.0, config.slow_distance_m, samples + 1)
    curvature = steering / config.min_turn_radius_m
    if abs(curvature) < 1e-6:
        xs = distances
        ys = np.zeros_like(distances)
        yaws = np.zeros_like(distances)
    else:
        yaws = curvature * distances
        xs = np.sin(yaws) / curvature
        ys = (1.0 - np.cos(yaws)) / curvature

    half_width = config.chair_width_m / 2.0 + config.lateral_margin_m
    for distance, x_m, y_m, yaw_rad in zip(distances, xs, ys, yaws):
        dx = obstacles[:, 0] - x_m
        dy = obstacles[:, 1] - y_m
        cosine = np.cos(yaw_rad)
        sine = np.sin(yaw_rad)
        longitudinal = cosine * dx + sine * dy
        lateral = -sine * dx + cosine * dy
        collision = (
            (longitudinal >= -config.rear_extent_m)
            & (longitudinal <= config.front_extent_m)
            & (np.abs(lateral) <= half_width)
        )
        if np.any(collision):
            return float(distance)
    return None


def _stop(reason: str, nearest: float | None = None) -> SafetyDecision:
    return SafetyDecision(STOP, 0.0, 0.0, reason, nearest)


__all__ = [
    "CLEAR",
    "SLOW",
    "STOP",
    "OperatorIntentData",
    "SafetyConfig",
    "SafetyDecision",
    "evaluate_safety",
    "nearest_swept_obstacle_distance",
    "occupied_points_from_grid",
]
