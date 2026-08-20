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
    max_steering: float = 0.466307658
    slow_forward_limit: float = 0.30
    reverse_limit: float = 0.65
    path_sample_step_m: float = 0.05
    steering_sample_step: float = 0.05
    neutral_deadzone: float = 0.05
    forward_cone_half_angle_deg: float = 25.0
    forward_right_cone_half_angle_deg: float = 30.0
    max_map_age_s: float = 0.50
    slow_cost_threshold: int = 1
    stop_cost_threshold: int = 99
    enable_hard_right_turn: bool = False
    partial_turn_coverage_acknowledged: bool = False
    max_turn_map_age_s: float = 0.50
    footprint_half_length_m: float = 0.40
    footprint_half_width_m: float = 0.35


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
class PathCostSummary:
    """Cost evidence sampled across the requested trajectory union."""

    maximum_cost: int | None = None
    nearest_slow_distance_m: float | None = None
    nearest_stop_distance_m: float | None = None
    valid: bool = False
    failure_reason: str | None = None
    checked_cells: tuple[tuple[int, int], ...] = ()


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
    checked_cells: tuple[tuple[int, int], ...] = ()
    permitted_lateral: float = 0.0
    permitted_longitudinal: float = 0.0


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


def evaluate_safety(
    intent: OperatorIntentData,
    front_costmap: WeightedCostmap,
    map_age_s: float,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    """Limit forward motion by Nav2 costs and cap unmonitored reverse."""

    if not config.enable_motion:
        return _stop("live_control_disabled")
    if not config.geometry_calibrated:
        return _stop("uncalibrated_geometry")
    if not math.isfinite(map_age_s) or map_age_s < 0.0:
        return _stop("invalid_map_age")
    if map_age_s > config.max_map_age_s:
        return _stop("stale_map")
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
            forward_right_cone_half_angle_deg=(
                config.forward_right_cone_half_angle_deg
            ),
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
        magnitude = min(abs(float(intent.longitudinal)), config.reverse_limit)
        scale = magnitude / abs(float(intent.longitudinal))
        return SafetyDecision(
            SLOW,
            magnitude,
            steering,
            "reverse_unmonitored_slow",
            permitted_lateral=float(intent.lateral) * scale,
            permitted_longitudinal=-magnitude,
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
        permitted = min(
            float(intent.longitudinal), config.slow_forward_limit
        )
        scale = permitted / float(intent.longitudinal)
        return SafetyDecision(
            SLOW,
            permitted,
            steering,
            "nav2_cost_slow",
            summary.nearest_slow_distance_m,
            summary.maximum_cost,
            summary.nearest_slow_distance_m,
            summary.nearest_stop_distance_m,
            True,
            summary.checked_cells,
            float(intent.lateral) * scale,
            permitted,
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
        summary.checked_cells,
        float(intent.lateral),
        float(intent.longitudinal),
    )


def evaluate_right_turn_safety(
    intent: OperatorIntentData,
    turn_costmap: WeightedCostmap,
    map_age_s: float,
    config: SafetyConfig = SafetyConfig(),
) -> SafetyDecision:
    """Evaluate an attended forward/right or clockwise pivot request."""

    if not config.enable_motion:
        return _stop("live_control_disabled")
    if not config.geometry_calibrated:
        return _stop("uncalibrated_geometry")
    if not config.enable_hard_right_turn:
        return _stop("right_turn_not_enabled")
    if not config.partial_turn_coverage_acknowledged:
        return _stop("partial_turn_coverage_not_acknowledged")
    if not math.isfinite(map_age_s) or map_age_s < 0.0:
        return _stop("invalid_turn_map_age")
    if map_age_s > config.max_turn_map_age_s:
        return _stop("stale_turn_map")
    try:
        classified = classify_normalized_axes(
            intent.lateral,
            intent.longitudinal,
            neutral_deadzone=config.neutral_deadzone,
            forward_cone_half_angle_deg=config.forward_cone_half_angle_deg,
            forward_right_cone_half_angle_deg=(
                config.forward_right_cone_half_angle_deg
            ),
        )
    except ValueError:
        return _stop("invalid_intent")
    if (
        int(intent.intent_class) != classified.intent_class
        or bool(intent.deadman) != classified.deadman
    ):
        return _stop("intent_class_mismatch")
    if not intent.deadman:
        return _stop("deadman_released")
    if classified.intent_class != RIGHT_TURN:
        return _stop("unsupported_turn_intent")
    if classified.longitudinal < 0.0:
        return _stop("reverse_right_not_enabled")

    cells, failure_reason = right_turn_checked_cells(turn_costmap, config)
    if failure_reason is not None:
        return SafetyDecision(
            STOP,
            0.0,
            0.0,
            failure_reason,
            path_cost_valid=False,
            checked_cells=cells,
        )
    maximum_cost = max(
        int(turn_costmap.costs[row, col]) for col, row in cells
    )
    if maximum_cost >= config.stop_cost_threshold:
        return SafetyDecision(
            STOP,
            0.0,
            0.0,
            "nav2_right_turn_stop",
            maximum_path_cost=maximum_cost,
            path_cost_valid=True,
            checked_cells=cells,
        )

    decision = SLOW if maximum_cost >= config.slow_cost_threshold else CLEAR
    reason = (
        "nav2_right_turn_slow"
        if decision == SLOW
        else "nav2_right_turn_clear"
    )
    local_limit = config.slow_forward_limit if decision == SLOW else 1.0
    requested_max = max(
        abs(float(classified.lateral)),
        abs(float(classified.longitudinal)),
    )
    scale = 1.0 if requested_max <= local_limit else local_limit / requested_max
    permitted_lateral = float(classified.lateral) * scale
    permitted_longitudinal = float(classified.longitudinal) * scale
    return SafetyDecision(
        decision,
        abs(permitted_longitudinal),
        0.0,
        reason,
        maximum_path_cost=maximum_cost,
        path_cost_valid=True,
        checked_cells=cells,
        permitted_lateral=permitted_lateral,
        permitted_longitudinal=permitted_longitudinal,
    )


def right_turn_checked_cells(
    costmap: WeightedCostmap,
    config: SafetyConfig,
) -> tuple[tuple[tuple[int, int], ...], str | None]:
    """Return a conservative pivot disk plus the forward-right corridor."""

    cells = []
    cell_set = set()
    radius = math.hypot(
        config.footprint_half_length_m,
        config.footprint_half_width_m,
    )
    for row in range(costmap.height):
        y_m = costmap.origin_y_m + (row + 0.5) * costmap.resolution_m
        for col in range(costmap.width):
            x_m = costmap.origin_x_m + (col + 0.5) * costmap.resolution_m
            if math.hypot(x_m, y_m) <= radius:
                cells.append((col, row))
                cell_set.add((col, row))

    corridor = swept_path_costs(costmap, config.min_steering, config)
    for cell in corridor.checked_cells:
        if cell not in cell_set:
            cells.append(cell)
            cell_set.add(cell)
    if not corridor.valid:
        return tuple(cells), corridor.failure_reason
    for col, row in cells:
        if not (0 <= col < costmap.width and 0 <= row < costmap.height):
            return tuple(cells), "turn_sweep_outside_costmap"
        if int(costmap.costs[row, col]) < 0:
            return tuple(cells), "unknown_right_turn_cost"
    if not cells:
        return (), "empty_right_turn_sweep"
    return tuple(cells), None


def swept_path_costs(
    costmap: WeightedCostmap,
    steering: float,
    config: SafetyConfig,
) -> PathCostSummary:
    """Sample Nav2 costs from straight through the requested correction."""

    validate_cost_policy(config)
    steering_step = float(config.steering_sample_step)
    count = max(1, int(math.ceil(abs(float(steering)) / steering_step)))
    candidates = np.linspace(0.0, float(steering), count + 1)
    if abs(float(steering)) < 1e-9:
        candidates = np.array([0.0])

    maximum_cost = None
    nearest_slow = None
    nearest_stop = None
    checked_cells = []
    checked_cell_set = set()
    for candidate in candidates:
        summary = _costs_for_steering(costmap, float(candidate), config)
        if not summary.valid:
            return summary
        for cell in summary.checked_cells:
            if cell not in checked_cell_set:
                checked_cell_set.add(cell)
                checked_cells.append(cell)
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
        checked_cells=tuple(checked_cells),
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
    xs = np.asarray([point[0] for point in points])
    ys = np.asarray([point[1] for point in points])

    maximum_cost = None
    nearest_slow = None
    nearest_stop = None
    checked_cells = []
    checked_cell_set = set()
    for distance, x_m, y_m in zip(distances, xs, ys):
        col = math.floor(
            (float(x_m) - costmap.origin_x_m) / costmap.resolution_m
        )
        row = math.floor(
            (float(y_m) - costmap.origin_y_m) / costmap.resolution_m
        )
        cell = (col, row)
        if cell not in checked_cell_set:
            checked_cell_set.add(cell)
            checked_cells.append(cell)
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
                checked_cells=tuple(checked_cells),
            )
        cost = int(costmap.costs[row, col])
        if cost < 0:
            return PathCostSummary(
                maximum_cost=maximum_cost,
                nearest_slow_distance_m=nearest_slow,
                nearest_stop_distance_m=nearest_stop,
                valid=False,
                failure_reason="unknown_nav2_cost",
                checked_cells=tuple(checked_cells),
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
        checked_cells=tuple(checked_cells),
    )


def trajectory_points(
    steering: float,
    config: SafetyConfig,
) -> tuple[tuple[float, float], ...]:
    """Return the exact robot-centre samples used for one path check."""

    step = float(config.path_sample_step_m)
    samples = max(1, int(math.ceil(config.slow_distance_m / step)))
    distances = np.linspace(0.0, config.slow_distance_m, samples + 1)
    curvature = steering / config.min_turn_radius_m
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


def validate_cost_policy(config: SafetyConfig) -> None:
    finite_positive = (
        config.path_sample_step_m,
        config.steering_sample_step,
        config.min_turn_radius_m,
        config.stop_distance_m,
        config.slow_distance_m,
        config.max_turn_map_age_s,
        config.footprint_half_length_m,
        config.footprint_half_width_m,
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
        1
        <= config.slow_cost_threshold
        < config.stop_cost_threshold
        <= 100
    ):
        raise ValueError(
            "cost thresholds must satisfy 1 <= slow < stop <= 100"
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
        summary.checked_cells,
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
    "evaluate_right_turn_safety",
    "right_turn_checked_cells",
    "swept_path_costs",
    "trajectory_points",
    "validate_cost_policy",
    "weighted_costmap_from_grid",
]
