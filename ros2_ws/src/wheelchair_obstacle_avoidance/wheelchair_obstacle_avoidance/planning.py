"""Pure, odometry-free local-path validation and steering assistance."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple


FORWARD = 1
FORWARD_LEFT = 2
FORWARD_RIGHT = 3
FORWARD_CLASSES = (FORWARD, FORWARD_LEFT, FORWARD_RIGHT)


@dataclass(frozen=True)
class PlanningConfig:
    goal_distance_m: float = 3.8
    maximum_cross_track_m: float = 0.8
    maximum_path_ratio: float = 1.25
    endpoint_tolerance_m: float = 0.2
    reverse_progress_tolerance_m: float = 0.05
    lookahead_distance_m: float = 0.8
    steering_gain: float = 1.2
    minimum_correction: float = 0.02
    maximum_steering: float = 0.577350269
    maximum_assist: float = 0.15


@dataclass(frozen=True)
class PathValidation:
    valid: bool
    reason: str
    path_length_m: float = 0.0
    maximum_cross_track_m: float = 0.0


def validate_config(config: PlanningConfig) -> None:
    values = (
        config.goal_distance_m,
        config.maximum_cross_track_m,
        config.maximum_path_ratio,
        config.endpoint_tolerance_m,
        config.reverse_progress_tolerance_m,
        config.lookahead_distance_m,
        config.steering_gain,
        config.minimum_correction,
        config.maximum_steering,
        config.maximum_assist,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("planning configuration must be finite")
    if config.goal_distance_m <= 0.0:
        raise ValueError("goal_distance_m must be positive")
    if config.maximum_cross_track_m <= 0.0:
        raise ValueError("maximum_cross_track_m must be positive")
    if config.maximum_path_ratio < 1.0:
        raise ValueError("maximum_path_ratio must be at least one")
    if config.endpoint_tolerance_m < 0.0:
        raise ValueError("endpoint_tolerance_m must be non-negative")
    if config.reverse_progress_tolerance_m < 0.0:
        raise ValueError("reverse_progress_tolerance_m must be non-negative")
    if config.lookahead_distance_m <= 0.0 or config.steering_gain <= 0.0:
        raise ValueError("lookahead distance and steering gain must be positive")
    if config.minimum_correction < 0.0:
        raise ValueError("minimum_correction must be non-negative")
    if config.maximum_steering <= 0.0:
        raise ValueError("maximum_steering must be positive")
    if not 0.0 <= config.maximum_assist <= config.maximum_steering:
        raise ValueError("maximum_assist must be within steering limits")


def joystick_heading(lateral: float, longitudinal: float) -> float:
    """Return the forward joystick ray heading in radians."""

    lateral = float(lateral)
    longitudinal = float(longitudinal)
    if not math.isfinite(lateral) or not math.isfinite(longitudinal):
        raise ValueError("joystick axes must be finite")
    if longitudinal <= 0.0:
        raise ValueError("a temporary goal requires forward intent")
    return math.atan2(lateral, longitudinal)


def temporary_goal(
    lateral: float,
    longitudinal: float,
    distance_m: float = 3.8,
) -> Tuple[float, float, float]:
    """Return a fixed-distance base_link goal along the joystick ray."""

    heading = joystick_heading(lateral, longitudinal)
    distance_m = float(distance_m)
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        raise ValueError("goal distance must be finite and positive")
    return (
        distance_m * math.cos(heading),
        distance_m * math.sin(heading),
        heading,
    )


def _finite_points(points: Iterable[Sequence[float]]) -> Tuple[Tuple[float, float], ...]:
    parsed = tuple((float(point[0]), float(point[1])) for point in points)
    if not parsed or not all(
        math.isfinite(x) and math.isfinite(y) for x, y in parsed
    ):
        raise ValueError("path points must be non-empty and finite")
    return parsed


def validate_path(
    points: Iterable[Sequence[float]],
    goal: Sequence[float],
    config: PlanningConfig = PlanningConfig(),
) -> PathValidation:
    """Apply bounded-route gates before a Nav2 path can influence steering."""

    validate_config(config)
    try:
        parsed = _finite_points(points)
        goal_x, goal_y = float(goal[0]), float(goal[1])
    except (IndexError, TypeError, ValueError):
        return PathValidation(False, "invalid_path")
    if not math.isfinite(goal_x) or not math.isfinite(goal_y):
        return PathValidation(False, "invalid_goal")
    direct = math.hypot(goal_x, goal_y)
    if direct <= 0.0:
        return PathValidation(False, "invalid_goal")

    unit_x, unit_y = goal_x / direct, goal_y / direct
    if math.hypot(parsed[0][0], parsed[0][1]) > config.endpoint_tolerance_m:
        return PathValidation(False, "start_mismatch")
    previous_progress = None
    maximum_cross_track = 0.0
    path_length = 0.0
    previous = (0.0, 0.0)
    for point in parsed:
        x, y = point
        progress = x * unit_x + y * unit_y
        cross_track = abs(-unit_y * x + unit_x * y)
        maximum_cross_track = max(maximum_cross_track, cross_track)
        if cross_track > config.maximum_cross_track_m:
            return PathValidation(
                False, "cross_track_limit", path_length, maximum_cross_track
            )
        if progress < -config.reverse_progress_tolerance_m:
            return PathValidation(False, "reverse_progress")
        if (
            previous_progress is not None
            and progress
            < previous_progress - config.reverse_progress_tolerance_m
        ):
            return PathValidation(False, "reverse_progress")
        path_length += math.hypot(x - previous[0], y - previous[1])
        previous = point
        previous_progress = progress

    if path_length > direct * config.maximum_path_ratio:
        return PathValidation(
            False, "path_length_limit", path_length, maximum_cross_track
        )
    if math.hypot(parsed[-1][0] - goal_x, parsed[-1][1] - goal_y) > (
        config.endpoint_tolerance_m
    ):
        return PathValidation(
            False, "endpoint_mismatch", path_length, maximum_cross_track
        )
    return PathValidation(True, "accepted", path_length, maximum_cross_track)


def lookahead_point(
    points: Iterable[Sequence[float]], lookahead_distance_m: float
) -> Tuple[float, float]:
    """Interpolate a point by arc length from the beginning of a path."""

    parsed = _finite_points(points)
    distance = float(lookahead_distance_m)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("lookahead distance must be finite and positive")
    travelled = 0.0
    previous = parsed[0]
    for point in parsed[1:]:
        segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if segment > 0.0 and travelled + segment >= distance:
            fraction = (distance - travelled) / segment
            return (
                previous[0] + fraction * (point[0] - previous[0]),
                previous[1] + fraction * (point[1] - previous[1]),
            )
        travelled += segment
        previous = point
    return parsed[-1]


def path_steering(
    points: Iterable[Sequence[float]],
    config: PlanningConfig = PlanningConfig(),
) -> float:
    """Convert the beginning of an accepted path into a bounded correction."""

    validate_config(config)
    x, y = lookahead_point(points, config.lookahead_distance_m)
    denominator = x * x + y * y
    if denominator <= 1e-9:
        raise ValueError("lookahead point is at the base origin")
    steering = config.steering_gain * (2.0 * y / denominator)
    return max(-config.maximum_steering, min(config.maximum_steering, steering))


def bound_assisted_steering(
    requested: float,
    intent_class: int,
    planned: float,
    advertised_assist: float,
    config: PlanningConfig = PlanningConfig(),
) -> Tuple[float, str]:
    """Apply the user's explicit steering authority to a planner result."""

    validate_config(config)
    requested = float(requested)
    planned = float(planned)
    advertised_assist = float(advertised_assist)
    if not all(math.isfinite(value) for value in (requested, planned, advertised_assist)):
        raise ValueError("steering values must be finite")
    if intent_class not in FORWARD_CLASSES:
        return requested, "ineligible_intent"
    authority = min(max(0.0, advertised_assist), config.maximum_assist)
    if authority <= 0.0:
        return requested, "no_authority"

    if intent_class == FORWARD:
        assisted = max(-authority, min(authority, planned))
    elif intent_class == FORWARD_LEFT:
        assisted = max(max(0.0, requested - authority), min(requested, planned))
    else:
        assisted = min(min(0.0, requested + authority), max(requested, planned))
    assisted = max(-config.maximum_steering, min(config.maximum_steering, assisted))
    if abs(assisted - requested) < config.minimum_correction:
        return requested, "below_minimum_correction"
    return assisted, "assisted"


class StraightSideHysteresis:
    """Require two accepted paths before changing a straight assist side."""

    def __init__(self) -> None:
        self._active_side = 0
        self._pending_side = 0
        self._pending_count = 0

    def reset(self) -> None:
        self._active_side = 0
        self._pending_side = 0
        self._pending_count = 0

    def filter(self, steering: float) -> Tuple[float, bool]:
        side = 1 if steering > 0.0 else -1 if steering < 0.0 else 0
        if side == 0:
            self.reset()
            return 0.0, True
        if self._active_side in (0, side):
            self._active_side = side
            self._pending_side = 0
            self._pending_count = 0
            return steering, True
        if self._pending_side != side:
            self._pending_side = side
            self._pending_count = 1
            return 0.0, False
        self._pending_count += 1
        if self._pending_count < 2:
            return 0.0, False
        self._active_side = side
        self._pending_side = 0
        self._pending_count = 0
        return steering, True


__all__ = [
    "FORWARD",
    "FORWARD_LEFT",
    "FORWARD_RIGHT",
    "FORWARD_CLASSES",
    "PathValidation",
    "PlanningConfig",
    "StraightSideHysteresis",
    "bound_assisted_steering",
    "joystick_heading",
    "lookahead_point",
    "path_steering",
    "temporary_goal",
    "validate_config",
    "validate_path",
]
