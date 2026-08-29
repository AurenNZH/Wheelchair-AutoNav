import math

import pytest

from wheelchair_obstacle_avoidance.planning import (
    FORWARD,
    FORWARD_LEFT,
    FORWARD_RIGHT,
    PlanningConfig,
    StraightSideHysteresis,
    bound_assisted_steering,
    path_steering,
    temporary_goal,
    validate_path,
)


def _straight_path(goal_x=3.8, goal_y=0.0):
    return tuple(
        (goal_x * step / 19.0, goal_y * step / 19.0)
        for step in range(20)
    )


def test_fixed_goal_tracks_heading_not_joystick_magnitude():
    first = temporary_goal(0.1, 0.5)
    second = temporary_goal(0.2, 1.0)
    assert first == pytest.approx(second)
    assert math.hypot(first[0], first[1]) == pytest.approx(3.8)


@pytest.mark.parametrize(
    "points, reason",
    [
        (((0.0, 0.0), (1.0, 0.9), (3.8, 0.0)), "cross_track_limit"),
        (((0.0, 0.0), (1.0, 0.0), (0.8, 0.0), (3.8, 0.0)), "reverse_progress"),
        (((0.0, 0.0), (3.5, 0.0)), "endpoint_mismatch"),
        (((0.3, 0.0), (3.8, 0.0)), "start_mismatch"),
        ((), "invalid_path"),
    ],
)
def test_route_acceptance_gates(points, reason):
    result = validate_path(points, (3.8, 0.0), PlanningConfig())
    assert not result.valid
    assert result.reason == reason


def test_route_rejects_long_detour_within_cross_track_band():
    points = (
        (0.0, 0.0),
        (0.9, 0.79),
        (1.8, -0.79),
        (2.7, 0.79),
        (3.8, 0.0),
    )
    result = validate_path(points, (3.8, 0.0))
    assert not result.valid
    assert result.reason == "path_length_limit"


def test_valid_path_and_pure_pursuit_steering():
    points = _straight_path(3.8, 0.38)
    result = validate_path(points, (3.8, 0.38))
    assert result.valid
    assert 0.0 < path_steering(points) <= PlanningConfig().maximum_steering


def test_straight_authority_can_choose_either_side_but_not_exceed_cap():
    config = PlanningConfig(maximum_assist=0.15)
    assert bound_assisted_steering(0.0, FORWARD, 0.4, 0.15, config)[0] == 0.15
    assert bound_assisted_steering(0.0, FORWARD, -0.4, 0.10, config)[0] == -0.10


def test_correction_authority_only_reduces_towards_straight():
    config = PlanningConfig(maximum_assist=0.15)
    left, _ = bound_assisted_steering(0.30, FORWARD_LEFT, -0.4, 0.15, config)
    right, _ = bound_assisted_steering(-0.30, FORWARD_RIGHT, 0.4, 0.15, config)
    assert left == pytest.approx(0.15)
    assert right == pytest.approx(-0.15)
    assert bound_assisted_steering(0.30, FORWARD_LEFT, 0.5, 0.15, config)[0] == 0.30
    assert bound_assisted_steering(-0.30, FORWARD_RIGHT, -0.5, 0.15, config)[0] == -0.30


def test_zero_authority_and_tiny_corrections_preserve_manual_steering():
    assert bound_assisted_steering(0.0, FORWARD, 0.1, 0.0)[0] == 0.0
    assisted, reason = bound_assisted_steering(0.0, FORWARD, 0.019, 0.15)
    assert assisted == 0.0
    assert reason == "below_minimum_correction"


def test_straight_side_switch_needs_two_consecutive_accepted_plans():
    state = StraightSideHysteresis()
    assert state.filter(0.1) == (0.1, True)
    assert state.filter(-0.1) == (0.0, False)
    assert state.filter(-0.1) == (-0.1, True)
    assert state.filter(0.1) == (0.0, False)
    assert state.filter(-0.1) == (-0.1, True)
