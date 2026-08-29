from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus

from wheelchair_obstacle_avoidance.result_handling import (
    completed_result_failure_reason,
    duration_to_milliseconds,
    optional_milliseconds_text,
    planner_action_status_name,
)


@pytest.mark.parametrize(
    "status, name",
    [
        (GoalStatus.STATUS_UNKNOWN, "unknown"),
        (GoalStatus.STATUS_ACCEPTED, "accepted"),
        (GoalStatus.STATUS_EXECUTING, "executing"),
        (GoalStatus.STATUS_CANCELING, "canceling"),
        (GoalStatus.STATUS_SUCCEEDED, "succeeded"),
        (GoalStatus.STATUS_CANCELED, "canceled"),
        (GoalStatus.STATUS_ABORTED, "aborted"),
        (99, "invalid_99"),
    ],
)
def test_action_status_has_stable_diagnostic_name(status, name):
    assert planner_action_status_name(status) == name


@pytest.mark.parametrize(
    "status, expected",
    [
        (GoalStatus.STATUS_ABORTED, "planner_aborted"),
        (GoalStatus.STATUS_CANCELED, "planner_canceled"),
        (GoalStatus.STATUS_UNKNOWN, "planner_status_invalid"),
        (GoalStatus.STATUS_EXECUTING, "planner_status_invalid"),
    ],
)
def test_terminal_failure_status_precedes_empty_path_or_lateness(
    status, expected
):
    reason = completed_result_failure_reason(
        status=status,
        pose_count=0,
        frame_id="",
        elapsed_ms=450.0,
        discard_after_ms=300.0,
    )
    assert reason == expected


def test_successful_path_structure_precedes_lateness():
    empty = completed_result_failure_reason(
        status=GoalStatus.STATUS_SUCCEEDED,
        pose_count=0,
        frame_id="",
        elapsed_ms=450.0,
        discard_after_ms=300.0,
    )
    wrong_frame = completed_result_failure_reason(
        status=GoalStatus.STATUS_SUCCEEDED,
        pose_count=2,
        frame_id="map",
        elapsed_ms=450.0,
        discard_after_ms=300.0,
    )
    assert empty == "planner_empty_path"
    assert wrong_frame == "path_frame_mismatch"


def test_only_structurally_valid_slow_success_is_planner_late():
    late = completed_result_failure_reason(
        status=GoalStatus.STATUS_SUCCEEDED,
        pose_count=2,
        frame_id="base_link",
        elapsed_ms=300.001,
        discard_after_ms=300.0,
    )
    accepted = completed_result_failure_reason(
        status=GoalStatus.STATUS_SUCCEEDED,
        pose_count=2,
        frame_id="base_link",
        elapsed_ms=300.0,
        discard_after_ms=300.0,
    )
    assert late == "planner_late"
    assert accepted is None


def test_nav2_duration_is_converted_independently_from_round_trip_time():
    duration = SimpleNamespace(sec=2, nanosec=345_678_000)
    assert duration_to_milliseconds(duration) == pytest.approx(2345.678)


@pytest.mark.parametrize(
    "duration",
    [
        None,
        SimpleNamespace(sec=-1, nanosec=0),
        SimpleNamespace(sec=0, nanosec=1_000_000_000),
        SimpleNamespace(sec="invalid", nanosec=0),
    ],
)
def test_malformed_nav2_duration_is_unavailable(duration):
    assert duration_to_milliseconds(duration) is None


def test_optional_duration_diagnostic_format():
    assert optional_milliseconds_text(12.3456) == "12.346"
    assert optional_milliseconds_text(None) == "none"
    assert optional_milliseconds_text(float("nan")) == "none"
