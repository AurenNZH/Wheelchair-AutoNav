"""Pure interpretation helpers for Nav2 ComputePathToPose results."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus


_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


def planner_action_status_name(status: int) -> str:
    """Return a stable readable name for a ROS action status value."""

    return _STATUS_NAMES.get(int(status), "invalid_%d" % int(status))


def completed_result_failure_reason(
    *,
    status: int,
    pose_count: int,
    frame_id: str,
    elapsed_ms: float,
    discard_after_ms: float,
    expected_frame: str = "base_link",
) -> str | None:
    """Classify terminal state and path structure before path geometry."""

    status = int(status)
    if status == GoalStatus.STATUS_ABORTED:
        return "planner_aborted"
    if status == GoalStatus.STATUS_CANCELED:
        return "planner_canceled"
    if status != GoalStatus.STATUS_SUCCEEDED:
        return "planner_status_invalid"
    if int(pose_count) <= 0:
        return "planner_empty_path"
    if str(frame_id) != expected_frame:
        return "path_frame_mismatch"
    elapsed_ms = float(elapsed_ms)
    if not math.isfinite(elapsed_ms) or elapsed_ms > float(discard_after_ms):
        return "planner_late"
    return None


def duration_to_milliseconds(duration) -> float | None:
    """Convert a canonical ROS Duration, returning None when malformed."""

    try:
        seconds = int(duration.sec)
        nanoseconds = int(duration.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return None
    return seconds * 1000.0 + nanoseconds / 1_000_000.0


def optional_milliseconds_text(value: float | None) -> str:
    """Format an optional millisecond measurement for ROS diagnostics."""

    if value is None or not math.isfinite(float(value)):
        return "none"
    return "%.3f" % float(value)


__all__ = [
    "completed_result_failure_reason",
    "duration_to_milliseconds",
    "optional_milliseconds_text",
    "planner_action_status_name",
]
