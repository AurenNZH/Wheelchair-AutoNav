"""RViz markers for the existing front-costmap safety corridor."""

from __future__ import annotations

from dataclasses import dataclass
import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from wheelchair_shared_control.models import (
    CLEAR,
    SLOW,
    SafetyConfig,
    SafetyDecision,
)
from wheelchair_shared_control.operator_intent import classify_normalized_axes
from wheelchair_shared_control.trajectory import trajectory_points


@dataclass(frozen=True)
class CorridorIntentView:
    """Requested steering and human-readable label for corridor markers."""

    requested_steering: float | None
    label: str


def corridor_intent_view(
    *,
    lateral: float,
    longitudinal: float,
    legacy_forward: float,
    legacy_steering: float,
    config: SafetyConfig,
) -> CorridorIntentView:
    """Interpret current and legacy intent fields for visualization only."""

    if lateral == 0.0 and longitudinal == 0.0:
        longitudinal = legacy_forward
        lateral = legacy_steering * longitudinal
    try:
        classified = classify_normalized_axes(
            lateral,
            longitudinal,
            neutral_deadzone=config.neutral_deadzone,
            forward_cone_half_angle_deg=(
                config.forward_cone_half_angle_deg
            ),
        )
    except ValueError:
        return CorridorIntentView(None, "invalid_intent")

    angle = (
        "none"
        if classified.heading_deg is None
        else "%.1fdeg" % classified.heading_deg
    )
    label = "%s %s" % (classified.label, angle)
    if classified.is_forward:
        return CorridorIntentView(classified.steering_ratio, label)
    if classified.is_reverse:
        label += " unmonitored"
    return CorridorIntentView(None, label)


def _color(decision: int, alpha: float) -> ColorRGBA:
    if decision == CLEAR:
        return ColorRGBA(r=0.1, g=0.9, b=0.2, a=alpha)
    if decision == SLOW:
        return ColorRGBA(r=1.0, g=0.75, b=0.0, a=alpha)
    return ColorRGBA(r=1.0, g=0.1, b=0.1, a=alpha)


def build_checked_corridor_markers(
    *,
    header: Header,
    decision: SafetyDecision,
    requested_steering: float | None,
    config: SafetyConfig,
    label: str,
) -> MarkerArray:
    """Build the requested path and color-coded decision label."""

    delete = Marker()
    delete.header = header
    delete.action = Marker.DELETEALL
    markers = [delete]

    if requested_steering is not None and math.isfinite(requested_steering):
        boundary = Marker()
        boundary.header = header
        boundary.ns = "requested_path"
        boundary.id = 0
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 0.035
        boundary.color = _color(decision.decision, 0.95)
        boundary.points = [
            Point(x=x_m, y=y_m, z=0.06)
            for x_m, y_m in trajectory_points(requested_steering, config)
        ]
        markers.append(boundary)

    text = Marker()
    text.header = header
    text.ns = "corridor_label"
    text.id = 1
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = 0.25
    text.pose.position.y = 0.75
    text.pose.position.z = 0.35
    text.pose.orientation.w = 1.0
    text.scale.z = 0.16
    text.color = _color(decision.decision, 1.0)
    text.text = "%s | %s" % (label, decision.reason)
    markers.append(text)
    return MarkerArray(markers=markers)


__all__ = [
    "CorridorIntentView",
    "build_checked_corridor_markers",
    "corridor_intent_view",
]
