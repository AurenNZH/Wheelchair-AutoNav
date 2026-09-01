"""RViz markers for the existing forward safety corridor."""

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
from wheelchair_shared_control.operator_intent import (
    LEFT_TURN,
    RIGHT_TURN,
    classify_normalized_axes,
)
from wheelchair_shared_control.reactive_assistance import (
    ReactiveConfig,
    ReactiveSelection,
)
from wheelchair_shared_control.trajectory import (
    trajectory_points,
    trajectory_points_for_horizon,
)


@dataclass(frozen=True)
class CorridorIntentView:
    """Requested steering and human-readable label for corridor markers."""

    requested_steering: float | None
    label: str
    turn_disc_requested: bool = False


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
    return CorridorIntentView(
        None,
        label,
        classified.intent_class in (LEFT_TURN, RIGHT_TURN),
    )


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
    turn_disc_requested: bool = False,
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

    if turn_disc_requested:
        disc = Marker()
        disc.header = header
        disc.ns = "requested_turn_disc"
        disc.id = 2
        disc.type = Marker.LINE_STRIP
        disc.action = Marker.ADD
        disc.pose.orientation.w = 1.0
        disc.scale.x = 0.035
        disc.color = _color(decision.decision, 0.95)
        segments = 48
        radius = float(config.turn_clearance_radius_m)
        disc.points = [
            Point(
                x=radius * math.cos(2.0 * math.pi * index / segments),
                y=radius * math.sin(2.0 * math.pi * index / segments),
                z=0.06,
            )
            for index in range(segments + 1)
        ]
        markers.append(disc)

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


def build_reactive_candidate_markers(
    *,
    header: Header,
    selection: ReactiveSelection | None,
    confirmed_steering: float | None,
    config: SafetyConfig,
    reactive_config: ReactiveConfig,
    status: str,
) -> MarkerArray:
    """Build the individual arcs considered by reactive assistance."""

    delete = Marker()
    delete.header = header
    delete.action = Marker.DELETEALL
    markers = [delete]
    if selection is None:
        return MarkerArray(markers=markers)

    for marker_id, candidate in enumerate(selection.candidates):
        path = Marker()
        path.header = header
        path.ns = "reactive_candidate"
        path.id = marker_id
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.025
        selected = (
            selection.selected_steering is not None
            and abs(candidate.steering - selection.selected_steering) < 1e-6
            and selection.valid
        )
        requested = abs(
            candidate.steering - selection.requested_steering
        ) < 1e-6
        confirmed = (
            confirmed_steering is not None
            and abs(candidate.steering - confirmed_steering) < 1e-6
        )
        if not candidate.valid:
            path.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.85)
        elif confirmed:
            path.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)
            path.scale.x = 0.045
        elif selected:
            path.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)
            path.scale.x = 0.04
        elif requested:
            path.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            path.scale.x = 0.035
        else:
            path.color = ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.65)
        path.points = [
            Point(x=x_m, y=y_m, z=0.10)
            for x_m, y_m in trajectory_points_for_horizon(
                candidate.steering,
                config,
                horizon_m=reactive_config.horizon_m,
                sample_step_m=reactive_config.path_sample_step_m,
            )
        ]
        markers.append(path)

    text = Marker()
    text.header = header
    text.ns = "reactive_status"
    text.id = 1000
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = 0.25
    text.pose.position.y = -0.75
    text.pose.position.z = 0.35
    text.pose.orientation.w = 1.0
    text.scale.z = 0.14
    text.color = ColorRGBA(r=0.0, g=0.9, b=1.0, a=1.0)
    text.text = "reactive: %s" % status
    markers.append(text)
    return MarkerArray(markers=markers)


__all__ = [
    "CorridorIntentView",
    "build_checked_corridor_markers",
    "build_reactive_candidate_markers",
    "corridor_intent_view",
]
