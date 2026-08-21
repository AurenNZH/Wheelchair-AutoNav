"""RViz markers for the existing front-costmap safety corridor."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from wheelchair_shared_control.safety import (
    CLEAR,
    SLOW,
    SafetyConfig,
    SafetyDecision,
    WeightedCostmap,
    trajectory_points,
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
    costmap: WeightedCostmap | None,
    requested_steering: float | None,
    config: SafetyConfig,
    label: str,
) -> MarkerArray:
    """Build exact sampled cells, requested path, and decision label."""

    delete = Marker()
    delete.header = header
    delete.action = Marker.DELETEALL
    markers = [delete]

    if costmap is not None and decision.checked_cells:
        cells = Marker()
        cells.header = header
        cells.ns = "checked_cells"
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.pose.orientation.w = 1.0
        cells.scale.x = costmap.resolution_m
        cells.scale.y = costmap.resolution_m
        cells.scale.z = 0.025
        cells.color = _color(decision.decision, 0.45)
        for col, row in decision.checked_cells:
            if 0 <= col < costmap.width and 0 <= row < costmap.height:
                cells.points.append(
                    Point(
                        x=(col + 0.5) * costmap.resolution_m
                        + costmap.origin_x_m,
                        y=(row + 0.5) * costmap.resolution_m
                        + costmap.origin_y_m,
                        z=0.025,
                    )
                )
        markers.append(cells)

    if requested_steering is not None and math.isfinite(requested_steering):
        boundary = Marker()
        boundary.header = header
        boundary.ns = "requested_path"
        boundary.id = 1
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
    text.id = 2
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


__all__ = ["build_checked_corridor_markers"]
