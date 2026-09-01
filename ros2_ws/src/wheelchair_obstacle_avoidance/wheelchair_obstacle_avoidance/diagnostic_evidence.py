"""Pure costmap evidence helpers for local-planner diagnostics.

These helpers deliberately classify what was visible when a request was made;
they do not decide whether the wheelchair may move.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


Point = Tuple[float, float]


@dataclass(frozen=True)
class CostGrid:
    """An immutable occupancy grid in its source frame."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]
    stamp_sec: int = 0
    stamp_nanosec: int = 0
    received_monotonic: float = 0.0

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("cost grid dimensions must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("cost grid resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("cost grid data length does not match dimensions")
        if not all(
            math.isfinite(value)
            for value in (self.origin_x, self.origin_y, self.origin_yaw)
        ):
            raise ValueError("cost grid origin must be finite")


@dataclass(frozen=True)
class CellEvidence:
    state: str
    cost: int | None


@dataclass(frozen=True)
class RegionEvidence:
    state: str
    checked_cells: int
    max_cost: int | None
    unknown_cells: int
    collision_cells: int
    inflated_cells: int
    outside_cells: int


@dataclass(frozen=True)
class PlanningEvidence:
    costmap_available: bool
    footprint_available: bool
    goal_x: float
    goal_y: float
    goal_heading: float
    costmap_age_ms: float | None = None
    costmap_stamp_sec: int | None = None
    costmap_stamp_nanosec: int | None = None
    goal_center: CellEvidence | None = None
    start_footprint: RegionEvidence | None = None
    goal_footprint: RegionEvidence | None = None


def transform_polygon(points: Sequence[Point], x: float, y: float, yaw: float):
    """Return ``points`` rigidly transformed by a planar pose."""

    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return tuple(
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in points
    )


def _to_grid_local(grid: CostGrid, point: Point) -> Point:
    dx = point[0] - grid.origin_x
    dy = point[1] - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def classify_point(grid: CostGrid, point: Point) -> CellEvidence:
    """Classify the cell containing a point."""

    local_x, local_y = _to_grid_local(grid, point)
    column = math.floor(local_x / grid.resolution)
    row = math.floor(local_y / grid.resolution)
    if column < 0 or row < 0 or column >= grid.width or row >= grid.height:
        return CellEvidence("outside", None)
    cost = int(grid.data[row * grid.width + column])
    return CellEvidence(_cost_state(cost), cost)


def _cost_state(cost: int) -> str:
    if cost < 0:
        return "unknown"
    if cost >= 99:
        return "collision"
    if cost > 0:
        return "inflated"
    return "clear"


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (
        b[1] - a[1]
    ) * (c[0] - a[0])


def _on_segment(
    a: Point, b: Point, point: Point, epsilon: float = 1e-12
) -> bool:
    return (
        min(a[0], b[0]) - epsilon <= point[0] <= max(a[0], b[0]) + epsilon
        and min(a[1], b[1]) - epsilon
        <= point[1]
        <= max(a[1], b[1]) + epsilon
        and abs(_orientation(a, b, point)) <= epsilon
    )


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    epsilon = 1e-12
    crosses_ab = (ab_c > epsilon and ab_d < -epsilon) or (
        ab_c < -epsilon and ab_d > epsilon
    )
    crosses_cd = (cd_a > epsilon and cd_b < -epsilon) or (
        cd_a < -epsilon and cd_b > epsilon
    )
    if crosses_ab and crosses_cd:
        return True
    return any(
        (
            abs(value) <= epsilon and _on_segment(first, second, point)
        )
        for value, first, second, point in (
            (ab_c, a, b, c),
            (ab_d, a, b, d),
            (cd_a, c, d, a),
            (cd_b, c, d, b),
        )
    )


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _on_segment(previous, current, point):
            return True
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing_x = (
                (previous[0] - current[0])
                * (point[1] - current[1])
                / (previous[1] - current[1])
                + current[0]
            )
            if point[0] < crossing_x:
                inside = not inside
        previous = current
    return inside


def _polygon_intersects_cell(
    polygon: Sequence[Point],
    left: float,
    bottom: float,
    right: float,
    top: float,
) -> bool:
    def in_cell(point: Point) -> bool:
        return left <= point[0] <= right and bottom <= point[1] <= top

    if any(in_cell(point) for point in polygon):
        return True
    corners = ((left, bottom), (right, bottom), (right, top), (left, top))
    if any(_point_in_polygon(corner, polygon) for corner in corners):
        return True
    cell_edges = tuple(zip(corners, corners[1:] + corners[:1]))
    polygon_edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    return any(
        _segments_intersect(start, end, cell_start, cell_end)
        for start, end in polygon_edges
        for cell_start, cell_end in cell_edges
    )


def cells_intersecting_polygon(grid: CostGrid, polygon: Sequence[Point]):
    """Yield every grid index whose cell is touched by ``polygon``.

    Indices outside the grid are included so callers can report a footprint
    that extends beyond the available map.
    """

    if len(polygon) < 3:
        raise ValueError("footprint must contain at least three points")
    if not all(math.isfinite(value) for point in polygon for value in point):
        raise ValueError("footprint points must be finite")
    local = tuple(_to_grid_local(grid, point) for point in polygon)
    resolution = grid.resolution
    min_column = math.floor(min(point[0] for point in local) / resolution)
    max_column = math.floor(max(point[0] for point in local) / resolution)
    min_row = math.floor(min(point[1] for point in local) / resolution)
    max_row = math.floor(max(point[1] for point in local) / resolution)
    for row in range(min_row, max_row + 1):
        bottom = row * resolution
        top = bottom + resolution
        for column in range(min_column, max_column + 1):
            left = column * resolution
            right = left + resolution
            if _polygon_intersects_cell(local, left, bottom, right, top):
                yield column, row


def classify_polygon(
    grid: CostGrid, polygon: Sequence[Point]
) -> RegionEvidence:
    """Summarize costs under a footprint with fail-visible precedence."""

    checked = 0
    maximum = None
    unknown = 0
    collision = 0
    inflated = 0
    outside = 0
    for column, row in cells_intersecting_polygon(grid, polygon):
        checked += 1
        if column < 0 or row < 0 or column >= grid.width or row >= grid.height:
            outside += 1
            continue
        cost = int(grid.data[row * grid.width + column])
        if cost < 0:
            unknown += 1
            continue
        maximum = cost if maximum is None else max(maximum, cost)
        if cost >= 99:
            collision += 1
        elif cost > 0:
            inflated += 1
    if outside:
        state = "outside"
    elif collision:
        state = "collision"
    elif unknown:
        state = "unknown"
    elif inflated:
        state = "inflated"
    else:
        state = "clear"
    return RegionEvidence(
        state=state,
        checked_cells=checked,
        max_cost=maximum,
        unknown_cells=unknown,
        collision_cells=collision,
        inflated_cells=inflated,
        outside_cells=outside,
    )


def collect_planning_evidence(
    grid: CostGrid | None,
    footprint: Sequence[Point] | None,
    goal_x: float,
    goal_y: float,
    goal_heading: float,
    *,
    now_monotonic: float,
) -> PlanningEvidence:
    """Freeze map and footprint observations for one planner request."""

    base = dict(
        costmap_available=grid is not None,
        footprint_available=footprint is not None,
        goal_x=float(goal_x),
        goal_y=float(goal_y),
        goal_heading=float(goal_heading),
    )
    if grid is None:
        return PlanningEvidence(**base)
    age_ms = max(0.0, (now_monotonic - grid.received_monotonic) * 1000.0)
    if footprint is None:
        return PlanningEvidence(
            **base,
            costmap_age_ms=age_ms,
            costmap_stamp_sec=grid.stamp_sec,
            costmap_stamp_nanosec=grid.stamp_nanosec,
            goal_center=classify_point(grid, (goal_x, goal_y)),
        )
    start = tuple((float(x), float(y)) for x, y in footprint)
    goal = transform_polygon(start, goal_x, goal_y, goal_heading)
    return PlanningEvidence(
        **base,
        costmap_age_ms=age_ms,
        costmap_stamp_sec=grid.stamp_sec,
        costmap_stamp_nanosec=grid.stamp_nanosec,
        goal_center=classify_point(grid, (goal_x, goal_y)),
        start_footprint=classify_polygon(grid, start),
        goal_footprint=classify_polygon(grid, goal),
    )


def abort_hint(
    planner_action_status: str,
    evidence: PlanningEvidence | None,
    nav2_planning_time_ms: float | None,
    planner_search_budget_ms: float,
) -> str:
    """Return a conservative diagnostic hint for an aborted Nav2 request."""

    if planner_action_status != "aborted":
        return "not_aborted"
    if evidence is None or not evidence.costmap_available:
        return "costmap_unavailable"
    if not evidence.footprint_available:
        return "footprint_unavailable"
    start = evidence.start_footprint
    goal = evidence.goal_footprint
    if start is None or goal is None:
        return "footprint_unavailable"
    precedence = (
        (start.state == "outside", "start_outside_costmap"),
        (start.state == "collision", "start_collision"),
        (start.state == "unknown", "start_unknown"),
        (goal.state == "outside", "goal_outside_costmap"),
        (goal.state == "collision", "goal_collision"),
        (goal.state == "unknown", "goal_unknown"),
    )
    for applies, hint in precedence:
        if applies:
            return hint
    if (
        nav2_planning_time_ms is not None
        and math.isfinite(float(nav2_planning_time_ms))
        and float(nav2_planning_time_ms) > 0.0
        and float(nav2_planning_time_ms)
        >= 0.9 * float(planner_search_budget_ms)
    ):
        return "search_budget_likely"
    return "no_path_or_budget_unknown"


__all__ = [
    "CellEvidence",
    "CostGrid",
    "PlanningEvidence",
    "RegionEvidence",
    "abort_hint",
    "cells_intersecting_polygon",
    "classify_point",
    "classify_polygon",
    "collect_planning_evidence",
    "transform_polygon",
]
