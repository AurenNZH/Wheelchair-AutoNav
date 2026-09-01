import pytest

from wheelchair_obstacle_avoidance.diagnostic_evidence import (
    CostGrid,
    abort_hint,
    classify_point,
    classify_polygon,
    collect_planning_evidence,
    transform_polygon,
)


def _grid(*, width=8, height=8, data=None, received=9.5):
    if data is None:
        data = [0] * (width * height)
    return CostGrid(
        width=width,
        height=height,
        resolution=1.0,
        origin_x=-2.0,
        origin_y=-2.0,
        origin_yaw=0.0,
        data=tuple(data),
        stamp_sec=12,
        stamp_nanosec=34,
        received_monotonic=received,
    )


def _footprint():
    return ((-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4))


def _evidence(grid=None, footprint=None):
    return collect_planning_evidence(
        _grid() if grid is None else grid,
        _footprint() if footprint is None else footprint,
        3.0,
        0.0,
        0.0,
        now_monotonic=10.0,
    )


def test_point_cost_states_and_outside_are_distinct():
    data = [0] * 64
    data[2 * 8 + 2] = 25
    data[2 * 8 + 3] = 99
    data[2 * 8 + 4] = -1
    grid = _grid(data=data)

    assert classify_point(grid, (0.1, 0.1)).state == "inflated"
    assert classify_point(grid, (1.1, 0.1)).state == "collision"
    assert classify_point(grid, (2.1, 0.1)).state == "unknown"
    assert classify_point(grid, (-2.1, 0.0)).state == "outside"


def test_polygon_rasterization_includes_edges_and_uses_state_precedence():
    data = [0] * 64
    # The footprint spans the four cells around the origin. Put different
    # evidence in two of them and verify collision wins over unknown.
    data[1 * 8 + 1] = -1
    data[2 * 8 + 2] = 100
    evidence = classify_polygon(_grid(data=data), _footprint())

    assert evidence.checked_cells == 4
    assert evidence.unknown_cells == 1
    assert evidence.collision_cells == 1
    assert evidence.state == "collision"


def test_polygon_outside_map_has_highest_precedence():
    evidence = classify_polygon(
        _grid(),
        ((-2.2, -0.2), (-1.8, -0.2), (-1.8, 0.2), (-2.2, 0.2)),
    )

    assert evidence.outside_cells > 0
    assert evidence.state == "outside"


def test_request_evidence_freezes_goal_footprint_and_map_age():
    evidence = collect_planning_evidence(
        _grid(), _footprint(), 3.0, 0.0, 0.0, now_monotonic=10.0
    )

    assert evidence.costmap_available is True
    assert evidence.footprint_available is True
    assert evidence.costmap_age_ms == pytest.approx(500.0)
    assert evidence.costmap_stamp_sec == 12
    assert evidence.goal_center.state == "clear"
    assert evidence.start_footprint.state == "clear"
    assert evidence.goal_footprint.state == "clear"
    assert transform_polygon(((1.0, 0.0),), 2.0, 3.0, 0.0) == (
        (3.0, 3.0),
    )


@pytest.mark.parametrize(
    ("mutate", "expected"),
    [
        ("costmap", "costmap_unavailable"),
        ("footprint", "footprint_unavailable"),
        ("start_outside", "start_outside_costmap"),
        ("start_collision", "start_collision"),
        ("start_unknown", "start_unknown"),
        ("goal_outside", "goal_outside_costmap"),
        ("goal_collision", "goal_collision"),
        ("goal_unknown", "goal_unknown"),
    ],
)
def test_abort_hint_reports_request_time_map_evidence(mutate, expected):
    if mutate == "costmap":
        evidence = collect_planning_evidence(
            None, _footprint(), 3.0, 0.0, 0.0, now_monotonic=10.0
        )
    elif mutate == "footprint":
        evidence = collect_planning_evidence(
            _grid(), None, 3.0, 0.0, 0.0, now_monotonic=10.0
        )
    elif mutate == "start_outside":
        evidence = collect_planning_evidence(
            _grid(),
            ((-2.2, -0.2), (-1.8, -0.2), (-1.8, 0.2), (-2.2, 0.2)),
            3.0,
            0.0,
            0.0,
            now_monotonic=10.0,
        )
    elif mutate == "goal_outside":
        evidence = collect_planning_evidence(
            _grid(), _footprint(), 6.0, 0.0, 0.0, now_monotonic=10.0
        )
    else:
        data = [0] * 64
        if mutate == "start_collision":
            data[2 * 8 + 2] = 100
        elif mutate == "start_unknown":
            data[2 * 8 + 2] = -1
        elif mutate == "goal_collision":
            data[2 * 8 + 5] = 100
        elif mutate == "goal_unknown":
            data[2 * 8 + 5] = -1
        evidence = _evidence(_grid(data=data))

    assert abort_hint("aborted", evidence, 0.0, 30.0) == expected


def test_abort_hint_uses_only_positive_nav2_internal_duration_for_budget():
    evidence = _evidence()

    assert abort_hint("succeeded", evidence, 30.0, 30.0) == "not_aborted"
    assert (
        abort_hint("aborted", evidence, 26.9, 30.0)
        == "no_path_or_budget_unknown"
    )
    assert (
        abort_hint("aborted", evidence, 27.0, 30.0)
        == "search_budget_likely"
    )
    assert (
        abort_hint("aborted", evidence, 0.0, 30.0)
        == "no_path_or_budget_unknown"
    )
