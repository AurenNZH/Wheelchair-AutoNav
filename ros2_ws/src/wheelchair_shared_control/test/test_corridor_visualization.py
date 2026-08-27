import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from wheelchair_shared_control.corridor_visualization import (
    build_checked_corridor_markers,
    corridor_intent_view,
)
from wheelchair_shared_control.operator_intent import FORWARD_RIGHT
from wheelchair_shared_control.models import (
    CLEAR,
    OperatorIntentData,
    SafetyConfig,
    SafetyDecision,
    weighted_costmap_from_grid,
)
from wheelchair_shared_control.safety_policy import evaluate_safety


def test_markers_show_requested_path_and_decision_label():
    costmap = weighted_costmap_from_grid(
        np.zeros(40 * 80, dtype=np.int16),
        frame_id="base_link",
        width=40,
        height=80,
        resolution_m=0.1,
        origin_x_m=0.0,
        origin_y_m=-4.0,
        origin_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    config = SafetyConfig(enable_motion=True, geometry_calibrated=True)
    decision = evaluate_safety(
        OperatorIntentData(
            "session", 1, -0.55, 1.0, FORWARD_RIGHT, True
        ),
        costmap,
        config,
    )

    markers = build_checked_corridor_markers(
        header=Header(frame_id="base_link"),
        decision=decision,
        requested_steering=-0.55,
        config=config,
        label="forward_right -28.8deg",
    ).markers

    assert decision.decision == CLEAR
    assert markers[0].action == Marker.DELETEALL
    path = next(marker for marker in markers if marker.ns == "requested_path")
    label = next(marker for marker in markers if marker.ns == "corridor_label")
    assert all(marker.type != Marker.CUBE_LIST for marker in markers)
    assert len(path.points) > 2
    assert "forward_right" in label.text
    assert "nav2_cost_clear" in label.text


def test_intent_view_preserves_current_legacy_and_reverse_labels():
    config = SafetyConfig()
    forward = corridor_intent_view(
        lateral=-0.55,
        longitudinal=1.0,
        legacy_forward=0.0,
        legacy_steering=0.0,
        config=config,
    )
    legacy = corridor_intent_view(
        lateral=0.0,
        longitudinal=0.0,
        legacy_forward=0.5,
        legacy_steering=0.2,
        config=config,
    )
    reverse = corridor_intent_view(
        lateral=0.0,
        longitudinal=-0.5,
        legacy_forward=0.0,
        legacy_steering=0.0,
        config=config,
    )

    assert forward.requested_steering == -0.55
    assert forward.label.startswith("forward_right ")
    assert legacy.requested_steering == 0.2
    assert legacy.label.startswith("forward_left ")
    assert reverse.requested_steering is None
    assert reverse.label.endswith(" unmonitored")


def test_invalid_intent_view_remains_non_visualized():
    view = corridor_intent_view(
        lateral=np.nan,
        longitudinal=1.0,
        legacy_forward=0.0,
        legacy_steering=0.0,
        config=SafetyConfig(),
    )

    assert view.requested_steering is None
    assert view.label == "invalid_intent"


def test_hard_turn_view_draws_disc_without_checked_cell_cubes():
    config = SafetyConfig(
        enable_motion=True,
        geometry_calibrated=True,
    )
    view = corridor_intent_view(
        lateral=0.8,
        longitudinal=0.0,
        legacy_forward=0.0,
        legacy_steering=0.0,
        config=config,
    )
    decision = SafetyDecision(
        CLEAR,
        0.0,
        0.8,
        "nav2_turn_cost_clear",
        path_cost_valid=True,
    )
    markers = build_checked_corridor_markers(
        header=Header(frame_id="base_link"),
        decision=decision,
        requested_steering=view.requested_steering,
        turn_disc_requested=view.turn_disc_requested,
        config=config,
        label=view.label,
    ).markers

    assert view.label.startswith("left_turn ")
    assert view.turn_disc_requested
    disc = next(
        marker for marker in markers if marker.ns == "requested_turn_disc"
    )
    assert disc.type == Marker.LINE_STRIP
    assert len(disc.points) == 49
    assert all(marker.type != Marker.CUBE_LIST for marker in markers)
