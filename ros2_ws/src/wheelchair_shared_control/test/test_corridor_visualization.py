import numpy as np

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from wheelchair_shared_control.corridor_visualization import (
    build_checked_corridor_markers,
)
from wheelchair_shared_control.operator_intent import FORWARD_RIGHT
from wheelchair_shared_control.safety import (
    CLEAR,
    OperatorIntentData,
    SafetyConfig,
    evaluate_safety,
    weighted_costmap_from_grid,
)


def test_markers_show_exact_checked_cells_and_requested_path():
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
        0.1,
        config,
    )

    markers = build_checked_corridor_markers(
        header=Header(frame_id="base_link"),
        decision=decision,
        costmap=costmap,
        requested_steering=-0.55,
        config=config,
        label="forward_right -28.8deg",
    ).markers

    assert decision.decision == CLEAR
    assert markers[0].action == Marker.DELETEALL
    cells = next(marker for marker in markers if marker.ns == "checked_cells")
    path = next(marker for marker in markers if marker.ns == "requested_path")
    label = next(marker for marker in markers if marker.ns == "corridor_label")
    assert len(cells.points) == len(decision.checked_cells)
    assert len(path.points) > 2
    assert "forward_right" in label.text
    assert "nav2_cost_clear" in label.text
