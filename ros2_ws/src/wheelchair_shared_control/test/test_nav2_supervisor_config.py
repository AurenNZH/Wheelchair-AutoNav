from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import yaml

from wheelchair_shared_control.safety import SafetyConfig, SafetyDecision
from wheelchair_shared_control.supervisor_node import (
    SafetySupervisorNode,
    safety_diagnostic_values,
)


def _parameters():
    path = Path(__file__).parents[1] / "config" / "shared_control.yaml"
    document = yaml.safe_load(path.read_text())
    return document["/**"]["ros__parameters"]


def test_production_defaults_to_weighted_nav2_costs():
    parameters = _parameters()

    assert parameters["front_costmap_topic"] == "/nav2_front_costmap"
    assert parameters["slow_cost_threshold"] == 1
    assert parameters["stop_cost_threshold"] == 99
    assert parameters["enable_motion"] is False
    assert parameters["geometry_calibrated"] is False


def test_launch_exposes_costmap_topic_and_thresholds():
    path = Path(__file__).parents[1] / "launch" / "shared_control.launch.py"
    source = path.read_text()

    assert '"front_costmap_topic"' in source
    assert 'default_value="/nav2_front_costmap"' in source
    assert '"slow_cost_threshold", default_value="1"' in source
    assert '"stop_cost_threshold", default_value="99"' in source


def test_replay_keeps_legacy_front_topic_explicit():
    path = Path(__file__).parents[1] / "launch" / "intent_replay.launch.py"
    source = path.read_text()

    assert '"front_costmap_topic": "/front_costmap"' in source


def test_supervisor_retains_weighted_grid_values():
    msg = OccupancyGrid()
    msg.header.frame_id = "base_link"
    msg.info.width = 2
    msg.info.height = 2
    msg.info.resolution = 0.1
    msg.info.origin.position.y = -0.1
    msg.info.origin.orientation.w = 1.0
    msg.data = [0, 50, 99, 100]

    costmap = SafetySupervisorNode._grid_costmap(msg)

    assert costmap.costs.tolist() == [[0, 50], [99, 100]]


def test_cost_decision_diagnostics_expose_calibration_evidence():
    config = SafetyConfig(
        slow_cost_threshold=5,
        stop_cost_threshold=90,
    )
    decision = SafetyDecision(
        decision=1,
        permitted_forward=0.2,
        permitted_steering=0.0,
        reason="nav2_cost_slow",
        nearest_path_distance_m=0.8,
        maximum_path_cost=75,
        nearest_slow_cost_distance_m=0.8,
        nearest_stop_cost_distance_m=None,
        path_cost_valid=True,
    )

    values = {
        item.key: item.value
        for item in safety_diagnostic_values(decision, config, 12.5, 1.25)
    }

    assert values["maximum_path_cost"] == "75"
    assert values["nearest_slow_cost_distance_m"] == "0.800"
    assert values["nearest_stop_cost_distance_m"] == "none"
    assert values["slow_cost_threshold"] == "5"
    assert values["stop_cost_threshold"] == "90"
    assert values["path_cost_valid"] == "True"
