from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import yaml

from wheelchair_shared_control.freshness import NAV2_LIVE
from wheelchair_shared_control.supervisor_node import SafetySupervisorNode


def _parameters():
    path = Path(__file__).parents[1] / "config" / "shared_control.yaml"
    document = yaml.safe_load(path.read_text())
    return document["/**"]["ros__parameters"]


def test_production_defaults_to_weighted_nav2_costs():
    parameters = _parameters()

    assert parameters["merged_costmap_topic"] == "/nav2_merged_costmap"
    assert parameters["source_header_topic"] == (
        "/lidar_right/filter/source_header"
    )
    assert parameters["left_source_header_topic"] == (
        "/lidar_left/filter/source_header"
    )
    assert parameters["freshness_mode"] == NAV2_LIVE
    assert parameters["max_map_age_s"] == 0.5
    assert parameters["max_source_age_s"] == 0.5
    assert parameters["slow_cost_threshold"] == 1
    assert parameters["stop_cost_threshold"] == 99
    assert parameters["slow_forward_limit"] == 0.60
    assert parameters["reverse_limit"] == 0.65
    assert parameters["turn_clearance_radius_m"] == 0.55
    assert parameters["clear_turn_limit"] == 0.90
    assert parameters["slow_turn_limit"] == 0.60
    assert parameters["turn_longitudinal_limit"] == 0.15
    assert parameters["min_steering"] == -0.577350269
    assert parameters["max_steering"] == 0.577350269
    assert parameters["forward_cone_half_angle_deg"] == 30.0
    assert parameters["enable_motion"] is False
    assert parameters["geometry_calibrated"] is False
    assert parameters["avoidance_mode"] == "disabled"
    assert parameters["avoidance_suggestion_max_age_s"] == 0.25
    assert parameters["avoidance_source_steering_tolerance"] == 0.05
    assert parameters["maximum_steering_assist"] == 0.15


def test_launch_exposes_costmap_topic_and_thresholds():
    path = Path(__file__).parents[1] / "launch" / "shared_control.launch.py"
    source = path.read_text()

    assert '"merged_costmap_topic"' in source
    assert 'default_value="/nav2_merged_costmap"' in source
    assert 'default_value="/lidar_right/filter/source_header"' in source
    assert 'default_value="/lidar_left/filter/source_header"' in source
    assert '"freshness_mode", default_value="nav2_live"' in source
    assert '"max_map_age_s", default_value="0.50"' in source
    assert '"max_source_age_s", default_value="0.50"' in source
    assert '"slow_cost_threshold", default_value="1"' in source
    assert '"stop_cost_threshold", default_value="99"' in source
    assert '"forward_cone_half_angle_deg", default_value="30.0"' in source
    assert '"turn_clearance_radius_m", default_value="0.55"' in source
    assert '"clear_turn_limit", default_value="0.90"' in source
    assert '"slow_turn_limit", default_value="0.60"' in source
    assert '"avoidance_mode", default_value="disabled"' in source


def test_replay_uses_nav2_costmap_with_map_stamp_freshness():
    path = Path(__file__).parents[1] / "launch" / "intent_replay.launch.py"
    source = path.read_text()

    assert '"merged_costmap_topic": "/nav2_merged_costmap"' in source
    assert '"freshness_mode": "map_stamp"' in source
    assert '"/front_costmap"' not in source
    assert '"/nav2_front_costmap"' not in source


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
