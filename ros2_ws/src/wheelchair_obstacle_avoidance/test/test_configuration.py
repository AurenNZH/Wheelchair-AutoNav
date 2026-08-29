from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_smac_hybrid_configuration_is_forward_only_and_latency_bounded():
    data = yaml.safe_load((PACKAGE / "config" / "local_planner.yaml").read_text())
    planner = data["nav2_planner"]["ros__parameters"]
    plugin = planner["LocalAvoidance"]
    assert planner["expected_planner_frequency"] == 10.0
    assert plugin["plugin"] == "smac_planner/SmacPlanner"
    assert plugin["motion_model_for_search"] == "DUBIN"
    assert plugin["minimum_turning_radius"] == 1.2
    assert plugin["angle_quantization_bins"] == 36
    assert plugin["max_planning_time_ms"] == 30.0
    assert plugin["downsample_costmap"] is False
    assert plugin["smooth_path"] is False
    assert plugin["allow_unknown"] is False
    supported_foxy_keys = {
        "plugin",
        "tolerance",
        "downsample_costmap",
        "downsampling_factor",
        "allow_unknown",
        "max_iterations",
        "max_planning_time_ms",
        "motion_model_for_search",
        "angle_quantization_bins",
        "minimum_turning_radius",
        "reverse_penalty",
        "change_penalty",
        "non_straight_penalty",
        "cost_penalty",
        "analytic_expansion_ratio",
        "smooth_path",
    }
    assert set(plugin) == supported_foxy_keys


def test_planner_costmap_is_small_robot_relative_and_inflated():
    data = yaml.safe_load((PACKAGE / "config" / "local_planner.yaml").read_text())
    costmap = data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["global_frame"] == "base_link"
    assert costmap["robot_base_frame"] == "base_link"
    cells = costmap["width"] * costmap["height"] / costmap["resolution"] ** 2
    assert cells == pytest.approx(4000)
    assert costmap["track_unknown_space"] is True
    assert costmap["inflation_layer"]["enabled"] is True
    assert costmap["inflation_layer"]["inflation_radius"] == 0.55
    assert costmap["inflation_layer"]["cost_scaling_factor"] == 3.0
