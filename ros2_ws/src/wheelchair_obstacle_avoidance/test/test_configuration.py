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


def test_planner_owned_costmap_outputs_use_absolute_remaps():
    source = (PACKAGE / "launch" / "obstacle_avoidance.launch.py").read_text()
    expected = {
        '("/global_costmap/costmap", "/nav2_merged_costmap")',
        '("/global_costmap/costmap_raw", "/nav2_merged_costmap_raw")',
        '"/global_costmap/costmap_updates"',
        '"/global_costmap/published_footprint"',
    }
    for remap in expected:
        assert remap in source

    # Relative spellings do not match the embedded planner costmap in Foxy.
    assert '("global_costmap/costmap",' not in source
    assert '("global_costmap/costmap_raw",' not in source


def test_shadow_validation_accepts_results_up_to_300_ms():
    launch_source = (
        PACKAGE / "launch" / "obstacle_avoidance.launch.py"
    ).read_text()
    node_source = (
        PACKAGE / "wheelchair_obstacle_avoidance" / "planner_node.py"
    ).read_text()
    assert 'LaunchConfiguration("discard_after_ms")' in launch_source
    assert 'default_value="300.0"' in launch_source
    assert 'declare_parameter("discard_after_ms", 300.0)' in node_source


def test_planner_diagnostics_separate_nav2_and_round_trip_timings():
    node_source = (
        PACKAGE / "wheelchair_obstacle_avoidance" / "planner_node.py"
    ).read_text()
    assert 'key="planning_time_ms"' in node_source
    assert 'key="nav2_planning_time_ms"' in node_source
    assert 'key="planner_action_status"' in node_source
