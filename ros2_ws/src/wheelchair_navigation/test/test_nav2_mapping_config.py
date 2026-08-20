from pathlib import Path

import yaml


def _parameters():
    path = Path(__file__).parents[1] / "config" / "nav2_front_costmap.yaml"
    document = yaml.safe_load(path.read_text())
    return document["costmap"]["costmap"]["ros__parameters"]


def _right_turn_parameters():
    path = (
        Path(__file__).parents[1]
        / "config"
        / "nav2_right_turn_costmap.yaml"
    )
    document = yaml.safe_load(path.read_text())
    return document["right_turn"]["costmap"]["costmap"][
        "ros__parameters"
    ]


def test_stock_costmap_is_forward_base_link_grid():
    parameters = _parameters()

    assert parameters["global_frame"] == "base_link"
    assert parameters["robot_base_frame"] == "base_link"
    assert parameters["rolling_window"] is False
    assert parameters["width"] == 4.0
    assert parameters["height"] == 8.0
    assert parameters["origin_x"] == 0.0
    assert parameters["origin_y"] == -4.0
    assert parameters["resolution"] == 0.1


def test_filtered_pipeline_has_disabled_optional_inflation_after_obstacles():
    parameters = _parameters()
    obstacle = parameters["obstacle_layer"]
    airy = obstacle["airy"]

    assert parameters["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert airy["topic"] == "/nav2_obstacle_points"
    assert airy["data_type"] == "PointCloud2"
    assert airy["marking"] is True
    assert airy["clearing"] is True
    assert airy["observation_persistence"] == 0.0
    assert (
        parameters["inflation_layer.plugin"]
        == "nav2_costmap_2d::InflationLayer"
    )
    assert parameters["inflation_layer.enabled"] is False
    assert parameters["inflation_layer.inflation_radius"] == 0.55
    assert parameters["inflation_layer.cost_scaling_factor"] == 3.0
    assert "denoise_layer" not in parameters
    assert "artifact_grid_mask_cells" not in parameters


def test_launch_defaults_to_filter_and_retains_raw_ab_switch():
    path = Path(__file__).parents[1] / "launch" / "nav2_mapping.launch.py"
    source = path.read_text()

    assert '"use_artifact_filter", default_value="true"' in source
    assert 'executable="artifact_point_filter"' in source
    assert '"/rslidar_points_artifact_filtered"' in source
    assert '"/rslidar_points"' in source
    assert 'UnlessCondition(' in source


def test_launch_exposes_disabled_tunable_inflation_profile():
    path = Path(__file__).parents[1] / "launch" / "nav2_mapping.launch.py"
    source = path.read_text()

    assert '"use_inflation", default_value="false"' in source
    assert '"inflation_radius", default_value="0.55"' in source
    assert '"cost_scaling_factor", default_value="3.0"' in source
    assert "RewrittenYaml(" in source
    assert '"inflation_layer.enabled"' in source
    assert '"inflation_layer.inflation_radius"' in source
    assert '"inflation_layer.cost_scaling_factor"' in source
    assert "convert_types=True" in source


def test_right_turn_costmap_is_centered_in_base_link_and_inflated():
    parameters = _right_turn_parameters()

    assert parameters["global_frame"] == "base_link"
    assert parameters["width"] == 8
    assert parameters["height"] == 8
    assert parameters["origin_x"] == -4.0
    assert parameters["origin_y"] == -4.0
    assert parameters["track_unknown_space"] is False
    assert parameters["obstacle_layer"]["airy"]["topic"] == (
        "/nav2_obstacle_points"
    )
    assert parameters["inflation_layer.enabled"] is True


def test_right_turn_costmap_launch_is_disabled_by_default():
    path = Path(__file__).parents[1] / "launch" / "nav2_mapping.launch.py"
    source = path.read_text()

    assert '"use_right_turn_costmap", default_value="false"' in source
    assert 'namespace="right_turn"' in source
    assert '"/nav2_right_turn_costmap"' in source
    assert '"/right_turn/costmap/costmap"' in source
