from pathlib import Path

import yaml


def _parameters():
    path = Path(__file__).parents[1] / "config" / "nav2_front_costmap.yaml"
    document = yaml.safe_load(path.read_text())
    return document["costmap"]["costmap"]["ros__parameters"]


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


def test_right_l2_pipeline_has_disabled_optional_inflation_after_obstacles():
    parameters = _parameters()
    obstacle = parameters["obstacle_layer"]
    right_l2 = obstacle["L2_lidar_right"]

    assert parameters["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle["observation_sources"] == "L2_lidar_right"
    assert right_l2["topic"] == "/nav2_obstacle_points_right"
    assert right_l2["data_type"] == "PointCloud2"
    assert right_l2["marking"] is True
    assert right_l2["clearing"] is True
    assert right_l2["observation_persistence"] == 0.0
    assert (
        parameters["inflation_layer.plugin"]
        == "nav2_costmap_2d::InflationLayer"
    )
    assert parameters["inflation_layer.enabled"] is False
    assert parameters["inflation_layer.inflation_radius"] == 0.55
    assert parameters["inflation_layer.cost_scaling_factor"] == 3.0
    assert "denoise_layer" not in parameters
    assert "artifact_grid_mask_cells" not in parameters


def test_launch_filters_right_l2_support_before_nav2():
    path = Path(__file__).parents[1] / "launch" / "nav2_mapping.launch.py"
    source = path.read_text()

    assert 'executable="point_support_filter"' in source
    assert '"lidar_topic": "/lidar_right/points"' in source
    assert '"/lidar_right/points_filtered"' in source
    assert "L2_lidar_right=/lidar_right/points_filtered" in source
    assert '"min_points_per_cell"' in source
    assert 'default_value="3"' in source
    assert '"/lidar_right/filter/source_header"' in source
    assert '"use_sim_time": LaunchConfiguration("use_sim_time")' in source
    assert '"validate_cloud_timestamps"' in source
    assert "/rslidar_points" not in source


def test_rviz_defaults_to_filtered_l2_and_retains_raw_comparison():
    path = Path(__file__).parents[1] / "rviz" / "nav2_front_costmap.rviz"
    document = yaml.safe_load(path.read_text())
    displays = {
        display["Name"]: display
        for display in document["Visualization Manager"]["Displays"]
    }
    raw = displays["L2 right PointCloud2 (raw)"]
    filtered = displays[
        "L2 right PointCloud2 (three-point filtered)"
    ]

    assert raw["Topic"]["Value"] == "/lidar_right/points"
    assert raw["Enabled"] is False
    assert raw["Decay Time"] == 1.2
    assert filtered["Topic"]["Value"] == "/lidar_right/points_filtered"
    assert filtered["Enabled"] is True
    assert filtered["Decay Time"] == 1.2


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
