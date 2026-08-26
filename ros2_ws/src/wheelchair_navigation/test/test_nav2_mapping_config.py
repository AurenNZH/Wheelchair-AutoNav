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


def test_dual_l2_pipeline_has_symmetric_sources_and_optional_inflation():
    parameters = _parameters()
    obstacle = parameters["obstacle_layer"]
    right_l2 = obstacle["L2_lidar_right"]
    left_l2 = obstacle["L2_lidar_left"]

    assert parameters["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle["observation_sources"] == (
        "L2_lidar_right L2_lidar_left"
    )
    assert right_l2["topic"] == "/nav2_obstacle_points_right"
    assert left_l2["topic"] == "/nav2_obstacle_points_left"
    assert {
        key: value for key, value in right_l2.items() if key != "topic"
    } == {
        key: value for key, value in left_l2.items() if key != "topic"
    }
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


def test_launch_filters_both_l2_sources_symmetrically_before_nav2():
    path = Path(__file__).parents[1] / "launch" / "nav2_mapping.launch.py"
    source = path.read_text()

    assert 'executable="point_support_filter"' in source
    assert 'support_filter_right = _support_filter(' in source
    assert '"right", "right", artifact_config' in source
    assert (
        'support_filter_left = _support_filter("left", "left", artifact_config)'
        in source
    )
    assert 'topic_root = "/lidar_%s" % side' in source
    assert '"lidar_topic": topic_root + "/points"' in source
    assert '"filtered_cloud_topic": topic_root + "/points_filtered"' in source
    assert '"source_header_topic": topic_root + "/filter/source_header"' in source
    assert '"low_support_points_topic": topic_root + "/low_support_points"' in source
    assert '"artifact_rejected_points_topic"' in source
    assert '"artifact_markers_topic"' in source
    assert '"use_%s_artifact_filter" % side' in source
    assert '"l2_artifact_filters.yaml"' in source
    assert '"/lidar_right/points_filtered"' in source
    assert '"/lidar_left/points_filtered"' in source
    assert "L2_lidar_right=/lidar_right/points_filtered" in source
    assert "L2_lidar_left=/lidar_left/points_filtered" in source
    assert '"min_points_per_cell"' in source
    assert 'default_value="3"' in source
    assert '"diagnostic_name"' in source
    assert '"use_sim_time": LaunchConfiguration("use_sim_time")' in source
    assert '"validate_cloud_timestamps"' in source
    assert "/rslidar_points" not in source


def test_l2_artifact_rules_are_independent_and_active_by_default():
    root = Path(__file__).parents[1]
    document = yaml.safe_load(
        (root / "config" / "l2_artifact_filters.yaml").read_text()
    )
    right = document["l2_lidar_right_support_filter"]["ros__parameters"]
    left = document["l2_lidar_left_support_filter"]["ros__parameters"]
    launch = (root / "launch" / "nav2_mapping.launch.py").read_text()

    assert right["artifact_filter_frame"] == "base_link"
    assert left["artifact_filter_frame"] == "base_link"
    assert right["artifact_box"] != left["artifact_box"]
    assert right["artifact_halo_margin_m"] == 0.10
    assert left["artifact_halo_margin_m"] == 0.10
    assert right["artifact_halo_min_points_per_cell"] == 15
    assert left["artifact_halo_min_points_per_cell"] == 15
    assert '"use_right_artifact_filter",\n                default_value="true"' in launch
    assert '"use_left_artifact_filter",\n                default_value="true"' in launch


def test_rviz_defaults_to_both_filtered_l2s_and_retains_raw_comparison():
    path = Path(__file__).parents[1] / "rviz" / "nav2_front_costmap.rviz"
    document = yaml.safe_load(path.read_text())
    displays = {
        display["Name"]: display
        for display in document["Visualization Manager"]["Displays"]
    }
    raw = displays["L2 right PointCloud2 (raw)"]
    raw_left = displays["L2 left PointCloud2 (raw)"]
    filtered = displays[
        "L2 right PointCloud2 (three-point filtered)"
    ]
    filtered_left = displays[
        "L2 left PointCloud2 (three-point filtered)"
    ]

    assert raw["Topic"]["Value"] == "/lidar_right/points"
    assert raw["Enabled"] is False
    assert raw["Decay Time"] == 1.2
    assert filtered["Topic"]["Value"] == "/lidar_right/points_filtered"
    assert filtered["Enabled"] is True
    assert filtered["Decay Time"] == 1.2
    assert raw_left["Topic"]["Value"] == "/lidar_left/points"
    assert raw_left["Enabled"] is False
    assert raw_left["Decay Time"] == 1.2
    assert filtered_left["Topic"]["Value"] == "/lidar_left/points_filtered"
    assert filtered_left["Enabled"] is True
    assert filtered_left["Decay Time"] == 1.2
    assert filtered["Color"] != filtered_left["Color"]
    assert displays["L2 right artifact box and halo"]["Enabled"] is True
    assert displays["L2 left artifact box and halo"]["Enabled"] is True
    assert displays["L2 right hard-rejected artifacts"]["Enabled"] is False
    assert displays["L2 left hard-rejected artifacts"]["Enabled"] is False


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
