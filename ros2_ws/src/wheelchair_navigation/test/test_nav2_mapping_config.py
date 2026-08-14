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


def test_initial_experiment_uses_only_stock_obstacle_layer():
    parameters = _parameters()
    obstacle = parameters["obstacle_layer"]
    airy = obstacle["airy"]

    assert parameters["plugins"] == ["obstacle_layer"]
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert airy["topic"] == "/rslidar_points"
    assert airy["data_type"] == "PointCloud2"
    assert airy["marking"] is True
    assert airy["clearing"] is True
    assert airy["observation_persistence"] == 0.0
    assert "denoise_layer" not in parameters
    assert "artifact_grid_mask_cells" not in parameters
