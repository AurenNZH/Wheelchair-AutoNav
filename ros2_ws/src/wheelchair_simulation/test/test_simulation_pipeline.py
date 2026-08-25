from pathlib import Path


def test_simulated_sensor_uses_production_l2_interface():
    root = Path(__file__).parents[1]
    urdf = (root / "urdf" / "wheelchair.urdf.xacro").read_text()

    assert 'name="lidar_right_link"' in urdf
    assert 'name="lidar_left_link"' in urdf
    assert 'name="lidar_left_mount"' in urdf
    assert "/lidar_right/points" in urdf
    assert "sensor_msgs/PointCloud2" in urdf
    assert 'name="rslidar"' not in urdf


def test_shared_control_sim_uses_nav2_mapping_launch():
    path = Path(__file__).parents[1] / "launch" / "shared_control_sim.launch.py"
    source = path.read_text()

    assert '"nav2_mapping.launch.py"' in source
    assert 'executable="local_costmap"' not in source
    assert '"use_sim_time": "true"' in source
