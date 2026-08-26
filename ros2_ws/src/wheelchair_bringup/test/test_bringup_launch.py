import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument

from wheelchair_bringup.defaults import LAUNCH_DEFAULTS, SENSOR_TRANSFORMS


def test_defaults_are_lidar_first_and_non_actuating():
    assert LAUNCH_DEFAULTS["use_lidar"] == "true"
    assert LAUNCH_DEFAULTS["use_camera"] == "false"
    assert LAUNCH_DEFAULTS["publish_camera_tf"] == "false"
    assert LAUNCH_DEFAULTS["use_mapping"] == "false"
    assert LAUNCH_DEFAULTS["use_rviz"] == "false"
    assert LAUNCH_DEFAULTS["use_robot_model"] == "true"
    assert LAUNCH_DEFAULTS["publish_lidar_tf"] == "true"
    assert LAUNCH_DEFAULTS["publish_left_lidar_tf"] == "true"


def test_sensor_mount_transforms_are_siblings_under_base_link():
    assert SENSOR_TRANSFORMS == {
        "lidar_right_link": {
            "x": "0.330",
            "y": "-0.220",
            "z": "0.320",
            "yaw": "0.479965544",
            "pitch": "0.0",
            "roll": "0.0",
            "parent": "base_link",
            "child": "lidar_right_link",
        },
        "lidar_left_link": {
            "x": "0.330",
            "y": "0.220",
            "z": "0.320",
            "yaw": "-0.479965544",
            "pitch": "0.0",
            "roll": "0.0",
            "parent": "base_link",
            "child": "lidar_left_link",
        },
        "camera_link": {
            "x": "-0.360",
            "y": "0.265",
            "z": "1.300",
            "yaw": "0.0",
            "pitch": "0.0",
            "roll": "0.0",
            "parent": "base_link",
            "child": "camera_link",
        },
    }


def test_launch_description_declares_every_public_argument():
    launch_path = Path(__file__).parents[1] / "launch" / "wheelchair.launch.py"
    spec = importlib.util.spec_from_file_location("wheelchair_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    description = module.generate_launch_description()
    declared = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert declared == set(LAUNCH_DEFAULTS)


def test_right_l2_configuration_retains_validated_runtime_contract():
    config_path = Path(__file__).parents[1] / "config" / "unitree_l2_right.yaml"
    source = config_path.read_text()

    assert "lidar_ip: 192.168.1.62" in source
    assert "local_ip: 192.168.1.2" in source
    assert "cloud_topic: /lidar_right/points" in source
    assert "cloud_frame: lidar_right_link" in source
    assert "cloud_scan_num: 18" in source
    assert "range_min: 0.45" in source


def test_left_l2_configuration_uses_distinct_network_and_ros_interfaces():
    root = Path(__file__).parents[1]
    config_source = (root / "config" / "unitree_l2_left.yaml").read_text()
    launch_source = (root / "launch" / "l2_left.launch.py").read_text()

    assert "unitree_l2_left:" in config_source
    assert "lidar_ip: 192.168.1.63" in config_source
    assert "local_ip: 192.168.1.2" in config_source
    assert "local_port: 6202" in config_source
    assert "cloud_topic: /lidar_left/points" in config_source
    assert "cloud_frame: lidar_left_link" in config_source
    assert 'name="unitree_l2_left"' in launch_source
    assert launch_source.count('package="unitree_lidar_ros2"') == 1
    assert '("/tf", "/lidar_left/vendor_tf")' in launch_source


def test_dual_l2_launch_composes_both_validated_driver_launches():
    root = Path(__file__).parents[1]
    source = (root / "launch" / "dual_l2.launch.py").read_text()
    top_level = (root / "launch" / "wheelchair.launch.py").read_text()
    rviz_source = (root / "rviz" / "wheelchair.rviz").read_text()

    assert '"l2_right.launch.py"' in source
    assert '"l2_left.launch.py"' in source
    assert '"unitree_l2_right.yaml"' in source
    assert '"unitree_l2_left.yaml"' in source
    assert source.count("scoped=True") == 2
    assert '"use_robot_model": LaunchConfiguration("use_robot_model")' in source
    assert '"use_rviz": LaunchConfiguration("use_rviz")' in source
    assert '"dual_l2.launch.py"' in top_level
    assert "Value: /lidar_right/points_filtered" in rviz_source
    assert "Value: /lidar_left/points_filtered" in rviz_source
    assert "Value: /lidar_right/low_support_points" in rviz_source
    assert "Value: /lidar_left/low_support_points" in rviz_source


def test_right_l2_launch_restores_model_and_defined_left_mount_without_driver():
    root = Path(__file__).parents[1]
    launch_source = (root / "launch" / "l2_right.launch.py").read_text()
    rviz_source = (root / "rviz" / "wheelchair.rviz").read_text()

    assert 'executable="robot_state_publisher"' in launch_source
    assert 'executable="joint_state_publisher"' in launch_source
    assert 'name="lidar_left_mount_tf"' in launch_source
    assert '"base_link",\n            "lidar_left_link"' in launch_source
    assert 'package="unitree_lidar_ros2"' in launch_source
    assert launch_source.count('package="unitree_lidar_ros2"') == 1
    assert "rviz_default_plugins/RobotModel" in rviz_source


def test_child_rviz_arguments_are_scoped_from_top_level_rviz_toggle():
    launch_path = Path(__file__).parents[1] / "launch" / "wheelchair.launch.py"
    source = launch_path.read_text()

    assert "GroupAction" in source
    assert source.count("scoped=True") == 2
    assert source.count('"use_rviz": "false"') == 2


def test_left_mac_utility_is_explicit_and_guarded():
    tool = (
        Path(__file__).parents[1] / "tools" / "configure_l2_left_mac.cpp"
    ).read_text()

    assert 'kApplyArgument[] = "--apply"' in tool
    assert 'kLidarIp[] = "192.168.1.63"' in tool
    assert 'kJetsonIp[] = "192.168.1.2"' in tool
    assert "kJetsonPort = 6202" in tool
    assert "{0x02, 0x29, 0xab, 0x7c, 0x00, 0x63}" in tool
    assert "setLidarMacAddressConfig(config)" in tool
