import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
import yaml

from wheelchair_bringup.defaults import LAUNCH_DEFAULTS, SENSOR_TRANSFORMS


def test_defaults_are_lidar_first_and_non_actuating():
    assert LAUNCH_DEFAULTS["use_lidar"] == "true"
    assert LAUNCH_DEFAULTS["use_camera"] == "false"
    assert LAUNCH_DEFAULTS["publish_camera_tf"] == "false"
    assert LAUNCH_DEFAULTS["use_mapping"] == "false"
    assert LAUNCH_DEFAULTS["use_rviz"] == "false"
    assert LAUNCH_DEFAULTS["runtime_profile"] == "safety"
    assert LAUNCH_DEFAULTS["publish_base_lidar_tf"] == "true"


def test_sensor_mount_transforms_are_siblings_under_base_link():
    assert SENSOR_TRANSFORMS == {
        "rslidar": {
            "x": "0.330",
            "y": "-0.265",
            "z": "0.320",
            "yaw": "1.04720",
            "pitch": "0.0",
            "roll": "0.0",
            "parent": "base_link",
            "child": "rslidar",
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


def test_safety_profile_uses_latest_only_lidar_and_lightweight_rviz():
    package = Path(__file__).parents[1]
    lidar = yaml.safe_load(
        (package / "config" / "rslidar_airy.yaml").read_text()
    )
    safety_rviz = (
        package / "rviz" / "wheelchair_safety.rviz"
    ).read_text()
    debug_rviz = (package / "rviz" / "wheelchair.rviz").read_text()

    assert lidar["lidar"][0]["ros"]["ros_queue_length"] == 1
    assert "/front_costmap" in safety_rviz
    assert "Frame Rate: 10" in safety_rviz
    assert "PointCloud2" not in safety_rviz
    assert "PointCloud2" in debug_rviz
