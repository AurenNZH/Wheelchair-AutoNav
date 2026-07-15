import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument

from wheelchair_bringup.defaults import LAUNCH_DEFAULTS, SENSOR_TRANSFORMS


def test_defaults_are_safe_and_start_both_sensors():
    assert LAUNCH_DEFAULTS["use_lidar"] == "true"
    assert LAUNCH_DEFAULTS["use_camera"] == "true"
    assert LAUNCH_DEFAULTS["use_navigation"] == "false"
    assert LAUNCH_DEFAULTS["use_rviz"] == "false"
    assert LAUNCH_DEFAULTS["publish_base_lidar_tf"] == "true"


def test_sensor_mount_transforms_are_siblings_under_base_link():
    assert SENSOR_TRANSFORMS == {
        "rslidar": {
            "x": "-0.265",
            "y": "-0.330",
            "z": "0.320",
            "yaw": "-0.78540",
            "pitch": "0.0",
            "roll": "0.0",
            "parent": "base_link",
            "child": "rslidar",
        },
        "camera_link": {
            "x": "0.265",
            "y": "0.360",
            "z": "1.300",
            "yaw": "-1.5708",
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
