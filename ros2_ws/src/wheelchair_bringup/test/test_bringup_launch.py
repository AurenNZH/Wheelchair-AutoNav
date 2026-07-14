import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument

from wheelchair_bringup.defaults import CAMERA_LIDAR_TRANSFORM, LAUNCH_DEFAULTS


def test_defaults_are_safe_and_start_both_sensors():
    assert LAUNCH_DEFAULTS["use_lidar"] == "true"
    assert LAUNCH_DEFAULTS["use_camera"] == "true"
    assert LAUNCH_DEFAULTS["use_navigation"] == "false"
    assert LAUNCH_DEFAULTS["use_rviz"] == "false"
    assert LAUNCH_DEFAULTS["publish_base_lidar_tf"] == "false"


def test_camera_mount_transform_matches_marked_sensor_setup():
    assert CAMERA_LIDAR_TRANSFORM == {
        "x": "0.42",
        "y": "0.65",
        "z": "1.03",
        "yaw": "-0.78540",
        "pitch": "0.0",
        "roll": "0.0",
        "parent": "rslidar",
        "child": "camera_link",
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
