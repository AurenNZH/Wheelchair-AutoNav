"""Bring up both Ethernet-connected Unitree L2 sensors."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _argument(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=str(default),
        description=description,
    )


def generate_launch_description():
    share = get_package_share_directory("wheelchair_bringup")
    right_launch = os.path.join(share, "launch", "l2_right.launch.py")
    left_launch = os.path.join(share, "launch", "l2_left.launch.py")

    declarations = [
        _argument(
            "right_config_file",
            os.path.join(share, "config", "unitree_l2_right.yaml"),
            "Right-L2 ROS parameter file",
        ),
        _argument(
            "left_config_file",
            os.path.join(share, "config", "unitree_l2_left.yaml"),
            "Left-L2 ROS parameter file",
        ),
        _argument("publish_mount_tf", "true", "Publish the right-L2 mount TF"),
        _argument("mount_x", "0.330", "Right-L2 X translation in metres"),
        _argument("mount_y", "-0.220", "Right-L2 Y translation in metres"),
        _argument("mount_z", "0.320", "Right-L2 Z translation in metres"),
        _argument("mount_yaw", "0.479965544", "Right-L2 yaw in radians"),
        _argument("mount_pitch", "0.0", "Right-L2 pitch in radians"),
        _argument("mount_roll", "0.0", "Right-L2 roll in radians"),
        _argument(
            "publish_left_mount_tf",
            "true",
            "Publish the left-L2 mount TF",
        ),
        _argument("left_mount_x", "0.330", "Left-L2 X translation in metres"),
        _argument("left_mount_y", "0.220", "Left-L2 Y translation in metres"),
        _argument("left_mount_z", "0.320", "Left-L2 Z translation in metres"),
        _argument("left_mount_yaw", "-0.479965544", "Left-L2 yaw in radians"),
        _argument("left_mount_pitch", "0.0", "Left-L2 pitch in radians"),
        _argument("left_mount_roll", "0.0", "Left-L2 roll in radians"),
        _argument("use_robot_model", "true", "Publish the wheelchair URDF model"),
        _argument("use_rviz", "true", "Start RViz with the wheelchair view"),
        _argument("use_sim_time", "false", "Use the ROS simulation clock"),
    ]

    right = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(right_launch),
                launch_arguments={
                    "config_file": LaunchConfiguration("right_config_file"),
                    "publish_mount_tf": LaunchConfiguration("publish_mount_tf"),
                    "mount_x": LaunchConfiguration("mount_x"),
                    "mount_y": LaunchConfiguration("mount_y"),
                    "mount_z": LaunchConfiguration("mount_z"),
                    "mount_yaw": LaunchConfiguration("mount_yaw"),
                    "mount_pitch": LaunchConfiguration("mount_pitch"),
                    "mount_roll": LaunchConfiguration("mount_roll"),
                    "publish_left_mount_tf": LaunchConfiguration(
                        "publish_left_mount_tf"
                    ),
                    "left_mount_x": LaunchConfiguration("left_mount_x"),
                    "left_mount_y": LaunchConfiguration("left_mount_y"),
                    "left_mount_z": LaunchConfiguration("left_mount_z"),
                    "left_mount_yaw": LaunchConfiguration("left_mount_yaw"),
                    "left_mount_pitch": LaunchConfiguration(
                        "left_mount_pitch"
                    ),
                    "left_mount_roll": LaunchConfiguration("left_mount_roll"),
                    "use_robot_model": LaunchConfiguration("use_robot_model"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        ],
        scoped=True,
    )
    left = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(left_launch),
                launch_arguments={
                    "config_file": LaunchConfiguration("left_config_file"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        ],
        scoped=True,
    )

    return LaunchDescription(declarations + [right, left])
