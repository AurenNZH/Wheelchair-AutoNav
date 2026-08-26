"""Bring up the left Unitree L2 for isolated hardware validation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wheelchair_bringup")
    default_config = os.path.join(
        share,
        "config",
        "unitree_l2_left.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Left-L2 ROS parameter file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock",
            ),
            Node(
                package="unitree_lidar_ros2",
                executable="unitree_lidar_ros2_node",
                name="unitree_l2_left",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[("/tf", "/lidar_left/vendor_tf")],
            ),
        ]
    )
