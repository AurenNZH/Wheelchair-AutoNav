import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wheelchair_navigation")
    parameters = os.path.join(package_share, "config", "local_mapping.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="wheelchair_navigation",
                executable="local_costmap",
                name="local_costmap",
                output="screen",
                parameters=[
                    parameters,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            )
        ]
    )
