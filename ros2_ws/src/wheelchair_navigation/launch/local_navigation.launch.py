from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory("wheelchair_navigation")
    parameters = os.path.join(package_share, "config", "local_navigation.yaml")
    return LaunchDescription(
        [
            Node(
                package="wheelchair_navigation",
                executable="local_navigation",
                name="local_navigation",
                output="screen",
                parameters=[parameters],
            )
        ]
    )
