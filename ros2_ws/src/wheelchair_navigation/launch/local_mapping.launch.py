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
            DeclareLaunchArgument(
                "validate_cloud_timestamps", default_value="true"
            ),
            DeclareLaunchArgument(
                "restamp_output_with_node_time", default_value="false"
            ),
            DeclareLaunchArgument(
                "publish_artifact_shadow", default_value="true"
            ),
            Node(
                package="wheelchair_navigation",
                executable="local_costmap",
                name="local_costmap",
                output="screen",
                parameters=[
                    parameters,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "validate_cloud_timestamps": LaunchConfiguration(
                            "validate_cloud_timestamps"
                        ),
                        "restamp_output_with_node_time": LaunchConfiguration(
                            "restamp_output_with_node_time"
                        ),
                        "publish_artifact_shadow": LaunchConfiguration(
                            "publish_artifact_shadow"
                        ),
                    },
                ],
            )
        ]
    )
