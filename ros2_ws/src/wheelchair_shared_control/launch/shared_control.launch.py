import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wheelchair_shared_control")
    parameters = os.path.join(share, "config", "shared_control.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument("geometry_calibrated", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("max_map_age_s", default_value="0.30"),
            Node(
                package="wheelchair_shared_control",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
                parameters=[
                    parameters,
                    {
                        "enable_motion": LaunchConfiguration("enable_motion"),
                        "geometry_calibrated": LaunchConfiguration(
                            "geometry_calibrated"
                        ),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "max_map_age_s": LaunchConfiguration(
                            "max_map_age_s"
                        ),
                    },
                ],
            ),
            Node(
                package="wheelchair_shared_control",
                executable="udp_bridge",
                name="shared_control_udp_bridge",
                output="screen",
                parameters=[
                    parameters,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
