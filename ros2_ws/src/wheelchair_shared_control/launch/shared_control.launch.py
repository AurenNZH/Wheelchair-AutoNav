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
            DeclareLaunchArgument(
                "front_costmap_topic",
                default_value="/nav2_front_costmap",
            ),
            DeclareLaunchArgument(
                "source_header_topic",
                default_value="/artifact_filter/source_header",
            ),
            DeclareLaunchArgument(
                "freshness_mode", default_value="nav2_live"
            ),
            DeclareLaunchArgument("max_map_age_s", default_value="0.50"),
            DeclareLaunchArgument("max_source_age_s", default_value="0.50"),
            DeclareLaunchArgument(
                "max_future_source_offset_s", default_value="0.10"
            ),
            DeclareLaunchArgument("slow_cost_threshold", default_value="1"),
            DeclareLaunchArgument("stop_cost_threshold", default_value="99"),
            DeclareLaunchArgument("slow_forward_limit", default_value="0.35"),
            DeclareLaunchArgument("min_steering", default_value="-0.466307658"),
            DeclareLaunchArgument("max_steering", default_value="0.466307658"),
            DeclareLaunchArgument("enable_udp", default_value="false"),
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("pi_address", default_value=""),
            DeclareLaunchArgument("allowed_pi_address", default_value=""),
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
                        "front_costmap_topic": LaunchConfiguration(
                            "front_costmap_topic"
                        ),
                        "source_header_topic": LaunchConfiguration(
                            "source_header_topic"
                        ),
                        "freshness_mode": LaunchConfiguration(
                            "freshness_mode"
                        ),
                        "max_map_age_s": LaunchConfiguration(
                            "max_map_age_s"
                        ),
                        "max_source_age_s": LaunchConfiguration(
                            "max_source_age_s"
                        ),
                        "max_future_source_offset_s": LaunchConfiguration(
                            "max_future_source_offset_s"
                        ),
                        "slow_forward_limit": LaunchConfiguration(
                            "slow_forward_limit"
                        ),
                        "min_steering": LaunchConfiguration("min_steering"),
                        "max_steering": LaunchConfiguration("max_steering"),
                        "slow_cost_threshold": LaunchConfiguration(
                            "slow_cost_threshold"
                        ),
                        "stop_cost_threshold": LaunchConfiguration(
                            "stop_cost_threshold"
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
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "enable_udp": LaunchConfiguration("enable_udp"),
                        "bind_address": LaunchConfiguration("bind_address"),
                        "pi_address": LaunchConfiguration("pi_address"),
                        "allowed_pi_address": LaunchConfiguration(
                            "allowed_pi_address"
                        ),
                    },
                ],
            ),
        ]
    )
