import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
                "merged_costmap_topic",
                default_value="/nav2_merged_costmap",
            ),
            DeclareLaunchArgument(
                "source_header_topic",
                default_value="/lidar_right/filter/source_header",
            ),
            DeclareLaunchArgument(
                "left_source_header_topic",
                default_value="/lidar_left/filter/source_header",
            ),
            DeclareLaunchArgument(
                "freshness_mode", default_value="nav2_live"
            ),
            DeclareLaunchArgument("max_intent_age_s", default_value="1.00"),
            DeclareLaunchArgument("max_map_age_s", default_value="0.50"),
            DeclareLaunchArgument("max_source_age_s", default_value="0.50"),
            DeclareLaunchArgument(
                "max_future_source_offset_s", default_value="0.10"
            ),
            DeclareLaunchArgument("slow_cost_threshold", default_value="1"),
            DeclareLaunchArgument("stop_cost_threshold", default_value="99"),
            DeclareLaunchArgument(
                "reactive_assistance_mode",
                default_value="disabled",
                choices=["disabled", "shadow", "enforce"],
            ),
            DeclareLaunchArgument("reactive_horizon_m", default_value="1.20"),
            DeclareLaunchArgument(
                "reactive_path_sample_step_m", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "reactive_steering_step", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "reactive_minimum_correction", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "reactive_minimum_cost_improvement", default_value="5"
            ),
            DeclareLaunchArgument(
                "reactive_confirmation_cycles", default_value="2"
            ),
            DeclareLaunchArgument(
                "reactive_intent_change_tolerance", default_value="0.05"
            ),
            DeclareLaunchArgument(
                "maximum_steering_assist", default_value="0.577350269"
            ),
            DeclareLaunchArgument("slow_forward_limit", default_value="0.60"),
            DeclareLaunchArgument("reverse_limit", default_value="0.65"),
            DeclareLaunchArgument(
                "turn_clearance_radius_m", default_value="0.45"
            ),
            DeclareLaunchArgument("clear_turn_limit", default_value="0.90"),
            DeclareLaunchArgument("slow_turn_limit", default_value="0.60"),
            DeclareLaunchArgument(
                "turn_longitudinal_limit", default_value="0.15"
            ),
            DeclareLaunchArgument("min_steering", default_value="-0.577350269"),
            DeclareLaunchArgument("max_steering", default_value="0.577350269"),
            DeclareLaunchArgument(
                "forward_cone_half_angle_deg", default_value="30.0"
            ),
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
                        "merged_costmap_topic": LaunchConfiguration(
                            "merged_costmap_topic"
                        ),
                        "source_header_topic": LaunchConfiguration(
                            "source_header_topic"
                        ),
                        "left_source_header_topic": LaunchConfiguration(
                            "left_source_header_topic"
                        ),
                        "freshness_mode": LaunchConfiguration(
                            "freshness_mode"
                        ),
                        "max_intent_age_s": LaunchConfiguration(
                            "max_intent_age_s"
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
                        "reverse_limit": LaunchConfiguration("reverse_limit"),
                        "turn_clearance_radius_m": LaunchConfiguration(
                            "turn_clearance_radius_m"
                        ),
                        "clear_turn_limit": LaunchConfiguration(
                            "clear_turn_limit"
                        ),
                        "slow_turn_limit": LaunchConfiguration(
                            "slow_turn_limit"
                        ),
                        "turn_longitudinal_limit": LaunchConfiguration(
                            "turn_longitudinal_limit"
                        ),
                        "min_steering": LaunchConfiguration("min_steering"),
                        "max_steering": LaunchConfiguration("max_steering"),
                        "forward_cone_half_angle_deg": LaunchConfiguration(
                            "forward_cone_half_angle_deg"
                        ),
                        "slow_cost_threshold": LaunchConfiguration(
                            "slow_cost_threshold"
                        ),
                        "stop_cost_threshold": LaunchConfiguration(
                            "stop_cost_threshold"
                        ),
                        "reactive_assistance_mode": LaunchConfiguration(
                            "reactive_assistance_mode"
                        ),
                        "reactive_horizon_m": LaunchConfiguration(
                            "reactive_horizon_m"
                        ),
                        "reactive_path_sample_step_m": LaunchConfiguration(
                            "reactive_path_sample_step_m"
                        ),
                        "reactive_steering_step": LaunchConfiguration(
                            "reactive_steering_step"
                        ),
                        "reactive_minimum_correction": LaunchConfiguration(
                            "reactive_minimum_correction"
                        ),
                        "reactive_minimum_cost_improvement": (
                            LaunchConfiguration(
                                "reactive_minimum_cost_improvement"
                            )
                        ),
                        "reactive_confirmation_cycles": LaunchConfiguration(
                            "reactive_confirmation_cycles"
                        ),
                        "reactive_intent_change_tolerance": (
                            LaunchConfiguration(
                                "reactive_intent_change_tolerance"
                            )
                        ),
                        "maximum_steering_assist": LaunchConfiguration(
                            "maximum_steering_assist"
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
                condition=IfCondition(LaunchConfiguration("enable_udp")),
            ),
        ]
    )
