import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = get_package_share_directory("wheelchair_navigation")
    shared_control_share = get_package_share_directory(
        "wheelchair_shared_control"
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "nav2_mapping.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_costmap": "true",
            "use_inflation": "true",
        }.items(),
    )
    shared_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                shared_control_share, "launch", "shared_control.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "reactive_assistance_mode": LaunchConfiguration(
                "reactive_assistance_mode"
            ),
            "maximum_steering_assist": LaunchConfiguration(
                "maximum_assist"
            ),
            "turn_clearance_radius_m": LaunchConfiguration(
                "turn_clearance_radius_m"
            ),
            "max_intent_age_s": LaunchConfiguration("max_intent_age_s"),
            "enable_motion": LaunchConfiguration("enable_motion"),
            "geometry_calibrated": LaunchConfiguration("geometry_calibrated"),
            "enable_udp": LaunchConfiguration("enable_udp"),
            "bind_address": LaunchConfiguration("bind_address"),
            "pi_address": LaunchConfiguration("pi_address"),
            "allowed_pi_address": LaunchConfiguration("allowed_pi_address"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "reactive_assistance_mode",
                default_value="enforce",
                description="disabled, shadow, or enforce",
                choices=["disabled", "shadow", "enforce"],
            ),
            DeclareLaunchArgument("maximum_assist", default_value="0.30"),
            DeclareLaunchArgument(
                "turn_clearance_radius_m", default_value="0.45"
            ),
            DeclareLaunchArgument("max_intent_age_s", default_value="1.00"),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument(
                "geometry_calibrated", default_value="false"
            ),
            DeclareLaunchArgument("enable_udp", default_value="false"),
            DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
            DeclareLaunchArgument("pi_address", default_value=""),
            DeclareLaunchArgument("allowed_pi_address", default_value=""),
            LogInfo(
                msg=[
                    "Reactive obstacle assistance=",
                    LaunchConfiguration("reactive_assistance_mode"),
                    "; turn disc=",
                    LaunchConfiguration("turn_clearance_radius_m"),
                    " m; intent timeout=",
                    LaunchConfiguration("max_intent_age_s"),
                    " s",
                    "; standalone local costmap with inflation enabled; "
                    "motion gate=",
                    LaunchConfiguration("enable_motion"),
                    "; UDP bridge=",
                    LaunchConfiguration("enable_udp"),
                    ".",
                ]
            ),
            mapping,
            shared_control,
        ]
    )
