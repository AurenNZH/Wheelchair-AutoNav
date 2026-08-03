import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wheelchair_shared_control")
    parameters = os.path.join(share, "config", "shared_control.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "command",
                default_value="released",
                description="Initial injector preset: released or forward",
            ),
            DeclareLaunchArgument(
                "forward_request",
                default_value="0.5",
                description="Normalized forward intent in (0, 1]",
            ),
            DeclareLaunchArgument(
                "motion_timeout_s",
                default_value="30.0",
                description="Wall-time lease before automatic release",
            ),
            DeclareLaunchArgument(
                "max_map_age_s",
                default_value="0.30",
                description="Supervisor timeout for restamped replay maps",
            ),
            LogInfo(
                msg=(
                    "Recorded-map decision replay only: UDP, CAN, Gazebo, and "
                    "velocity adapters are not launched."
                )
            ),
            Node(
                package="wheelchair_shared_control",
                executable="operator_intent_injector",
                name="operator_intent_injector",
                output="screen",
                parameters=[
                    {
                        "command": LaunchConfiguration("command"),
                        "forward_request": LaunchConfiguration(
                            "forward_request"
                        ),
                        "motion_timeout_s": LaunchConfiguration(
                            "motion_timeout_s"
                        ),
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="wheelchair_shared_control",
                executable="replay_map_restamper",
                name="replay_map_restamper",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="wheelchair_shared_control",
                executable="safety_supervisor",
                name="safety_supervisor",
                output="screen",
                parameters=[
                    parameters,
                    {
                        "enable_motion": True,
                        "geometry_calibrated": True,
                        "max_map_age_s": LaunchConfiguration(
                            "max_map_age_s"
                        ),
                        "use_sim_time": False,
                    },
                ],
            ),
            Node(
                package="wheelchair_shared_control",
                executable="safety_envelope_monitor",
                name="safety_envelope_monitor",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
