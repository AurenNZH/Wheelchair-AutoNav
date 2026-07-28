import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("wheelchair_simulation")
    navigation_share = get_package_share_directory("wheelchair_navigation")
    control_share = get_package_share_directory("wheelchair_shared_control")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("move_dummy", default_value="false"),
            DeclareLaunchArgument("enable_sim_motion", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        simulation_share, "launch", "mapping_sim.launch.py"
                    )
                ),
                launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        navigation_share, "launch", "local_mapping.launch.py"
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        control_share, "launch", "shared_control.launch.py"
                    )
                ),
                launch_arguments={
                    "enable_motion": "true",
                    "geometry_calibrated": "true",
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(
                package="wheelchair_simulation",
                executable="safe_cmd_adapter",
                name="sim_safe_cmd_adapter",
                output="screen",
                parameters=[
                    {
                        "enable_sim_motion": LaunchConfiguration(
                            "enable_sim_motion"
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
            Node(
                package="wheelchair_simulation",
                executable="moving_dummy",
                name="moving_dummy",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("move_dummy")),
            ),
        ]
    )
