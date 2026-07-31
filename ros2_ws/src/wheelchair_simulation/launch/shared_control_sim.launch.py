import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("wheelchair_simulation")
    navigation_share = get_package_share_directory("wheelchair_navigation")
    control_share = get_package_share_directory("wheelchair_shared_control")
    keyboard_operator = Node(
        package="wheelchair_simulation",
        executable="sim_operator_intent",
        name="sim_operator_intent",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("operator_mode"), "' == 'keyboard'"]
            )
        ),
    )
    scenario_runner = Node(
        package="wheelchair_simulation",
        executable="sim_scenario_runner",
        name="sim_scenario_runner",
        output="screen",
        parameters=[
            {
                "scenario": LaunchConfiguration("scenario"),
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("operator_mode"), "' == 'scenario'"]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("move_dummy", default_value="false"),
            DeclareLaunchArgument("enable_sim_motion", default_value="false"),
            DeclareLaunchArgument(
                "operator_mode",
                default_value="none",
                description="none, keyboard, or scenario",
            ),
            DeclareLaunchArgument("scenario", default_value="all"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        simulation_share, "launch", "mapping_sim.launch.py"
                    )
                ),
                launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
            ),
            Node(
                package="wheelchair_navigation",
                executable="local_costmap",
                name="local_costmap",
                output="screen",
                parameters=[
                    os.path.join(
                        navigation_share, "config", "local_mapping.yaml"
                    ),
                    {
                        "use_sim_time": False,
                        "validate_cloud_timestamps": False,
                        "restamp_output_with_node_time": True,
                        "self_filter_boxes": [
                            -0.55,
                            0.55,
                            -0.40,
                            0.40,
                            0.0,
                            0.90,
                        ],
                    },
                ],
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
                    "use_sim_time": "false",
                    "max_map_age_s": "1.0",
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
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="wheelchair_simulation",
                executable="moving_dummy",
                name="moving_dummy",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(LaunchConfiguration("move_dummy")),
            ),
            keyboard_operator,
            scenario_runner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=keyboard_operator,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=scenario_runner,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ),
        ]
    )
