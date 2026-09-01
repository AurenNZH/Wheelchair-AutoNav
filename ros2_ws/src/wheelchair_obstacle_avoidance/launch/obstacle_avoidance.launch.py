import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory(
        "wheelchair_obstacle_avoidance"
    )
    navigation_share = get_package_share_directory("wheelchair_navigation")
    shared_control_share = get_package_share_directory(
        "wheelchair_shared_control"
    )
    planner_parameters = os.path.join(
        package_share, "config", "local_planner.yaml"
    )
    configured_planner_parameters = RewrittenYaml(
        source_file=planner_parameters,
        param_rewrites={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "max_planning_time_ms": LaunchConfiguration(
                "planner_search_budget_ms"
            ),
        },
        convert_types=True,
    )
    costmap_enabled = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("reactive_assistance_mode"),
                "' != 'disabled' or '",
                LaunchConfiguration("nav2_waypoint_mode"),
                "' != 'disabled'",
            ]
        )
    )
    waypoint_enabled = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("nav2_waypoint_mode"), "' == 'shadow'"]
        )
    )

    filters = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_share, "launch", "nav2_mapping.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_costmap": "false",
            "use_inflation": "true",
        }.items(),
        condition=costmap_enabled,
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
            "enable_motion": LaunchConfiguration("enable_motion"),
            "geometry_calibrated": LaunchConfiguration("geometry_calibrated"),
            "enable_udp": LaunchConfiguration("enable_udp"),
            "bind_address": LaunchConfiguration("bind_address"),
            "pi_address": LaunchConfiguration("pi_address"),
            "allowed_pi_address": LaunchConfiguration("allowed_pi_address"),
        }.items(),
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="nav2_planner",
        output="screen",
        parameters=[configured_planner_parameters],
        remappings=[
            ("/nav2_obstacle_points_right", "/lidar_right/points_filtered"),
            ("/nav2_obstacle_points_left", "/lidar_left/points_filtered"),
            ("/global_costmap/costmap", "/nav2_merged_costmap"),
            ("/global_costmap/costmap_raw", "/nav2_merged_costmap_raw"),
            (
                "/global_costmap/costmap_updates",
                "/nav2_merged_costmap_updates",
            ),
            (
                "/global_costmap/published_footprint",
                "/nav2_merged_costmap_footprint",
            ),
        ],
        condition=costmap_enabled,
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="local_avoidance_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                "node_names": ["nav2_planner"],
            }
        ],
        condition=costmap_enabled,
    )
    planner_client = Node(
        package="wheelchair_obstacle_avoidance",
        executable="local_avoidance_planner",
        name="local_avoidance_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "maximum_assist": ParameterValue(
                    LaunchConfiguration("maximum_assist"), value_type=float
                ),
                "discard_after_ms": ParameterValue(
                    LaunchConfiguration("discard_after_ms"), value_type=float
                ),
                "planner_search_budget_ms": ParameterValue(
                    LaunchConfiguration("planner_search_budget_ms"),
                    value_type=float,
                ),
                "planning_rate_hz": ParameterValue(
                    LaunchConfiguration("nav2_waypoint_rate_hz"),
                    value_type=float,
                ),
                "suggestion_topic": (
                    "/shared_control/nav2_waypoint_suggestion"
                ),
            }
        ],
        condition=waypoint_enabled,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "reactive_assistance_mode",
                default_value="disabled",
                description="disabled, shadow, or enforce",
                choices=["disabled", "shadow", "enforce"],
            ),
            DeclareLaunchArgument(
                "nav2_waypoint_mode",
                default_value="shadow",
                description="disabled or shadow; waypoint enforcement is forbidden",
                choices=["disabled", "shadow"],
            ),
            DeclareLaunchArgument(
                "nav2_waypoint_rate_hz",
                default_value="2.0",
                description="Research-only Nav2 waypoint comparison rate",
            ),
            DeclareLaunchArgument("maximum_assist", default_value="0.15"),
            DeclareLaunchArgument(
                "planner_search_budget_ms",
                default_value="30.0",
                description=(
                    "Smac internal search budget; also labels abort "
                    "diagnostics"
                ),
            ),
            DeclareLaunchArgument(
                "discard_after_ms",
                default_value="300.0",
                description=(
                    "Discard planner results older than this many milliseconds"
                ),
            ),
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
                    "Reactive assistance=",
                    LaunchConfiguration("reactive_assistance_mode"),
                    "; Nav2 waypoint assistance=",
                    LaunchConfiguration("nav2_waypoint_mode"),
                    " at ",
                    LaunchConfiguration("nav2_waypoint_rate_hz"),
                    " Hz (shadow only).",
                ]
            ),
            filters,
            planner_server,
            lifecycle_manager,
            planner_client,
            shared_control,
        ]
    )
