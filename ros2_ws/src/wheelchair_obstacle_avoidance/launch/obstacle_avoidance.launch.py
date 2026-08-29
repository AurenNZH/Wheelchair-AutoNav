import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
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
        param_rewrites={"use_sim_time": LaunchConfiguration("use_sim_time")},
        convert_types=True,
    )
    enabled = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration("avoidance_mode"), "' != 'disabled'"]
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
        condition=enabled,
    )
    shared_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                shared_control_share, "launch", "shared_control.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "avoidance_mode": LaunchConfiguration("avoidance_mode"),
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
            ("global_costmap/costmap", "/nav2_merged_costmap"),
            ("global_costmap/costmap_raw", "/nav2_merged_costmap_raw"),
            ("global_costmap/costmap_updates", "/nav2_merged_costmap_updates"),
            (
                "global_costmap/published_footprint",
                "/nav2_merged_costmap_footprint",
            ),
        ],
        condition=enabled,
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
        condition=enabled,
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
            }
        ],
        condition=enabled,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "avoidance_mode",
                default_value="disabled",
                description="disabled, shadow, or enforce",
            ),
            DeclareLaunchArgument("maximum_assist", default_value="0.15"),
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
                    "Local obstacle avoidance mode=",
                    LaunchConfiguration("avoidance_mode"),
                    ". Motion remains gated by shared control.",
                ]
            ),
            filters,
            planner_server,
            lifecycle_manager,
            planner_client,
            shared_control,
        ]
    )
