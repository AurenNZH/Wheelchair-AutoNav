import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wheelchair_navigation")
    parameters = os.path.join(
        package_share, "config", "nav2_front_costmap.yaml"
    )
    artifact_parameters = os.path.join(
        package_share, "config", "local_mapping.yaml"
    )
    rviz_config = os.path.join(
        package_share, "rviz", "nav2_front_costmap.rviz"
    )
    configured_parameters = RewrittenYaml(
        source_file=parameters,
        param_rewrites={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "inflation_layer.enabled": LaunchConfiguration(
                "use_inflation"
            ),
            "inflation_layer.inflation_radius": LaunchConfiguration(
                "inflation_radius"
            ),
            "inflation_layer.cost_scaling_factor": LaunchConfiguration(
                "cost_scaling_factor"
            ),
        },
        convert_types=True,
    )

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        # Rewrite the exact node section. Foxy gives it precedence over the
        # wildcard parameter files produced by a launch dictionary.
        parameters=[configured_parameters],
        remappings=[
            (
                "/nav2_obstacle_points",
                LaunchConfiguration("nav2_points_topic"),
            ),
            ("costmap", "/nav2_front_costmap"),
            ("costmap_raw", "/nav2_front_costmap_raw"),
            ("costmap_updates", "/nav2_front_costmap_updates"),
            ("published_footprint", "/nav2_front_costmap_footprint"),
        ],
    )
    artifact_filter = Node(
        package="wheelchair_navigation",
        executable="artifact_point_filter",
        name="artifact_point_filter",
        output="screen",
        parameters=[
            artifact_parameters,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "validate_cloud_timestamps": LaunchConfiguration(
                    "validate_cloud_timestamps"
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_artifact_filter")),
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav2_front_costmap_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "autostart": True,
                # Foxy's standalone Costmap2DROS creates this fixed fully
                # qualified lifecycle-node name internally.
                "node_names": ["/costmap/costmap"],
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_costmap_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "use_artifact_filter", default_value="true"
            ),
            DeclareLaunchArgument(
                "validate_cloud_timestamps", default_value="true"
            ),
            DeclareLaunchArgument("use_inflation", default_value="false"),
            DeclareLaunchArgument("inflation_radius", default_value="0.55"),
            DeclareLaunchArgument(
                "cost_scaling_factor", default_value="3.0"
            ),
            SetLaunchConfiguration(
                "nav2_points_topic",
                "/rslidar_points_artifact_filtered",
                condition=IfCondition(
                    LaunchConfiguration("use_artifact_filter")
                ),
            ),
            SetLaunchConfiguration(
                "nav2_points_topic",
                "/rslidar_points",
                condition=UnlessCondition(
                    LaunchConfiguration("use_artifact_filter")
                ),
            ),
            LogInfo(
                msg=[
                    "Stock Foxy Nav2 observation only: input=",
                    LaunchConfiguration("nav2_points_topic"),
                    "; inflation=",
                    LaunchConfiguration("use_inflation"),
                    " (radius=",
                    LaunchConfiguration("inflation_radius"),
                    " m, scaling=",
                    LaunchConfiguration("cost_scaling_factor"),
                    "); publishing /nav2_front_costmap; /front_costmap and "
                    "physical shared control are not connected.",
                ]
            ),
            artifact_filter,
            costmap,
            lifecycle_manager,
            rviz,
        ]
    )
