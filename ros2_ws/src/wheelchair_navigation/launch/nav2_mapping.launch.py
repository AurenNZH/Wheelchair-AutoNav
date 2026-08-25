import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wheelchair_navigation")
    parameters = os.path.join(
        package_share, "config", "nav2_front_costmap.yaml"
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
            ("/nav2_obstacle_points_right", "/lidar_right/points"),
            ("costmap", "/nav2_front_costmap"),
            ("costmap_raw", "/nav2_front_costmap_raw"),
            ("costmap_updates", "/nav2_front_costmap_updates"),
            ("published_footprint", "/nav2_front_costmap_footprint"),
        ],
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
            DeclareLaunchArgument("use_inflation", default_value="false"),
            DeclareLaunchArgument("inflation_radius", default_value="0.55"),
            DeclareLaunchArgument(
                "cost_scaling_factor", default_value="3.0"
            ),
            LogInfo(
                msg=[
                    "Stock Foxy Nav2 observation only: "
                    "L2_lidar_right=/lidar_right/points",
                    "; inflation=",
                    LaunchConfiguration("use_inflation"),
                    " (radius=",
                    LaunchConfiguration("inflation_radius"),
                    " m, scaling=",
                    LaunchConfiguration("cost_scaling_factor"),
                    "); publishing /nav2_front_costmap; no supervisor or "
                    "physical command process is launched.",
                ]
            ),
            costmap,
            lifecycle_manager,
            rviz,
        ]
    )
