import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _support_filter(side, label):
    topic_root = "/lidar_%s" % side
    return Node(
        package="wheelchair_navigation",
        executable="point_support_filter",
        name="l2_lidar_%s_support_filter" % side,
        output="screen",
        parameters=[
            {
                "sensor_label": "Unitree L2 %s" % label,
                "lidar_topic": topic_root + "/points",
                "filtered_cloud_topic": topic_root + "/points_filtered",
                "source_header_topic": topic_root + "/filter/source_header",
                "low_support_points_topic": topic_root + "/low_support_points",
                "diagnostic_name": (
                    "wheelchair_navigation/point_support_filter/lidar_%s"
                    % side
                ),
                "target_frame": "base_link",
                "size_m": 8.0,
                "resolution_m": 0.1,
                "min_height_m": 0.05,
                "max_height_m": 1.5,
                "min_range_m": 0.45,
                "max_range_m": 4.0,
                "front_length_m": 4.0,
                "front_width_m": 8.0,
                "front_resolution_m": 0.1,
                "front_fov_deg": 180.0,
                "min_points_per_cell": ParameterValue(
                    LaunchConfiguration("support_min_points_per_cell"),
                    value_type=int,
                ),
                "validate_cloud_timestamps": LaunchConfiguration(
                    "validate_cloud_timestamps"
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )


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

    support_filter_right = _support_filter("right", "right")
    support_filter_left = _support_filter("left", "left")

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        output="screen",
        # Rewrite the exact node section. Foxy gives it precedence over the
        # wildcard parameter files produced by a launch dictionary.
        parameters=[configured_parameters],
        remappings=[
            (
                "/nav2_obstacle_points_right",
                "/lidar_right/points_filtered",
            ),
            (
                "/nav2_obstacle_points_left",
                "/lidar_left/points_filtered",
            ),
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
            DeclareLaunchArgument(
                "validate_cloud_timestamps", default_value="true"
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "support_min_points_per_cell",
                default_value="3",
                description=(
                    "Minimum points in each 0.1 m Nav2 cell; set 1 for "
                    "a support-filter pass-through comparison"
                ),
            ),
            DeclareLaunchArgument("use_inflation", default_value="false"),
            DeclareLaunchArgument("inflation_radius", default_value="0.55"),
            DeclareLaunchArgument(
                "cost_scaling_factor", default_value="3.0"
            ),
            LogInfo(
                msg=[
                    "Stock Foxy Nav2 observation only: "
                    "L2_lidar_right=/lidar_right/points_filtered, ",
                    "L2_lidar_left=/lidar_left/points_filtered",
                    " (support points/cell=",
                    LaunchConfiguration("support_min_points_per_cell"),
                    ")",
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
            support_filter_right,
            support_filter_left,
            costmap,
            lifecycle_manager,
            rviz,
        ]
    )
