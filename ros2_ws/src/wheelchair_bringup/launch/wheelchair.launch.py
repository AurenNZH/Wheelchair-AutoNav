import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wheelchair_bringup.defaults import LAUNCH_DEFAULTS


def _argument(name: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        name,
        default_value=LAUNCH_DEFAULTS[name],
        description=description,
    )


def generate_launch_description():
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )
    lidar_launch = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "launch",
        "l2_right.launch.py",
    )
    mapping_launch = os.path.join(
        get_package_share_directory("wheelchair_navigation"),
        "launch",
        "nav2_mapping.launch.py",
    )
    rviz_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "rviz",
        "wheelchair.rviz",
    )
    declarations = [
        _argument("use_lidar", "Start the right Unitree L2 driver."),
        _argument("use_camera", "Start the RealSense L515 driver."),
        _argument("use_mapping", "Start non-actuating Nav2 local mapping."),
        _argument("use_rviz", "Start RViz with the wheelchair view."),
        _argument("use_sim_time", "Use the ROS simulation clock."),
        _argument(
            "support_min_points_per_cell",
            "Minimum L2 points required in each 0.1 m obstacle cell.",
        ),
        _argument("use_inflation", "Enable weighted Nav2 inflation costs."),
        _argument("inflation_radius", "Nav2 inflation radius in metres."),
        _argument("cost_scaling_factor", "Nav2 inflation cost scaling factor."),
        _argument("publish_camera_tf", "Publish base_link -> camera_link."),
        _argument("use_robot_model", "Publish the wheelchair URDF model."),
        _argument("publish_lidar_tf", "Publish the measured right-L2 transform."),
        _argument("lidar_x", "Measured right-L2 X translation in metres."),
        _argument("lidar_y", "Measured right-L2 Y translation in metres."),
        _argument("lidar_z", "Measured right-L2 Z translation in metres."),
        _argument("lidar_yaw", "Measured right-L2 yaw in radians."),
        _argument("lidar_pitch", "Measured right-L2 pitch in radians."),
        _argument("lidar_roll", "Measured right-L2 roll in radians."),
        _argument(
            "publish_left_lidar_tf",
            "Publish the defined left-L2 mount transform.",
        ),
        _argument("lidar_left_x", "Measured left-L2 X translation in metres."),
        _argument("lidar_left_y", "Measured left-L2 Y translation in metres."),
        _argument("lidar_left_z", "Measured left-L2 Z translation in metres."),
        _argument("lidar_left_yaw", "Measured left-L2 yaw in radians."),
        _argument("lidar_left_pitch", "Measured left-L2 pitch in radians."),
        _argument("lidar_left_roll", "Measured left-L2 roll in radians."),
        _argument("base_camera_x", "Measured camera X translation in metres."),
        _argument("base_camera_y", "Measured camera Y translation in metres."),
        _argument("base_camera_z", "Measured camera Z translation in metres."),
        _argument("base_camera_yaw", "Measured camera yaw in radians."),
        _argument("base_camera_pitch", "Measured camera pitch in radians."),
        _argument("base_camera_roll", "Measured camera roll in radians."),
    ]

    lidar = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    "publish_mount_tf": LaunchConfiguration(
                        "publish_lidar_tf"
                    ),
                    "mount_x": LaunchConfiguration("lidar_x"),
                    "mount_y": LaunchConfiguration("lidar_y"),
                    "mount_z": LaunchConfiguration("lidar_z"),
                    "mount_yaw": LaunchConfiguration("lidar_yaw"),
                    "mount_pitch": LaunchConfiguration("lidar_pitch"),
                    "mount_roll": LaunchConfiguration("lidar_roll"),
                    "publish_left_mount_tf": LaunchConfiguration(
                        "publish_left_lidar_tf"
                    ),
                    "left_mount_x": LaunchConfiguration("lidar_left_x"),
                    "left_mount_y": LaunchConfiguration("lidar_left_y"),
                    "left_mount_z": LaunchConfiguration("lidar_left_z"),
                    "left_mount_yaw": LaunchConfiguration("lidar_left_yaw"),
                    "left_mount_pitch": LaunchConfiguration(
                        "lidar_left_pitch"
                    ),
                    "left_mount_roll": LaunchConfiguration("lidar_left_roll"),
                    "use_robot_model": LaunchConfiguration("use_robot_model"),
                    "use_rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_lidar")),
            )
        ],
        scoped=True,
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            "enable_depth": "true",
            "enable_color": "true",
            "enable_sync": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "true",
            "depth_module.profile": "640x480x30",
            "rgb_camera.profile": "640x480x30",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        arguments=[
            LaunchConfiguration("base_camera_x"),
            LaunchConfiguration("base_camera_y"),
            LaunchConfiguration("base_camera_z"),
            LaunchConfiguration("base_camera_yaw"),
            LaunchConfiguration("base_camera_pitch"),
            LaunchConfiguration("base_camera_roll"),
            "base_link",
            "camera_link",
        ],
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
    )
    mapping = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_rviz": "false",
                    "support_min_points_per_cell": LaunchConfiguration(
                        "support_min_points_per_cell"
                    ),
                    "use_inflation": LaunchConfiguration("use_inflation"),
                    "inflation_radius": LaunchConfiguration(
                        "inflation_radius"
                    ),
                    "cost_scaling_factor": LaunchConfiguration(
                        "cost_scaling_factor"
                    ),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_mapping")),
            )
        ],
        scoped=True,
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        declarations
        + [lidar, camera, camera_tf, mapping, rviz]
    )
