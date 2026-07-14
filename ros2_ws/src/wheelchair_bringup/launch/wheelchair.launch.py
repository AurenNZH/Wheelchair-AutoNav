import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wheelchair_bringup.defaults import CAMERA_LIDAR_TRANSFORM, LAUNCH_DEFAULTS


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
    navigation_launch = os.path.join(
        get_package_share_directory("wheelchair_navigation"),
        "launch",
        "local_navigation.launch.py",
    )
    rviz_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "rviz",
        "wheelchair.rviz",
    )

    declarations = [
        _argument("use_lidar", "Start the RoboSense AIRY driver."),
        _argument("use_camera", "Start the RealSense L515 driver."),
        _argument("use_navigation", "Start non-actuating local navigation."),
        _argument("use_rviz", "Start RViz with the wheelchair view."),
        _argument("publish_camera_tf", "Publish rslidar -> camera_link."),
        _argument(
            "publish_base_lidar_tf",
            "Publish the measured base_link -> rslidar transform.",
        ),
        _argument("base_lidar_x", "Measured LiDAR X translation in metres."),
        _argument("base_lidar_y", "Measured LiDAR Y translation in metres."),
        _argument("base_lidar_z", "Measured LiDAR Z translation in metres."),
        _argument("base_lidar_yaw", "Measured LiDAR yaw in radians."),
        _argument("base_lidar_pitch", "Measured LiDAR pitch in radians."),
        _argument("base_lidar_roll", "Measured LiDAR roll in radians."),
    ]

    lidar = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": ""}],
        condition=IfCondition(LaunchConfiguration("use_lidar")),
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
            CAMERA_LIDAR_TRANSFORM["x"],
            CAMERA_LIDAR_TRANSFORM["y"],
            CAMERA_LIDAR_TRANSFORM["z"],
            CAMERA_LIDAR_TRANSFORM["yaw"],
            CAMERA_LIDAR_TRANSFORM["pitch"],
            CAMERA_LIDAR_TRANSFORM["roll"],
            CAMERA_LIDAR_TRANSFORM["parent"],
            CAMERA_LIDAR_TRANSFORM["child"],
        ],
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
    )
    base_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_mount_tf",
        arguments=[
            LaunchConfiguration("base_lidar_x"),
            LaunchConfiguration("base_lidar_y"),
            LaunchConfiguration("base_lidar_z"),
            LaunchConfiguration("base_lidar_yaw"),
            LaunchConfiguration("base_lidar_pitch"),
            LaunchConfiguration("base_lidar_roll"),
            "base_link",
            "rslidar",
        ],
        condition=IfCondition(LaunchConfiguration("publish_base_lidar_tf")),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        condition=IfCondition(LaunchConfiguration("use_navigation")),
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
        + [lidar, camera, camera_tf, base_lidar_tf, navigation, rviz]
    )
