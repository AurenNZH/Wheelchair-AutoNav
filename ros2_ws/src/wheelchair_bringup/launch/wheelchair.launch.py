import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    mapping_launch = os.path.join(
        get_package_share_directory("wheelchair_navigation"),
        "launch",
        "local_mapping.launch.py",
    )
    rviz_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "rviz",
        "wheelchair.rviz",
    )
    safety_rviz_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "rviz",
        "wheelchair_safety.rviz",
    )
    lidar_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "config",
        "rslidar_airy.yaml",
    )

    declarations = [
        _argument("use_lidar", "Start the RoboSense AIRY driver."),
        _argument("use_camera", "Start the RealSense L515 driver."),
        _argument("use_mapping", "Start non-actuating LiDAR local mapping."),
        _argument("use_rviz", "Start RViz with the wheelchair view."),
        DeclareLaunchArgument(
            "runtime_profile",
            default_value=LAUNCH_DEFAULTS["runtime_profile"],
            description="Select latency-safe or full artifact diagnostics.",
            choices=["safety", "artifact_debug"],
        ),
        _argument("publish_camera_tf", "Publish base_link -> camera_link."),
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
        _argument("base_camera_x", "Measured camera X translation in metres."),
        _argument("base_camera_y", "Measured camera Y translation in metres."),
        _argument("base_camera_z", "Measured camera Z translation in metres."),
        _argument("base_camera_yaw", "Measured camera yaw in radians."),
        _argument("base_camera_pitch", "Measured camera pitch in radians."),
        _argument("base_camera_roll", "Measured camera roll in radians."),
    ]

    lidar = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        parameters=[{"config_path": lidar_config}],
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
    safety_profile = PythonExpression(
        ["'", LaunchConfiguration("runtime_profile"), "' == 'safety'"]
    )
    debug_profile = PythonExpression(
        [
            "'",
            LaunchConfiguration("runtime_profile"),
            "' == 'artifact_debug'",
        ]
    )
    mapping_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            "publish_local_obstacles": "false",
            "publish_artifact_shadow": "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_mapping"),
                    "' == 'true' and ",
                    safety_profile,
                ]
            )
        ),
    )
    mapping_debug = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        launch_arguments={
            "publish_local_obstacles": "true",
            "publish_artifact_shadow": "true",
        }.items(),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_mapping"),
                    "' == 'true' and ",
                    debug_profile,
                ]
            )
        ),
    )
    rviz_safety = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", safety_rviz_config],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_rviz"),
                    "' == 'true' and ",
                    safety_profile,
                ]
            )
        ),
    )
    rviz_debug = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_rviz"),
                    "' == 'true' and ",
                    debug_profile,
                ]
            )
        ),
    )

    return LaunchDescription(
        declarations
        + [
            lidar,
            camera,
            camera_tf,
            base_lidar_tf,
            mapping_safety,
            mapping_debug,
            rviz_safety,
            rviz_debug,
        ]
    )
