"""Bring up one Ethernet-connected Unitree L2 on the right footrest."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _argument(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=str(default),
        description=description,
    )


def generate_launch_description():
    arguments = [
        _argument("lidar_ip", "192.168.1.62", "Unitree L2 IPv4 address"),
        _argument("lidar_port", "6101", "Unitree L2 UDP port"),
        _argument("local_ip", "192.168.1.2", "Jetson L2 receive IPv4 address"),
        _argument("local_port", "6201", "Jetson UDP receive port"),
        _argument("cloud_topic", "/lidar_right/points", "Published PointCloud2 topic"),
        _argument("cloud_frame", "lidar_right_link", "Point-cloud frame ID"),
        _argument("imu_topic", "/lidar_right/imu", "Published IMU topic"),
        _argument("imu_frame", "lidar_right_imu_link", "IMU frame ID"),
        _argument("cloud_scan_num", "18", "Number of scans accumulated per cloud"),
        _argument("range_min", "0.0", "Minimum accepted range in metres"),
        _argument("range_max", "100.0", "Maximum accepted range in metres"),
        _argument("use_rviz", "true", "Start RViz with the single-L2 view"),
    ]

    lidar = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_l2_right",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "initialize_type": 2,
                "work_mode": 0,
                "use_system_timestamp": True,
                "lidar_ip": LaunchConfiguration("lidar_ip"),
                "lidar_port": ParameterValue(
                    LaunchConfiguration("lidar_port"), value_type=int
                ),
                "local_ip": LaunchConfiguration("local_ip"),
                "local_port": ParameterValue(
                    LaunchConfiguration("local_port"), value_type=int
                ),
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "cloud_frame": LaunchConfiguration("cloud_frame"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "imu_frame": LaunchConfiguration("imu_frame"),
                "cloud_scan_num": ParameterValue(
                    LaunchConfiguration("cloud_scan_num"), value_type=int
                ),
                "range_min": ParameterValue(
                    LaunchConfiguration("range_min"), value_type=float
                ),
                "range_max": ParameterValue(
                    LaunchConfiguration("range_max"), value_type=float
                ),
            }
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("wheelchair_unitree_l2"),
        "rviz",
        "single_l2.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="unitree_l2_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(arguments + [lidar, rviz])
