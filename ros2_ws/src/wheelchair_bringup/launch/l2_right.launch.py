"""Bring up the wheelchair's right-hand Unitree L2 sensor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _argument(name, default, description):
    return DeclareLaunchArgument(
        name, default_value=str(default), description=description
    )


def generate_launch_description():
    share = get_package_share_directory("wheelchair_bringup")
    default_config = os.path.join(share, "config", "unitree_l2_right.yaml")
    rviz_config = os.path.join(share, "rviz", "wheelchair.rviz")

    declarations = [
        _argument("config_file", default_config, "Right-L2 ROS parameter file"),
        _argument("publish_mount_tf", "true", "Publish base_link to right-L2 TF"),
        _argument("mount_x", "0.330", "Right-L2 X translation in metres"),
        _argument("mount_y", "-0.265", "Right-L2 Y translation in metres"),
        _argument("mount_z", "0.320", "Right-L2 Z translation in metres"),
        _argument("mount_yaw", "0.392699082", "Right-L2 yaw in radians"),
        _argument("mount_pitch", "0.0", "Right-L2 pitch in radians"),
        _argument("mount_roll", "0.0", "Right-L2 roll in radians"),
        _argument("use_rviz", "true", "Start RViz with the wheelchair view"),
        _argument("use_sim_time", "false", "Use the ROS simulation clock"),
    ]

    lidar = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_l2_right",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        # Preserve the vendor's IMU-derived TF for diagnostics without giving
        # lidar_right_link two parents in the wheelchair TF tree.
        remappings=[("/tf", "/lidar_right/vendor_tf")],
    )
    mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_right_mount_tf",
        arguments=[
            LaunchConfiguration("mount_x"),
            LaunchConfiguration("mount_y"),
            LaunchConfiguration("mount_z"),
            LaunchConfiguration("mount_yaw"),
            LaunchConfiguration("mount_pitch"),
            LaunchConfiguration("mount_roll"),
            "base_link",
            "lidar_right_link",
        ],
        condition=IfCondition(LaunchConfiguration("publish_mount_tf")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="wheelchair_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription(declarations + [lidar, mount_tf, rviz])
