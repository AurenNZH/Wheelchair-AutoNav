"""Bring up the wheelchair's right-hand Unitree L2 sensor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
        _argument(
            "publish_left_mount_tf",
            "true",
            "Publish the defined base_link to left-L2 mount frame",
        ),
        _argument("left_mount_x", "0.330", "Left-L2 X translation in metres"),
        _argument("left_mount_y", "0.265", "Left-L2 Y translation in metres"),
        _argument("left_mount_z", "0.320", "Left-L2 Z translation in metres"),
        _argument("left_mount_yaw", "-0.392699082", "Left-L2 yaw in radians"),
        _argument("left_mount_pitch", "0.0", "Left-L2 pitch in radians"),
        _argument("left_mount_roll", "0.0", "Left-L2 roll in radians"),
        _argument(
            "use_robot_model",
            "true",
            "Publish and display the wheelchair URDF model",
        ),
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
    static_mount_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("publish_mount_tf"),
                "' == 'true' and '",
                LaunchConfiguration("use_robot_model"),
                "' == 'false'",
            ]
        )
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
        condition=static_mount_condition,
    )
    left_static_mount_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("publish_left_mount_tf"),
                "' == 'true' and '",
                LaunchConfiguration("use_robot_model"),
                "' == 'false'",
            ]
        )
    )
    left_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_left_mount_tf",
        arguments=[
            LaunchConfiguration("left_mount_x"),
            LaunchConfiguration("left_mount_y"),
            LaunchConfiguration("left_mount_z"),
            LaunchConfiguration("left_mount_yaw"),
            LaunchConfiguration("left_mount_pitch"),
            LaunchConfiguration("left_mount_roll"),
            "base_link",
            "lidar_left_link",
        ],
        condition=left_static_mount_condition,
    )

    wheelchair_xacro = PathJoinSubstitution(
        [
            FindPackageShare("wheelchair_simulation"),
            "urdf",
            "wheelchair.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                wheelchair_xacro,
                " lidar_right_x:=",
                LaunchConfiguration("mount_x"),
                " lidar_right_y:=",
                LaunchConfiguration("mount_y"),
                " lidar_right_z:=",
                LaunchConfiguration("mount_z"),
                " lidar_right_yaw:=",
                LaunchConfiguration("mount_yaw"),
                " lidar_right_pitch:=",
                LaunchConfiguration("mount_pitch"),
                " lidar_right_roll:=",
                LaunchConfiguration("mount_roll"),
                " lidar_left_x:=",
                LaunchConfiguration("left_mount_x"),
                " lidar_left_y:=",
                LaunchConfiguration("left_mount_y"),
                " lidar_left_z:=",
                LaunchConfiguration("left_mount_z"),
                " lidar_left_yaw:=",
                LaunchConfiguration("left_mount_yaw"),
                " lidar_left_pitch:=",
                LaunchConfiguration("left_mount_pitch"),
                " lidar_left_roll:=",
                LaunchConfiguration("left_mount_roll"),
            ]
        ),
        value_type=str,
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="wheelchair_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_robot_model")),
    )
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="wheelchair_model_joint_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_robot_model")),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="wheelchair_rviz",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription(
        declarations
        + [
            lidar,
            mount_tf,
            left_mount_tf,
            robot_state_publisher,
            joint_state_publisher,
            rviz,
        ]
    )
