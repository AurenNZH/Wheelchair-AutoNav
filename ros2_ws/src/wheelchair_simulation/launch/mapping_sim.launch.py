import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wheelchair_simulation")
    gazebo = os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
    robot = os.path.join(share, "urdf", "wheelchair.urdf.xacro")
    world = os.path.join(share, "worlds", "mapping.world")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo), launch_arguments={"world": world, "gui": LaunchConfiguration("gui")}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[{"robot_description": Command(["xacro ", robot])}]),
        Node(package="gazebo_ros", executable="spawn_entity.py", arguments=["-entity", "wheelchair", "-topic", "robot_description"], output="screen"),
    ])
