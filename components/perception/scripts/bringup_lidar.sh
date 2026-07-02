#!/usr/bin/env bash
set -e

ROS_DISTRO="${ROS_DISTRO:-foxy}"
RSLIDAR_WS="${RSLIDAR_WS:-$HOME/rslidar_ws}"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
RSLIDAR_SETUP="${RSLIDAR_WS}/install/setup.bash"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS2 setup file not found: $ROS_SETUP" >&2
    echo "Set ROS_DISTRO if you are not using Foxy." >&2
    exit 1
fi

if [ ! -f "$RSLIDAR_SETUP" ]; then
    echo "RoboSense workspace setup file not found: $RSLIDAR_SETUP" >&2
    echo "Build rslidar_sdk first, or set RSLIDAR_WS=/path/to/your/workspace." >&2
    exit 1
fi

source "$ROS_SETUP"
source "$RSLIDAR_SETUP"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 command not found after sourcing ROS setup files." >&2
    exit 1
fi

if ! ros2 pkg prefix rslidar_sdk >/dev/null 2>&1; then
    echo "rslidar_sdk package was not found in the sourced workspace." >&2
    echo "Check that rslidar_sdk was built in: $RSLIDAR_WS" >&2
    exit 1
fi

echo "Starting RoboSense AIRY LiDAR via rslidar_sdk."
echo "Expected SDK config: lidar_type=RSAIRY, msg_source=1, send_point_cloud_ros=true."
echo "Expected ports: MSOP=6699, DIFOP=7788. Default point cloud topic: /rslidar_points."

exec ros2 launch rslidar_sdk start.py
