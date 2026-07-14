"""Launch defaults kept importable for tests and operator documentation."""


LAUNCH_DEFAULTS = {
    "use_lidar": "true",
    "use_camera": "true",
    "use_navigation": "false",
    "use_rviz": "false",
    "publish_camera_tf": "true",
    "publish_base_lidar_tf": "false",
    "base_lidar_x": "0.0",
    "base_lidar_y": "0.0",
    "base_lidar_z": "0.0",
    "base_lidar_yaw": "0.0",
    "base_lidar_pitch": "0.0",
    "base_lidar_roll": "0.0",
}


CAMERA_LIDAR_TRANSFORM = {
    "x": "0.42",
    "y": "0.65",
    "z": "1.03",
    "yaw": "-0.78540",
    "pitch": "0.0",
    "roll": "0.0",
    "parent": "rslidar",
    "child": "camera_link",
}
