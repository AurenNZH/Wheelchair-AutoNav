"""Launch defaults kept importable for tests and operator documentation."""


LAUNCH_DEFAULTS = {
    "use_lidar": "true",
    "use_camera": "false",
    "use_mapping": "false",
    "use_rviz": "false",
    "publish_camera_tf": "false",
    "publish_base_lidar_tf": "true",
    # Sensor poses converted from the legacy base axes (X left, Y backward)
    # to REP-103 base axes (X forward, Y left): x_new=-y_old, y_new=x_old,
    # yaw_new=yaw_old+pi/2.
    "base_lidar_x": "0.330",
    "base_lidar_y": "-0.265",
    "base_lidar_z": "0.320",
    # Physically validated with forward targets after the base-axis conversion.
    "base_lidar_yaw": "1.04720",
    "base_lidar_pitch": "0.0",
    "base_lidar_roll": "0.0",
    "base_camera_x": "-0.360",
    "base_camera_y": "0.265",
    "base_camera_z": "1.300",
    "base_camera_yaw": "0.0",
    "base_camera_pitch": "0.0",
    "base_camera_roll": "0.0",
}


SENSOR_TRANSFORMS = {
    "rslidar": {
        "x": LAUNCH_DEFAULTS["base_lidar_x"],
        "y": LAUNCH_DEFAULTS["base_lidar_y"],
        "z": LAUNCH_DEFAULTS["base_lidar_z"],
        "yaw": LAUNCH_DEFAULTS["base_lidar_yaw"],
        "pitch": LAUNCH_DEFAULTS["base_lidar_pitch"],
        "roll": LAUNCH_DEFAULTS["base_lidar_roll"],
        "parent": "base_link",
        "child": "rslidar",
    },
    "camera_link": {
        "x": LAUNCH_DEFAULTS["base_camera_x"],
        "y": LAUNCH_DEFAULTS["base_camera_y"],
        "z": LAUNCH_DEFAULTS["base_camera_z"],
        "yaw": LAUNCH_DEFAULTS["base_camera_yaw"],
        "pitch": LAUNCH_DEFAULTS["base_camera_pitch"],
        "roll": LAUNCH_DEFAULTS["base_camera_roll"],
        "parent": "base_link",
        "child": "camera_link",
    },
}
