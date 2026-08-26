# Current System Architecture

```text
Unitree L2 /lidar_right/points
        |
wheelchair_navigation
  point-support filtered PointCloud2
  /nav2_front_costmap
        |
wheelchair_shared_control + OperatorIntent
        |
  SafetyEnvelope: STOP / SLOW / CLEAR
        |
   UDP safety link
        |
Raspberry Pi wheelchair_teleop -> CAN/RNET joystick frames
```

`wheelchair_bringup` owns how physical sensors and transforms start.
`wheelchair_navigation` owns filtering and map generation.
`wheelchair_shared_control` owns how weighted costs affect motion permission.
`wheelchair_simulation` substitutes Gazebo sensors and motion while preserving
the same ROS interfaces. The operator remains the source of requested direction.

`base_link` uses X forward, Y left, and Z up. The installed right sensor is:

```text
base_link -> lidar_right_link: 0.330 -0.220 0.320 0.479965544 0 0
```

Motion fails closed when required intent, map receipt, source timestamp,
sequence, peer, session, or heartbeat data is invalid or stale. The current
right-only sensing scope does not validate hard left turns, rear obstacle
avoidance, drop-offs, steps, curbs, or hazards below the observed height.
