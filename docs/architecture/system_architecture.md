# Current System Architecture

```text
right + left Unitree L2 PointCloud2 streams
        |
wheelchair_navigation
  independent point-support filters
  stock Nav2 ObstacleLayer with two observation sources
  /nav2_merged_costmap
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
supervisor source-freshness heartbeat remains right-only pending a separate
dual-source physical-safety validation. Rear obstacle avoidance, drop-offs,
steps, curbs, and hazards below the observed height remain outside the accepted
scope. The merged map extends 0.6 m behind `base_link`, but reverse and hard-turn
obstacle intervention remain outside the current motion policy.
