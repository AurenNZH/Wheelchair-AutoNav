# Current System Architecture

## MVP Data Flow

```text
AIRY /rslidar_points
        |
        v
Jetson wheelchair_navigation
  artifact-filtered PointCloud2 ---- sensor diagnostics / RViz
  /nav2_front_costmap -------------- weighted trajectory evidence
        |
        v
Jetson wheelchair_shared_control
  OperatorIntent + provisional-fresh /nav2_front_costmap
        |
        v
  SafetyEnvelope: STOP / SLOW / CLEAR
        |
   UDP safety link
        |
        v
Raspberry Pi wheelchair_teleop
  2-D intent class + deadman + sequence + timeout + vector-preserving cap
        |
        v
CAN/RNET joystick frames
```

The operator always selects the requested direction. The Jetson never chooses
a route and never publishes a physical `Twist` or CAN frame.

## Responsibilities

- `wheelchair_bringup` owns sensor launch and calibrated static transforms.
- `wheelchair_navigation` filters calibrated AIRY artifacts and feeds the
  retained PointCloud2 records to Nav2's obstacle and inflation layers.
- `wheelchair_shared_control` samples weighted costs from straight through the
  requested correction and emits a normalized safety envelope.
- `wheelchair_teleop` owns keyboard input, CAN timing, command ramping, the
  physical deadman, and the fail-closed UDP client.
- `wheelchair_simulation` substitutes Gazebo differential drive for CAN and
  provides deterministic and interactive intent sources.

## Frames and Coverage

`base_link` follows the ROS mobile-base convention: X forward, Y left, Z up.
The validated AIRY transform is:

```text
base_link -> rslidar: 0.330 -0.265 0.320 1.04720 0 0
```

The software supports a symmetric forward-correction cone, but physical use on
either side remains conditional on RViz coverage and controlled obstacle gates.
Hard turns, reverse, drop-offs, steps, curbs, and low hazards beneath the
observed height remain outside the validated scope.

## Fail-Closed Layers

Motion is zero when the required costmap, intent, timestamp, sequence, peer,
session, or heartbeat is invalid or stale. Nav2 publication time is still a
provisional freshness source, so physical enforcement remains blocked pending
LiDAR-derived freshness. Side and rear obstacles outside the sampled path are
outside the current forward-only supervisor contract. STOP latches on the Pi
until the operator releases the motion key. Gazebo uses the same intent and
envelope contracts but can publish velocity only to `/sim/safe_cmd_vel`.

Future autonomous route selection must enter through a separate arbitration
interface; it must not impersonate operator intent.
