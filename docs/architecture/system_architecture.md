# Current System Architecture

## MVP Data Flow

```text
AIRY /rslidar_points
        |
        v
Jetson wheelchair_navigation
  /local_obstacles ---- nearby proximity evidence
  /front_costmap ------ forward/right swept-path evidence
        |
        v
Jetson wheelchair_shared_control
  OperatorIntent + fresh maps
        |
        v
  SafetyEnvelope: STOP / SLOW / CLEAR
        |
   UDP safety link
        |
        v
Raspberry Pi wheelchair_teleop
  deadman + sequence + timeout + command cap
        |
        v
CAN/RNET joystick frames
```

The operator always selects the requested direction. The Jetson never chooses
a route and never publishes a physical `Twist` or CAN frame.

## Responsibilities

- `wheelchair_bringup` owns sensor launch and calibrated static transforms.
- `wheelchair_navigation` decodes AIRY clouds, transforms them to
  `base_link`, applies range/height/self filtering, and publishes two raw maps.
- `wheelchair_shared_control` checks the requested swept footprint and emits a
  normalized safety envelope.
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

The single AIRY supports the inspected indoor forward/right MVP. Chair
occlusion prevents safe left turns. The sensor does not validate drop-offs,
steps, curbs, reverse, or low hazards beneath its observed height.

## Fail-Closed Layers

Motion is zero when any required map, intent, timestamp, sequence, peer,
session, or heartbeat is invalid or stale. STOP latches on the Pi until the
operator releases the motion key. Gazebo uses the same intent and envelope
contracts but can publish velocity only to `/sim/safe_cmd_vel`.

Future autonomous route selection must enter through a separate arbitration
interface; it must not impersonate operator intent.
