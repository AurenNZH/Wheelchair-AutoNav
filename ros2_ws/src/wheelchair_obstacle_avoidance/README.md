# Wheelchair Obstacle Avoidance

This package is the top-level launcher for the deployed reactive shared-control
pipeline. It starts the dual-L2 support/self filters, one standalone
`base_link` Nav2 costmap with 0.45 m inflation, the safety supervisor, and—only
when requested—the Jetson/Pi UDP bridge. It does not create temporary goals,
request Nav2 paths, or own another costmap.

The package launch defaults to reactive enforcement, while the independent
physical gates remain closed: `enable_motion`, `geometry_calibrated`, and
`enable_udp` all default to `false`. Thus the default invocation calculates and
visualizes decisions but cannot actuate the chair:

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  use_rviz:=true
```

For non-actuating evaluation, select `reactive_assistance_mode:=shadow`.
Reactive selection is eligible only for forward, forward-left, and
forward-right intent after the direct policy returns `nav2_cost_slow`. It
compares bounded 1.2 m arcs, requires two matching correction directions, and
changes steering only. STOP, CLEAR, reverse, hard turns, speed caps, freshness,
and invalid-evidence behavior remain under the direct fail-closed policy.

The configured maximum correction is 0.577350269 normalized steering ratio
(`tan(30 degrees)`). Physical enforcement also requires a protocol-v3 intent
advertising authority (normally Pi option
`--max-assist-ratio 0.577350269`), both Jetson motion gates, and UDP to be
enabled explicitly. The ratio describes the commanded joystick vector rather
than a guaranteed physical wheel angle. See
[the deployment roadmap](../../../docs/setup/obstacle_avoidance_roadmap.md).

The earlier temporary-waypoint/Smac experiment was removed. It competed for
CPU and costmap ownership, retained stale path state after failures, and did
not fit an odometry-free operator-led chair. Historical results remain in
project history rather than in the runtime package.
