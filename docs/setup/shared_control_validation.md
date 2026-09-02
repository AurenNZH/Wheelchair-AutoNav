# Shared-Control Validation Gates

This checklist validates supervised, low-speed forward motion inside the
configured correction cone and hard turns inside the base-centred clearance
disc. It does not validate autonomous navigation, reverse assistance,
stairs/drop-offs, curbs, outdoor/public operation, or operation without an
attendant and tested physical power cutoff.

Keep all three live gates off until their preceding stages pass:

- Jetson `enable_motion: false`
- Jetson `enable_udp: false`
- Pi `shared_control.enabled: false`

Record the software revision, configuration files, chair/load, L2 mounting
measurements, test layout, timestamps, and pass/fail evidence for every run.

## 1. Software and fail-closed checks

From the repository root:

```bash
source /opt/ros/foxy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-select \
  wheelchair_msgs wheelchair_navigation wheelchair_shared_control \
  wheelchair_simulation wheelchair_bringup
colcon test --packages-select wheelchair_msgs wheelchair_navigation \
  wheelchair_shared_control wheelchair_simulation wheelchair_bringup
colcon test-result --verbose
cd ..
python3 -m pytest components/can_controller/tests
```

Launch shared control with its defaults. Confirm diagnostics say
`live_control_disabled` once maps and intent exist, UDP reports disabled, and
no node publishes a physical `cmd_vel` or accesses CAN.

## 2. L2 and weighted Nav2 mapping acceptance

With drive power physically isolated, run the dual-L2/Nav2 pipeline and inspect
both filtered clouds and `/nav2_merged_costmap` in RViz. Shared control consumes
only the robot-relative Nav2 map; raw and low-support clouds remain diagnostic
evidence for sampling and blind rear coverage. Do not enable physical
enforcement until the supervisor validates both source heartbeats.

Pass only if all of the following hold:

- A measured target appears at the correct distance and size throughout the
  entire forward driving envelope, including beside both footrests.
- Obstacles remain present at both lateral edges and across the sensor-overlap
  region.
- The L2 window and mount are clean and secure, and empty-scene chassis returns
  do not enter the Nav2 obstacle map.
- Nearby real targets are not removed by the point-support filter.
- Raw obstacle size is credible.
- No unobserved sector is shown or treated as clear.
- At 10 Hz over ten minutes, processing p95 is below 100 ms, processing maximum
  and cloud-age p95 are below 150 ms, no repeating spike occurs, and no queue
  grows.
- Both `/lidar_<side>/filter/source_header` topics advance after every
  successfully filtered cloud. Forward motion retains the right-source gate;
  hard turns require both sources to remain fresh.
- Stopping the filter produces `stale_source`; stopping Nav2 while the filter
  remains live produces `stale_map`. Each transition must occur within the
  configured 0.50-second limit.

Save a short ROS bag for each measured target position and an empty-chair
reflection capture.

## 3. Simulation

Build the workspace, then run:

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=91
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false enable_sim_motion:=true operator_mode:=scenario scenario:=all
```

The full suite must pass without weakening the forward swept-footprint check.
Also launch `operator_mode:=keyboard gui:=true` and inspect straight, bounded-right,
unsupported-left, reverse-disabled, deadman-release, doorway, narrow-pole,
and moving-dummy behaviour. `enable_sim_motion:=true` must never affect a
non-`/sim` velocity topic.

## 4. Network and Pi bench test

Do this without a live CAN connection, preferably using `vcan`.

1. Give the Jetson and Pi fixed router addresses and allowlist each exact peer.
2. Confirm intent UDP 45450 is Pi-to-Jetson and envelope UDP 45451 is
   Jetson-to-Pi.
3. Enable only the UDP bridge and Pi safety link.
4. Verify five distinct clear heartbeats are required before a non-zero
   command is accepted.
5. During a held forward request, test Jetson process exit, Pi/Jetson cable
   removal, router power loss, packet loss/reordering, wrong sender, malformed
   JSON, old sequence, old session, stale map, and stale intent.
6. Every case must centre the output within the 200 ms envelope timeout.
7. Verify STOP remains latched until the operator releases the key.

The UDP protocol is unauthenticated; use only the isolated trusted router LAN.

## 5. Geometry, speed, and stopping calibration

Record the 1.00 m × 0.53 m centred body, then measure the wheel/caster sweep,
every supported footrest/mount position, L2 yaw convention, and maximum
intended loaded mass. L2 pitch and roll are confirmed zero. Replace
provisional collision dimensions only with the complete swept envelope.

After vCAN failure tests and live shadow observation, use either raised wheels
or the documented controlled open test area at the lowest command cap, with an
observer continuously holding the independent physical cutoff:

1. Confirm joystick release, stale input, Jetson loss, and emergency stop all
   centre the CAN command.
2. Confirm straight and shallow-correction behavior, the 90-count CLEAR cap,
   direction-preserving X scaling, and the distinct 60-count SLOW cap.
3. Confirm the physical cutoff works while software is unresponsive.

On a level, dry, controlled floor, measure worst-case stopping distance over
repeated runs at every allowed test speed and load. Set software STOP distance
no smaller than:

```text
worst measured stopping distance
+ distance travelled during worst measured end-to-end latency
+ localization/map resolution allowance
+ an explicit engineering safety margin
```

Do not infer braking distance from nominal motor speed.

## 6. Controlled obstacle gates

Use a safety observer, physical cutoff, open escape space, and the explicitly
configured 90-count CLEAR / 60-count SLOW caps.

1. Empty corridor: no false intervention.
2. Large static foam/cardboard obstacle: SLOW then STOP before the measured
   boundary.
3. In a clear disc, left/right turn requests may proceed only within the
   90-count CLEAR or 60-count SLOW lateral cap and 15-count longitudinal cap.
   A cost of 99 or greater anywhere in the checked disc must latch STOP until
   release. Reverse corrections may proceed only at the fixed 65-count reverse
   cap; maintain open rear clearance because reverse is unmonitored.
4. Doorway: reject openings below the measured chair envelope plus margin;
   allow a measured safe opening without autonomous steering.
5. Narrow pole and low block: repeat at multiple lateral offsets.
6. Side/rear obstacles outside the forward trajectory remain diagnostic-only;
   hard turns inspect only the 0.45 m pixelated disc and do not claim general
   full-surround protection.
7. Defer moving-person tests until forward, shallow-correction, and hard-turn
   CLEAR/SLOW/STOP gates pass repeatably.

Any collision, missed detection, oscillating permit/stop behavior, timeout
violation, or unexplained map dropout is a failed gate.

## 7. Conditional human crossing

Attempt this only after every previous gate passes with repeatable logs. Use a
walking volunteer only in a controlled indoor test area, at the lowest speed,
with an independent observer holding the tested cutoff. Start well outside the
stopping envelope and cross predictably. Stop immediately after any unexpected
behavior; do not tune during a live human run.

## Future left L2

Add the left sensor only after the right sensor independently passes these
gates. Dual-L2 integration requires separate power, Ethernet, timing, mounting,
driver configuration, cloud input, fusion, and regression evidence. The
measured symmetric left mount frame and URDF geometry are already defined, but
they do not claim that a left driver, point cloud, or symmetric obstacle
coverage is active.
