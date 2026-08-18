# Shared-Control Validation Gates

This checklist validates supervised, low-speed forward motion inside the
configured correction cone. It does not validate autonomous navigation, hard
turns, reverse assistance, stairs/drop-offs, curbs, outdoor/public operation,
or operation without an attendant and tested physical power cutoff.

Keep all three live gates off until their preceding stages pass:

- Jetson `enable_motion: false`
- Jetson `enable_udp: false`
- Pi `shared_control.enabled: false`

Record the software revision, configuration files, chair/load, AIRY mounting
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

## 2. AIRY and weighted Nav2 mapping acceptance

With drive power physically isolated, run the AIRY mapper and inspect
`/rslidar_points_artifact_filtered` and `/nav2_front_costmap` in RViz. Shared
control consumes only the robot-relative Nav2 map; raw and rejected clouds
remain diagnostic evidence for reflections and blind side/rear coverage.

Pass only if all of the following hold:

- A measured target appears at the correct distance and size throughout the
  entire forward driving envelope, including beside both footrests.
- Right-side obstacles remain in the observed raw map. Do not infer free space
  in the left-side sector occluded by the chair.
- The AIRY dome is clean, the hood is secure, and empty-scene chassis returns
  are absent or confined to the measured self-filter.
- Nearby real targets are not removed by the hood or self-filter.
- Raw obstacle size is credible.
- No unobserved sector is shown or treated as clear.
- At 10 Hz over ten minutes, processing p95 is below 100 ms, processing maximum
  and cloud-age p95 are below 150 ms, no repeating spike occurs, and no queue
  grows.

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

The full suite must eventually pass, but `clear_forward` currently exposes a
known Gazebo fixed-joint self-return. Keep this as a visible blocker; do not
weaken the forward swept-footprint check to make the simulation pass.
Meanwhile, launch
`operator_mode:=keyboard gui:=true` and inspect straight, bounded-right,
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
every supported footrest/mount position, AIRY yaw convention, and maximum
intended loaded mass. AIRY pitch and roll are confirmed zero. Replace
provisional collision dimensions only with the complete swept envelope.

After vCAN failure tests and live shadow observation, use either raised wheels
or the documented controlled open test area at the lowest command cap, with an
observer continuously holding the independent physical cutoff:

1. Confirm joystick release, stale input, Jetson loss, and emergency stop all
   centre the CAN command.
2. Confirm straight and shallow-correction behavior, the 20% CLEAR cap,
   direction-preserving X scaling, and the distinct 15% SLOW cap.
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

Use a safety observer, physical cutoff, open escape space, and command cap at
20% or lower.

1. Empty corridor: no false intervention.
2. Large static foam/cardboard obstacle: SLOW then STOP before the measured
   boundary.
3. Every hard-turn or reverse request must remain stopped; only corrections
   inside the validated 25-degree cone may proceed.
4. Doorway: reject openings below the measured chair envelope plus margin;
   allow a measured safe opening without autonomous steering.
5. Narrow pole and low block: repeat at multiple lateral offsets.
6. Keep side/rear obstacles outside the requested trajectory diagnostic-only;
   the current supervisor samples `/nav2_front_costmap` only along the
   straight-to-requested correction union and does not claim full-surround
   protection.
7. Defer moving-person tests and hard-turn physical control until straight and
   shallow-correction CLEAR/SLOW/STOP gates pass repeatably.

Any collision, missed detection, oscillating permit/stop behavior, timeout
violation, or unexplained map dropout is a failed gate.

## 7. Conditional human crossing

Attempt this only after every previous gate passes with repeatable logs. Use a
walking volunteer only in a controlled indoor test area, at the lowest speed,
with an independent observer holding the tested cutoff. Start well outside the
stopping envelope and cross predictably. Stop immediately after any unexpected
behavior; do not tune during a live human run.

## AIRY versus Unitree L2 decision

Keep the single AIRY through the August 31 delivery for forward/right motion if
it passes the mapping, coverage, latency, and controlled-obstacle gates. Its
working driver and good observed coverage are a schedule advantage.

Trigger a single-L2 contingency only if the AIRY has a documented,
safety-relevant blind/reflection sector that cannot be fixed physically or
filtered without losing real targets. Freeze the delivery sensor choice no
later than mid-August; a dual-L2 integration adds power, Ethernet, time
synchronization, calibration, fusion, mounting, and regression work and should
not be placed on the August 31 critical path. Consider dual L2 only as a
post-delivery coverage upgrade after one L2 independently passes the same
gates. Symmetric left/right shared control requires that additional left-side
coverage.
