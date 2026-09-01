# Physical-Joystick Shared Control

This procedure feeds the measured slot-2 physical JSM into the Jetson safety
supervisor. Forward motion and reverse motion inside symmetric 30-degree cones
are supported. Forward CLEAR is capped at 90 raw counts and SLOW at 60.
Reverse is deliberately unmonitored by the supervisor and is always capped
at SLOW 65 with reason `reverse_unmonitored_slow`. Correction X scales with
the permitted Y magnitude so the input direction is preserved. Hard turns
check a 0.55 m base-centred costmap disc and require both L2 filter heartbeats.
Their CLEAR/SLOW lateral caps are 90/60, with longitudinal adjustment capped
at 15. The two obstacle STOP reasons can automatically resume held physical
input after five fresh matching non-STOP envelopes; every other supervisor
STOP remains latched until joystick release.

The Pi and Jetson use lockstep UDP protocol v3 over the isolated Ethernet
router. Deploy and rebuild both sides before testing; a mixed v2/v3 enforce
setup intentionally fails closed. The validated control addresses are
`192.168.0.100` for the Pi and `192.168.0.102` for the Jetson. The separate
`192.168.1.0/24` addresses remain dedicated to the dual-L2 path. The Pi's
Wi-Fi address `10.0.0.253` is retained for recovery access, not for the UDP
safety link.

Before launch, verify that both control routes use Ethernet:

```bash
# Jetson
ip route get 192.168.0.100
ping -I 192.168.0.102 -c 4 192.168.0.100

# Raspberry Pi
ip route get 192.168.0.102
ping -I 192.168.0.100 -c 4 192.168.0.102
```

The route output must report `dev eth0`. `ROS_LOCALHOST_ONLY=1` may remain set
on the Jetson: it limits ROS DDS traffic, not this explicit UDP safety link.

The only enforced use covered here is one attended, controlled-floor,
low-speed validation. It is not approval for normal operation. The Pi changes
only the two signed axes in each live slot-2 JSM frame; it does not send the
older teleop speed-profile frame. All enforcement gates remain disabled by
default and must be selected explicitly for this test.

## 1. Jetson LiDAR and Nav2 mapping

Start both L2s, measured transforms, support filters, Nav2 map, and RViz:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true \
  use_inflation:=true
```

Confirm RViz shows a current, credible `/nav2_merged_costmap` and both filtered
clouds before continuing. Hard turns fail closed unless both source-heartbeat
topics are current.

## 2. Jetson supervisor and UDP bridge

In a third Jetson terminal:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_shared_control shared_control.launch.py \
  enable_motion:=true geometry_calibrated:=true enable_udp:=true \
  bind_address:=192.168.0.102 \
  pi_address:=192.168.0.100 allowed_pi_address:=192.168.0.100 \
  slow_forward_limit:=0.60 reverse_limit:=0.65 \
  turn_clearance_radius_m:=0.55 clear_turn_limit:=0.90 \
  slow_turn_limit:=0.60 turn_longitudinal_limit:=0.15 \
  slow_cost_threshold:=1 stop_cost_threshold:=99
```

Both motion gates and UDP are explicit; they remain disabled in the normal
launch defaults.

For reactive-assistance shadow validation, replace the separate mapping and
supervisor launches with the package-level launch below. It starts the same L2
support filters, a planner-owned 5 m by 8 m inflated costmap, shared control,
and the 2 Hz Nav2 waypoint research comparison. Reactive assistance remains
disabled unless explicitly set to shadow or enforce.

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  reactive_assistance_mode:=shadow nav2_waypoint_mode:=shadow \
  nav2_waypoint_rate_hz:=2.0 enable_udp:=true \
  bind_address:=192.168.0.102 \
  pi_address:=192.168.0.100 allowed_pi_address:=192.168.0.100
```

Do not enable motion during the shadow capture. Inspect
`/local_avoidance/path`, `/local_avoidance/goal`,
`/local_avoidance/diagnostics`, and
`/shared_control/nav2_waypoint_suggestion` for route-research results. Inspect
`/shared_control/reactive_suggestion`,
`/shared_control/reactive_candidates`, and the reactive diagnostic keys for
the low-latency selector. Shadow mode never changes the envelope.

## 3. Pi CAN preparation

With the wheelchair powered off, bring both in-line interfaces up and verify
that no kernel gateway is active:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 125000
sudo ip link set can1 up
ip -brief link show can0 can1
cangw -L
```

`cangw -L` must be empty. In the current physical wiring, `can1` is the
wheelchair-controller side and `can0` is the physical-JSM side. Do not run
keyboard teleop, the observer, or another gateway concurrently.

## 4. Shadow gate

Start the Pi program before powering the wheelchair:

```bash
cd /home/raspberrywheelchair/Wheelchair-AutoNav-control/components/can_controller/scripts
python3 supervise_physical_joystick.py \
  --mode shadow \
  --can-interface can1 --gateway-interface can0 --device-slot 2 \
  --jetson-address 192.168.0.102 \
  --allowed-jetson-address 192.168.0.102 \
  --deadzone 4 --forward-cone-deg 30 \
  --max-assist-ratio 0 \
  --csv /tmp/physical_shared_shadow_01.csv
```

Power the chair only after the gateway banner appears. Shadow mode displays
the semantic intent, angle, and safe command but forwards the physical command
unchanged. Pass this gate only when both forwarding counters rise, the
recorded forward corrections remain inside the cone, hard left/right and
reverse labels are correct, and clear/slow/stop decisions agree with RViz.
The banner reports whether obstacle auto-resume is enabled. By default,
`nav2_cost_stop` and `nav2_turn_cost_stop` may recover without joystick release,
but only after five fresh, distinct, matching SLOW or CLEAR envelopes; any
intervening STOP resets that count. All other STOP reasons remain latched until
release. Use `--require-release-after-obstacle-stop` for the previous behavior
where obstacle STOP also requires release.
The four-count Pi deadzone is a temporary compatibility setting for the
known float32 boundary mismatch; it must stay explicit until that issue is
fixed on both machines.

After shadow results pass, an unoccupied reactive-assistance enforce test uses
`reactive_assistance_mode:=enforce` on the Jetson and explicitly delegates at
most 0.15 on the physical gateway with `--max-assist-ratio 0.15`. Omitting that
flag (or setting it to zero) prevents reactive steering in enforce mode.
Reverse and hard turns retain the direct policy, direct STOP never attempts an
escape, and an enforced correction retains the direct SLOW cap and reason.

## 5. Low-speed enforcement

Use the controlled open area with clear escape space and an attendant holding
the tested physical cutoff. Start with the operator joystick centred and a
large soft obstacle already inside the STOP region. Do not carry a passenger,
run keyboard teleop, install `cangw` rules, or use another CAN gateway during
this first validation.

Before enforcement, record the Jetson evidence in a fourth terminal:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 bag record -o /tmp/physical_shared_enforce_01 \
  /lidar_right/points_filtered /lidar_left/points_filtered \
  /operator_intent /lidar_right/filter/source_header \
  /lidar_left/filter/source_header \
  /nav2_merged_costmap /nav2_merged_costmap_footprint \
  /safety_envelope /shared_control/diagnostics \
  /shared_control/checked_corridor /shared_control/reactive_candidates \
  /shared_control/reactive_suggestion \
  /shared_control/nav2_waypoint_suggestion \
  /local_avoidance/goal /local_avoidance/path /plan \
  /local_avoidance/diagnostics
```

After stopping the recording, verify that every required topic was captured:

```bash
ros2 bag info /tmp/physical_shared_enforce_01
```

In RViz, orange `/plan` is Nav2's last successful raw route and can remain
visible after an abort. Green `/local_avoidance/path` is the route accepted by
the wheelchair planner client; it is cleared after an invalid or superseding
result. The magenta arrow is the temporary goal and the cyan polygon is the
planner footprint.

Stop the shadow gateway, power-cycle the wheelchair if required by the R-Net
communication fault, and start a single explicit enforcing gateway. The CSV
path must not already exist:

```bash
python3 supervise_physical_joystick.py \
  --mode enforce \
  --can-interface can1 --gateway-interface can0 --device-slot 2 \
  --jetson-address 192.168.0.102 \
  --allowed-jetson-address 192.168.0.102 \
  --clear-cap 90 --slow-cap 60 --reverse-cap 65 \
  --turn-clear-cap 90 --turn-slow-cap 60 \
  --turn-longitudinal-cap 15 \
  --deadzone 4 --forward-cone-deg 30 \
  --max-assist-ratio 0.15 \
  --required-clear-envelopes 5 --envelope-timeout-s 0.20 \
  --csv /tmp/physical_shared_enforce_01.csv
```

Validate in this order:

1. STOP obstacle: held forward input must transmit `(0,0)`. Remove the soft
   obstacle from outside the wheelchair path without releasing the joystick;
   output must remain zero for four fresh matching SLOW/CLEAR envelopes and
   may resume on the fifth.
2. Repeat the held-input recovery into SLOW. Transmitted Y must not exceed 60,
   X must scale with the current joystick position, and a new STOP during the
   count must restart all five envelopes.
3. Repeat into CLEAR. The fifth fresh envelope may permit motion, which must
   use the current joystick position and never exceed 90.
4. Repeat shallow corrections on both sides; neither may create a local latch,
   and the reduced X/Y ratio must preserve the requested direction.
5. Make one straight approach to the soft obstacle and observe transmitted Y
   change in order from CLEAR `<=90`, to SLOW `<=60`, to STOP `0`, then recover
   only after five fresh matching SLOW/CLEAR envelopes.
6. Confirm straight and shallow-correction reverse requests never exceed a
   magnitude of 65 and report `reverse_unmonitored_slow`. Rear obstacles are
   not observed in this scope; use open rear clearance and the physical cutoff.
7. In open clearance, request pure left and right turns. The displayed disc,
   supervisor decision, and sent axes must agree; lateral output must remain
   within 90/60 and longitudinal output within 15.
8. Place a soft obstacle inside the disc and confirm held left and right turn
   requests send `(0,0)`, then resume only after five fresh matching SLOW/CLEAR
   envelopes when the disc clears. Stop either L2 filter separately; hard turns
   must fail closed and remain latched until joystick release.
9. At the capped CLEAR speed, stop the point-support filter, Jetson supervisor, and
   network separately. Each must centre output within the 200 ms envelope
   timeout. Restart the full pipeline and return to neutral between drills.

Stop immediately on unexplained motion, a missed STOP, decision oscillation,
CAN errors, loss of the physical cutoff, or a forwarding counter that stops
increasing. A `stale_source` during an approach is a safe abort only if the Pi
immediately sends `(0,0)` and latches STOP; return to neutral and repeat rather
than counting that approach as a CLEAR/SLOW pass.

The run passes only when RViz, the Jetson decision, Pi `safe`/`sent` fields,
and physical response agree for all three states; the sent command never
exceeds the operator request, its 90/60 forward or turn cap, the 15-count turn
longitudinal cap, or the 65 reverse cap;
failure drills centre within
200 ms; both forwarding counters continue increasing; and `errors=0` for the
entire capture.

Stopping the Pi program also stops its in-line gateway. The resulting R-Net
communication loss is expected to fail safe and may require a wheelchair power
cycle after another gateway is started.
