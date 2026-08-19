# Physical-Joystick Shared Control

This procedure feeds the measured slot-2 physical JSM into the Jetson safety
supervisor. Forward motion and reverse motion inside the calibrated 25-degree
cones are supported. Forward CLEAR is capped at 70 raw counts and SLOW at 40.
Reverse is deliberately unmonitored by the front costmap and is always capped
at SLOW 40 with reason `reverse_unmonitored_slow`. Correction X scales with
the permitted Y magnitude so the input direction is preserved. Hard turns
stop locally and remain latched until both joystick axes return to neutral.

The Pi and Jetson use lockstep UDP protocol v2. Deploy and rebuild both sides
before testing; a mixed v1/v2 enforce setup intentionally fails closed.

Use the measured fixed addresses on the isolated router: `10.0.0.222` for the
Pi and `10.0.0.48` for the Jetson.

The only enforced use covered here is one attended, controlled-floor,
low-speed validation. It is not approval for normal operation. The Pi changes
only the two signed axes in each live slot-2 JSM frame; it does not send the
older teleop speed-profile frame. All enforcement gates remain disabled by
default and must be selected explicitly for this test.

## 1. Jetson LiDAR and Nav2 mapping

Start the AIRY and measured transform without the legacy mapper:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=false use_rviz:=false
```

In a second Jetson terminal, start the artifact filter, inflated Nav2 map, and
RViz:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_navigation nav2_mapping.launch.py \
  use_artifact_filter:=true use_inflation:=true use_rviz:=true
```

Confirm RViz shows a current, credible `/nav2_front_costmap` before
continuing.

## 2. Jetson supervisor and UDP bridge

In a third Jetson terminal:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_shared_control shared_control.launch.py \
  enable_motion:=true geometry_calibrated:=true enable_udp:=true \
  pi_address:=10.0.0.222 allowed_pi_address:=10.0.0.222 \
  slow_forward_limit:=0.40 \
  slow_cost_threshold:=1 stop_cost_threshold:=99
```

Both motion gates and UDP are explicit; they remain disabled in the normal
launch defaults.

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

`cangw -L` must be empty. Do not run keyboard teleop, the observer, or another
gateway concurrently.

## 4. Shadow gate

Start the Pi program before powering the wheelchair:

```bash
cd /home/raspberrywheelchair/Wheelchair-AutoNav-control/components/can_controller/scripts
python3 supervise_physical_joystick.py \
  --mode shadow \
  --can-interface can0 --gateway-interface can1 --device-slot 2 \
  --jetson-address 10.0.0.48 \
  --deadzone 4 --forward-cone-deg 25 \
  --csv /tmp/physical_shared_shadow_01.csv
```

Power the chair only after the gateway banner appears. Shadow mode displays
the semantic intent, angle, and safe command but forwards the physical command
unchanged. Pass this gate only when both forwarding counters rise, the
recorded forward corrections remain inside the cone, hard left/right and
reverse labels are correct, and clear/slow/stop decisions agree with RViz.
The four-count Pi deadzone is a temporary compatibility setting for the
known float32 boundary mismatch; it must stay explicit until that issue is
fixed on both machines.

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
  /operator_intent /artifact_filter/source_header \
  /nav2_front_costmap /safety_envelope /shared_control/diagnostics
```

Stop the shadow gateway, power-cycle the wheelchair if required by the R-Net
communication fault, and start a single explicit enforcing gateway. The CSV
path must not already exist:

```bash
python3 supervise_physical_joystick.py \
  --mode enforce \
  --can-interface can0 --gateway-interface can1 --device-slot 2 \
  --jetson-address 10.0.0.48 \
  --clear-cap 70 --slow-cap 40 --deadzone 4 --forward-cone-deg 25 \
  --required-clear-envelopes 5 --envelope-timeout-s 0.20 \
  --csv /tmp/physical_shared_enforce_01.csv
```

Validate in this order:

1. STOP obstacle: held forward input must transmit `(0,0)`.
2. Return to neutral, place the obstacle in the SLOW region, and request
   forward motion. Transmitted Y must not exceed 40 and X must scale with it.
3. Return to neutral and establish a clear lane. Five fresh envelopes are
   required before transmitted Y may rise, and it must never exceed 70.
4. Repeat shallow corrections on both sides; neither may create a local latch,
   and the reduced X/Y ratio must preserve the requested direction.
5. Make one straight approach to the soft obstacle and observe transmitted Y
   change in order from CLEAR `<=70`, to SLOW `<=40`, to latched STOP `0`.
6. Confirm straight and shallow-correction reverse requests never exceed a
   magnitude of 40 and report `reverse_unmonitored_slow`. Rear obstacles are
   not observed in this scope; use open rear clearance and the physical cutoff.
   Hard turns must remain zero; centre both axes before re-arming.
7. At the capped CLEAR speed, stop the artifact filter, Jetson supervisor, and
   network separately. Each must centre output within the 200 ms envelope
   timeout. Restart the full pipeline and return to neutral between drills.

Stop immediately on unexplained motion, a missed STOP, decision oscillation,
CAN errors, loss of the physical cutoff, or a forwarding counter that stops
increasing. A `stale_source` during an approach is a safe abort only if the Pi
immediately sends `(0,0)` and latches STOP; return to neutral and repeat rather
than counting that approach as a CLEAR/SLOW pass.

The run passes only when RViz, the Jetson decision, Pi `safe`/`sent` fields,
and physical response agree for all three states; the sent command never
exceeds the operator request or its 70/40 cap; failure drills centre within
200 ms; both forwarding counters continue increasing; and `errors=0` for the
entire capture.

Stopping the Pi program also stops its in-line gateway. The resulting R-Net
communication loss is expected to fail safe and may require a wheelchair power
cycle after another gateway is started.
