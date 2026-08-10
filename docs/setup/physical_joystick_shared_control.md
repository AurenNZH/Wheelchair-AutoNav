# Physical-Joystick Shared Control

This procedure feeds the measured slot-2 physical JSM into the Jetson safety
supervisor. The first enforced scope is straight-forward only. CLEAR is capped
at 20 raw counts, SLOW at 15, and STOP at zero. Reverse or steering outside the
five-count deadzone stops locally and remains latched until the forward axis
returns to neutral.

Use fixed Pi and Jetson addresses on the isolated router. The examples below
use `192.168.1.20` for the Pi and `192.168.1.10` for the Jetson; substitute the
actual fixed addresses.

## 1. Jetson mapping

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

Confirm RViz shows a current, credible `/front_costmap` before continuing.

## 2. Jetson supervisor and UDP bridge

In a second Jetson terminal:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_shared_control shared_control.launch.py \
  enable_motion:=true geometry_calibrated:=true enable_udp:=true \
  pi_address:=192.168.1.20 allowed_pi_address:=192.168.1.20 \
  min_steering:=0.0 max_steering:=0.0 slow_forward_limit:=0.15
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
  --jetson-address 192.168.1.10 \
  --csv /tmp/physical_shared_shadow.csv
```

Power the chair only after the gateway banner appears. Shadow mode displays
the safe command but forwards the physical command unchanged. Pass this gate
only when both forwarding counters rise, intent follows the physical forward
axis, and clear/slow/stop decisions agree with RViz and measured obstacles.

## 5. Low-speed enforcement

Use the controlled open area with clear escape space and an attendant holding
the tested physical cutoff. Start with the operator joystick centred and a
large soft obstacle already inside the STOP region. Replace `shadow` with
`enforce` and use a new CSV path:

```bash
python3 supervise_physical_joystick.py \
  --mode enforce \
  --can-interface can0 --gateway-interface can1 --device-slot 2 \
  --jetson-address 192.168.1.10 \
  --clear-cap 20 --slow-cap 15 --deadzone 5 \
  --csv /tmp/physical_shared_enforce.csv
```

Validate in this order:

1. STOP obstacle: held forward input must transmit `(0,0)`.
2. Clear space: five fresh envelopes are required, then transmitted Y must not
   exceed 20.
3. SLOW obstacle: transmitted Y must not exceed 15.
4. Approach the soft obstacle and observe CLEAR, SLOW, then latched STOP.
5. Release to neutral before re-arming; steering and reverse must remain zero.
6. Stop the map, Jetson supervisor, and network separately in open space; each
   must centre output within the 200 ms envelope timeout.

Stop immediately on unexplained motion, a missed STOP, decision oscillation,
CAN errors, map dropout, or a forwarding counter that stops increasing.

Stopping the Pi program also stops its in-line gateway. The resulting R-Net
communication loss is expected to fail safe and may require a wheelchair power
cycle after another gateway is started.
