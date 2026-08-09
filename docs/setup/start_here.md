# Start Here

## 1. Build and Test

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Run Pi software tests separately:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
python3 -m pytest components/can_controller/tests
```

## 2. Inspect AIRY Mapping Without Motion

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_bringup wheelchair.launch.py \
  use_lidar:=true use_camera:=false use_mapping:=true use_rviz:=true
```

In a second terminal:

```bash
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run wheelchair_navigation mapping_monitor
```

The mapper publishes `/local_obstacles` and `/front_costmap`. Shared control
uses only `/front_costmap`; `/local_obstacles` remains useful for RViz and
self-filter diagnostics.

## 3. Run Gazebo Before Hardware

Use a separate ROS domain so simulated and physical sensor graphs cannot mix:

```bash
export ROS_DOMAIN_ID=91
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=false enable_sim_motion:=true operator_mode:=scenario scenario:=all
```

For interactive exploration:

```bash
export ROS_DOMAIN_ID=91
ros2 launch wheelchair_simulation shared_control_sim.launch.py \
  gui:=true enable_sim_motion:=true operator_mode:=keyboard
```

Controls are W straight, D right, A unsupported-left test, S reverse-disabled
test, Space/X stop, and Q quit. Movement expires unless the key is refreshed.
The full scripted suite is not yet a release gate: Gazebo still reports a
fixed-joint wheelchair self-return in `clear_forward`. See the simulation
package README before interpreting that failure.

## 4. Physical Progression

Follow [shared_control_validation.md](shared_control_validation.md) in order:

1. clean-dome, hood, coverage, and latency validation;
2. measured swept geometry;
3. `vcan` Jetson–Pi failure tests;
4. raised-wheel tests with an independent cutoff;
5. empty-corridor movement at the lowest effective speed;
6. controlled foam-obstacle SLOW/STOP tests.

Never jump directly from RViz validation to obstacle-driving tests.

Before physical-JSM shared control, complete the receive-only
[physical joystick observation](physical_joystick_observer.md). That observer
does not alter the existing CAN passthrough or publish an actuator command.
