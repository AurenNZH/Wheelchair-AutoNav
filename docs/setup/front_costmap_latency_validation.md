# Front-Costmap Latency Validation

This procedure measures acquisition-to-supervisor latency without moving the
wheelchair. It preserves the AIRY first-point timestamp and the supervisor's
300 ms freshness limit. Never use output restamping to pass this gate.

The normal `safety` profile publishes only `/front_costmap` and opens a light
10 FPS RViz map view. The `artifact_debug` profile retains the full point-cloud,
filtered-map, and marker calibration view; it is not an enforcement profile.

## Build

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  wheelchair_navigation wheelchair_bringup wheelchair_shared_control
source install/setup.bash
```

Confirm the intended Jetson power profile. Acceptance uses the normal 20 W
six-core mode without `jetson_clocks`:

```bash
nvpmodel -q
```

## Repeatable A/B captures

Keep the wheelchair and scene stationary. For every profile, start the launch
in terminal 1, then run the recorder and `tegrastats` in separate terminals.
The recorder ignores a 15-second warm-up and captures the following 120
seconds. Use a new CSV and tegrastats filename for every run.

Terminal 2 starts the supervisor with all motion gates disabled:

```bash
ros2 launch wheelchair_shared_control shared_control.launch.py
```

Terminal 3 provides a current, released intent without commanding motion:

```bash
ros2 run wheelchair_shared_control operator_intent_injector
```

Terminal 4:

```bash
ros2 run wheelchair_navigation mapping_latency_recorder --ros-args \
  -p output_csv:=/tmp/front_latency_PROFILE.csv \
  -p warmup_s:=15.0 -p duration_s:=120.0
```

Terminal 5:

```bash
tegrastats --interval 1000 --logfile /tmp/tegrastats_PROFILE.txt
```

Stop `tegrastats` after the recorder exits. Run these profiles in order:

1. Driver baseline, with no mapper or RViz:

   In terminal 1, start the driver:

   ```bash
   ros2 launch wheelchair_bringup wheelchair.launch.py \
     use_lidar:=true use_mapping:=false use_rviz:=false
   ```

   In another terminal, sample the source rate for 120 seconds, stop it with
   `Ctrl+C`, and then sample source delay for another 120 seconds:

   ```bash
   ros2 topic hz /rslidar_points --window 100
   ```

   ```bash
   ros2 topic delay /rslidar_points --window 100
   ```

   The map recorder is not used for this driver-only case; retain the `hz`,
   `delay`, and tegrastats output instead.

2. Safety mapper, headless:

   ```bash
   ros2 launch wheelchair_bringup wheelchair.launch.py \
     use_lidar:=true use_mapping:=true use_rviz:=false \
     runtime_profile:=safety
   ```

3. Safety mapper with the lightweight RViz view:

   ```bash
   ros2 launch wheelchair_bringup wheelchair.launch.py \
     use_lidar:=true use_mapping:=true use_rviz:=true \
     runtime_profile:=safety
   ```

4. Artifact computation without RViz:

   ```bash
   ros2 launch wheelchair_bringup wheelchair.launch.py \
     use_lidar:=true use_mapping:=true use_rviz:=false \
     runtime_profile:=artifact_debug
   ```

   After that capture completes, stop the launch with `Ctrl+C`. Then start a
   separate capture with the full debug view:

   ```bash
   ros2 launch wheelchair_bringup wheelchair.launch.py \
     use_lidar:=true use_mapping:=true use_rviz:=true \
     runtime_profile:=artifact_debug
   ```

Run one launch at a time and ensure no older mapper, RViz, or LiDAR process is
still active. Use the profile deltas to distinguish driver/source age,
front-map processing, artifact computation, and RViz rendering load.

## Acceptance gate

After the A/B investigation, run the safety profile with RViz continuously for
five minutes by setting `duration_s:=300.0`. It passes only when:

- supervisor-observed map age p99 is at most 250 ms;
- no map or supervisor decision exceeds the 300 ms deadline;
- map rate is at least 90% of the measured AIRY cloud rate;
- no front-map arrival gap exceeds 300 ms;
- safety RViz adds no more than 25 ms to p99 age and reduces rate by less than
  5% relative to the headless safety profile;
- timestamp, TF, rejected-cloud, and thermal-throttling errors remain zero.

Do not raise `max_map_age_s`, set `restamp_output_with_node_time`, or enable
physical enforcement to compensate for a failed latency gate.
