# Reactive Obstacle Assistance: Deployment Roadmap

This document is the authoritative scope and deployment order for obstacle
assistance. The deployed model is bounded reactive steering during an existing
SLOW decision. The operator continues to choose direction and speed; the
system does not infer a destination or follow a path.

The old temporary Nav2 waypoint experiment has been removed from the runtime.
Its evidence remains available in version history, but it must not be launched
beside this pipeline.

## Runtime pipeline

```text
dual L2 clouds
    -> support and fixed-artifact filtering
    -> compact XYZ clouds stamped in base_link
    -> one standalone 5 m x 8 m Nav2 costmap (0.1 m cells, 0.45 m inflation)
    -> direct arc/disc STOP-SLOW-CLEAR policy at 20 Hz
         STOP/CLEAR/reverse/hard turn -> unchanged direct decision
         eligible forward SLOW -> bounded 1.2 m steering fan
                              -> two-cycle confirmation
                              -> shadow suggestion or steering-only enforcement
    -> SafetyEnvelope -> optional UDP bridge -> Pi physical-JSM gateway
```

The supervisor remains authoritative. Reactive assistance never changes the
longitudinal command, SLOW cap, policy reason, freshness checks, peer checks,
or emergency behavior. Straight intent may inspect both sides. Forward-left
and forward-right may only reduce the requested correction toward straight;
they cannot cross zero or increase the turn. A STOP never initiates an escape
search.

## Deployed configuration

- `reactive_assistance_mode`: `disabled`, `shadow`, or `enforce`. The
  obstacle-avoidance launcher defaults to `enforce`; the generic shared-control
  launcher remains `disabled` for compatibility.
- `maximum_assist`: 0.577350269 normalized steering ratio (`tan(30 degrees)`).
- Candidate horizon/sample step: 1.2 m / 0.05 m.
- Candidate steering step/minimum correction: 0.05 / 0.02.
- Minimum same-class cost improvement: 5.
- Confirmation: two matching correction directions.
- Inflation radius/cost scaling: 0.45 m / 3.0.
- Hard-turn check radius: 0.45 m.
- Maximum physical-intent age: 1.00 s; the Pi envelope timeout remains 0.20 s.
- Costmap resolution and dimensions: 0.1 m, 5 m by 8 m, in `base_link`.
- `enable_motion`, `geometry_calibrated`, and `enable_udp`: all `false` by
  default. These gates are independent of the reactive-mode default.
- The UDP bridge is not created unless `enable_udp:=true`; when enabled it
  polls at 50 Hz.

The full system maximum is evaluated in shadow. Enforce uses the smaller of
the intent packet's advertised authority and the 0.577350269 system maximum.
Keyboard teleoperation advertises zero and retains direct behavior.

## Legacy conflict audit and resolution

| Legacy problem | Observable failure | Resolution |
|---|---|---|
| A planner server owned a second embedded costmap while mapping also had a standalone costmap | ambiguous publishers, frozen RViz map, lifecycle and remap sensitivity | one standalone costmap from `nav2_mapping.launch.py` |
| Temporary goals and Smac actions continued at 2 Hz in shadow | CPU contention, planner late/abort churn, stale path displays | waypoint node, configuration, executable, topics, and dependencies removed |
| RViz retained four clouds for 1.2 s and rendered at 20 Hz | high RViz CPU, old clouds, TF message-filter drops reported as `Unknown` | current-sample clouds, best-effort QoS, 10 Hz rendering, debug clouds disabled |
| Filtered clouds preserved vendor records in LiDAR frames | larger messages and repeated timestamped TF work in every consumer | compact XYZ output transformed once into `base_link`, source stamp preserved |
| Obstacle RViz profile omitted the robot model | misleading “robot model not parsed” diagnosis despite valid `robot_description` | explicit `/robot_description` RobotModel display |
| UDP node existed while disabled and polled at 100 Hz when enabled | extra graph endpoint and avoidable executor/network work | conditional node and 50 Hz polling |
| Reactive limits differed between launch/config/code documentation | inconsistent steering authority | aligned defaults at 0.577350269 (`tan(30 degrees)`) |

These changes reduce graph ambiguity and stale visualization work. They do not
claim to solve unstable LiDAR power, Ethernet addressing, DDS discovery, or
actual missing static transforms.

## Launch profiles

Non-actuating calculation and RViz (default motion/UDP gates remain closed):

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  use_rviz:=true reactive_assistance_mode:=enforce
```

Shadow comparison without envelope modification:

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  use_rviz:=true reactive_assistance_mode:=shadow
```

Attended physical enforcement requires all gates and the isolated control
addresses explicitly:

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  use_rviz:=true reactive_assistance_mode:=enforce maximum_assist:=0.577350269 \
  enable_motion:=true geometry_calibrated:=true enable_udp:=true \
  bind_address:=192.168.0.102 \
  pi_address:=192.168.0.101 allowed_pi_address:=192.168.0.101
```

The physical gateway must advertise `--max-assist-ratio 0.577350269`. Use the
attended, lowest-speed procedure in
[physical_joystick_shared_control.md](physical_joystick_shared_control.md).

## Validation order

1. Confirm both raw and filtered cloud rates, both successful-filter source
   headers, TF from each LiDAR to `base_link`, and a refreshing inflated map.
2. In open space, CLEAR must not receive a reactive correction.
3. In shadow, place a soft chair at the edge of the requested trajectory.
   SLOW should show candidate arcs and a stable winner after two cycles while
   the envelope steering remains unchanged.
4. Repeat forward-left and forward-right. Corrections may only move toward
   straight. Move the obstacle across both sides to avoid confusing geometry
   with a left/right software bias.
5. Put the chair in STOP range. No candidate may escape or alter STOP.
6. Verify reverse and hard turns match reactive-disabled behavior.
7. Interrupt either source stream and change intent/session. Assistance must
   clear immediately and the direct policy must fail closed where required.
8. Run unoccupied physical enforcement at the lowest speed, with an attendant
   and physical cutoff. Confirm steering stays inside advertised authority and
   longitudinal output remains under the original SLOW cap.

Record `/operator_intent`, both filter source headers, both filtered clouds,
`/nav2_merged_costmap`, `/safety_envelope`, `/shared_control/diagnostics`,
`/shared_control/checked_corridor`, `/shared_control/reactive_candidates`, and
`/shared_control/reactive_suggestion`.

## Remaining work

1. **Footrest/leg filtering (next, intentionally not started):** measure fixed
   geometry with an occupied chair, add evidence-based cuboids to both artifact
   filters, and verify real obstacles near the footrests are retained.
2. **Turn-disc validation:** after self-filtering, replay and physically check
   whether the reduced 0.45 m disc represents the intended turning clearance.
3. **Caster-aware geometry:** measure repeated low-speed swept paths without
   assuming deterministic caster state or adding encoders.
4. **Inflation replay:** compare 0.55, 0.45, 0.40, and 0.35 m on identical
   doorway, pole, chair, and stopping-boundary scenes only after self geometry
   is calibrated.
5. **Reactive tuning:** tune horizon, improvement threshold, steering step,
   and confirmation from recorded evidence. Preserve STOP/SLOW semantics.

## Preserved boundaries

- No ROS wire message, Jetson/Pi UDP protocol, obstacle policy, peer check,
  emergency behavior, or speed-cap change is part of this cleanup.
- Reverse remains unmonitored and capped by the direct policy.
- Hard turns remain controlled by the base-centred cost disc.
- The system does not track a persistent route, assume caster position, add a
  steering/acceleration ramp, or autonomously manoeuvre around STOP.
- Footrest/leg filtering is deliberately deferred until the current runtime is
  stable and its geometry can be measured.
