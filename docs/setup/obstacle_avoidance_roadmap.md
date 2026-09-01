# Reactive Obstacle Assistance Roadmap

This is the authoritative scope and deployment order for obstacle assistance.
The primary feature is now bounded, reactive steering during an existing SLOW
decision. Nav2 temporary-waypoint planning remains available at 2 Hz in shadow
mode for research comparison, but it cannot alter a `SafetyEnvelope`.

The design reflects the wheelchair's real strengths: the operator can make
precise corrections and the chair is highly manoeuvrable, while deterministic
odometry and repeatable caster state are unavailable. The system therefore
does not guess a destination, follow a stored path, or steer around a STOP.

## Runtime pipeline

```text
dual L2 clouds -> support/self filtering -> base_link Nav2 costmap
                                              |
physical joystick -> OperatorIntent -> direct arc/disc policy
                                              |
                           STOP --------> unchanged STOP
                           CLEAR -------> unchanged CLEAR
                           SLOW
                             -> individual 1.2 m candidate arcs
                             -> bounded rank + two-cycle confirmation
                             -> shadow suggestion, or steering-only enforcement
                                              |
                  original SLOW cap/reason/evidence -> SafetyEnvelope -> Pi

OperatorIntent -> 2 Hz temporary waypoint -> Nav2 path diagnostics
                                           -> shadow suggestion only
```

The direct 20 Hz supervisor remains authoritative. Reverse and hard left/right
never enter reactive selection. A direct STOP never triggers an escape search
and a direct CLEAR is not modified. When assistance is enforced, only
`permitted_steering` changes: the decision stays SLOW, its speed cap remains,
and its reason remains exactly `nav2_cost_slow`.

## Implemented reactive model

Reactive assistance is eligible only for forward, forward-left, and
forward-right intent with fresh map/source evidence and a direct
`nav2_cost_slow` result.

- Straight intent evaluates the current steering and both sides up to the
  current authority.
- Forward-left evaluates only from the requested left correction toward zero;
  it cannot cross zero or increase the turn. Forward-right is symmetric.
- The fan uses 0.05 steering increments and always includes the source and the
  exact authority boundary. The system maximum correction is 0.15.
- Each candidate is one individual 1.2 m arc sampled every 0.05 m. This is
  intentionally separate from the swept union retained by the direct policy.
- Unknown cells, leaving the map, or any cost of 99 or more anywhere on the
  candidate rejects that candidate.
- Candidates rank by CLEAR over SLOW, lower maximum cost, farther first
  inflated cost, lower accumulated cost, smaller steering change, the previous
  side, then left-positive as the deterministic straight-intent tie-break.
- SLOW-to-CLEAR always qualifies. SLOW-to-SLOW requires at least a five-cost
  reduction. Corrections below 0.02 are ignored.
- The same correction direction must win for two consecutive supervisor
  cycles. STOP, CLEAR, release, stale/invalid evidence, session/class changes,
  or a steering change over 0.05 reset confirmation immediately.

Shadow mode evaluates the full 0.15 system range even if packet authority is
zero. Enforce mode uses the smaller of packet authority and the 0.15 system
maximum. No acceleration or software steering ramp is added.

## Modes, topics, and diagnostics

The two assistance systems are deliberately named and configured separately:

- `reactive_assistance_mode`: `disabled`, `shadow`, or `enforce`; default
  `disabled`.
- `nav2_waypoint_mode`: `disabled` or `shadow`; default `shadow` in the
  obstacle-assistance launch. `enforce` is rejected during launch.
- `/shared_control/reactive_suggestion`: current local selector output.
- `/shared_control/nav2_waypoint_suggestion`: research waypoint output.
- `/shared_control/reactive_candidates`: transient-local RViz markers.

RViz renders the requested candidate in white, alternatives in grey, rejected
arcs in red, a pending winner in yellow, and a confirmed winner in cyan. Nav2
path and temporary-goal displays are labelled as waypoint shadow evidence.

`/shared_control/diagnostics` includes the mode/status, intent sequence,
requested and selected steering, advertised and applied authority, candidate
and rejection counts, maximum and accumulated costs, first inflated distance,
cost improvement, confirmation count, selector time, and cumulative suggestion
and enforcement counters. Existing decision, cost, and freshness evidence is
unchanged.

## Launch profiles

Research comparison with reactive shadow and Nav2 waypoint shadow:

```bash
cd /home/jetson-xavier-wheelchair/Wheelchair-AutoNav
source /opt/ros/foxy/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  reactive_assistance_mode:=shadow nav2_waypoint_mode:=shadow \
  nav2_waypoint_rate_hz:=2.0 use_rviz:=true
```

Low-latency reactive shadow without Nav2 waypoint requests:

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  reactive_assistance_mode:=shadow nav2_waypoint_mode:=disabled \
  use_rviz:=true
```

Reactive enforcement must remain an explicit later step:

```bash
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  reactive_assistance_mode:=enforce nav2_waypoint_mode:=shadow \
  enable_motion:=true geometry_calibrated:=true maximum_assist:=0.30
```

The physical gateway must also advertise `--max-assist-ratio 0.30`. Keyboard
teleoperation advertises zero authority and retains direct behavior.

## Validation sequence

1. Run package tests and confirm the selector p95 is below 10 ms and worst case
   below 25 ms on the Jetson.
2. Run reactive shadow in open space. Direct CLEAR must produce no correction.
3. Approach an office chair until direct SLOW. Confirm candidate markers and a
   stable suggested side after two cycles; the envelope must remain unchanged.
4. Repeat with forward-left and forward-right. A suggestion may only reduce
   the requested correction toward straight.
5. Put the chair inside STOP distance. No reactive candidate may be applied or
   used to escape STOP.
6. Verify reverse and hard-turn envelopes are identical with reactive mode
   disabled and shadow.
7. Stop either filter/source stream and change session, class, and steering.
   Each event must clear candidates and reset confirmation.
8. Replay the same bags in enforce mode without a passenger, at the lowest
   speed, with an attendant and physical cutoff. The applied envelope must
   retain SLOW speed and reason, respect authority, and move only after two
   matching cycles.

Record at least the intent, both source headers, merged costmap, safety
envelope, shared-control diagnostics, checked corridor, reactive candidates,
both suggestion topics, Nav2 goal/path, and planner diagnostics.

## Remaining work, in order

1. **Footrest/self geometry:** measure the fixed footrest/leg cuboid, add it to
   both L2 artifact filters, and include its XY projection in the footprint.
2. **Caster-aware geometry:** use repeated low-speed floor/video measurements
   to replace the nominal centreline assumption with a measured swept region.
   No caster encoders or deterministic odometry are required.
3. **Inflation replay:** the deployed default was reduced from 0.55 m to
   0.45 m for reactive-assistance testing. Replay 0.55, 0.45, 0.40, and
   0.35 m against identical doorway, narrow-pole, chair, and stopping-boundary
   scenes after footprint and swept checks are calibrated. Do not select
   0.20 m from subjective clearance alone.
4. **Reactive tuning:** tune horizon, improvement threshold, and confirmation
   only from shadow/replay evidence. Preserve direct STOP and SLOW semantics.
5. **Nav2 research:** keep waypoint planning isolated at 2 Hz. Planner radius,
   goal generation, unknown handling, and search optimisation may be studied,
   but waypoint output must remain shadow-only unless a separate future scope
   explicitly reintroduces autonomous destination selection and odometry.

## Preserved boundaries

- No ROS message, Pi/Jetson UDP protocol, peer check, emergency behavior,
  speed-cap, or obstacle-STOP recovery change is part of this feature.
- Reverse remains unmonitored and capped by the existing direct policy.
- Hard turns remain controlled by the base-centred cost disc.
- The costmap remains at 0.1 m and the current inflation default is 0.45 m.
- The feature never produces longitudinal motion, tracks a persistent path,
  assumes caster state, or automatically manoeuvres around a direct STOP.
