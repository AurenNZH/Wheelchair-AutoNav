# Wheelchair Obstacle Avoidance

This package launches the shared-control costmap and two separate experiments:
the supervisor's low-latency reactive steering and the older Nav2 temporary
waypoint planner. The reactive selector is the intended assistance model. Nav2
is retained at 2 Hz in shadow mode for research evidence and cannot alter a
`SafetyEnvelope`.

The planner uses the current 5 m by 8 m `base_link` costmap (4,000 cells), a
Dubins forward-only model, 1.2 m minimum turning radius, 36 heading bins, and a
30 ms search budget. For initial shadow validation, end-to-end planner results
arriving after 300 ms are discarded; this is a validation tolerance rather
than the latency target. Paths must stay within 0.8 m of the joystick ray, be
no longer than 1.25 times the direct route, make no reverse progress, and
finish within 0.2 m of the goal.

Reactive modes are `disabled` (default), `shadow`, and `enforce`.
`nav2_waypoint_mode` accepts only `disabled` and `shadow`; its default is
`shadow`. Start both shadow systems with:

```bash
sudo apt-get install ros-foxy-nav2-planner ros-foxy-smac-planner
ros2 launch wheelchair_obstacle_avoidance obstacle_avoidance.launch.py \
  reactive_assistance_mode:=shadow nav2_waypoint_mode:=shadow \
  nav2_waypoint_rate_hz:=2.0 discard_after_ms:=300.0 \
  planner_search_budget_ms:=30.0
```

To validate only the reactive latency path, set
`nav2_waypoint_mode:=disabled`. The costmap still runs because it is the
reactive selector's input.

`planner_search_budget_ms` is passed to both Smac's
`max_planning_time_ms` setting and the diagnostic classifier. This keeps the
reported budget aligned with the one Nav2 actually uses.

Planner result diagnostics distinguish total ROS action latency from Nav2's
reported computation time. `planning_time_ms` remains the monotonic interval
from request submission to result receipt, while `nav2_planning_time_ms` is
read from `ComputePathToPose.Result.planning_time` and is `none` when no result
was returned. `planner_action_status` records the terminal action state.
Aborted and cancelled actions, successful empty paths, and true frame errors
are reported respectively as `planner_aborted`, `planner_canceled`,
`planner_empty_path`, and `path_frame_mismatch`; an aborted default path is not
misreported as a frame error.

The suggestion reason remains `planner_aborted` because Foxy's
`ComputePathToPose` result does not expose a reliable cause code. The separate
`abort_hint` diagnostic reports request-time evidence in this precedence:
unavailable costmap or footprint, start footprint outside/collision/unknown,
goal footprint outside/collision/unknown, likely search-budget use, or
`no_path_or_budget_unknown`. `search_budget_likely` is used only when Nav2's
positive internal `planning_time` reaches 90% of the configured search budget;
round-trip action latency is never used to infer this hint.

Each diagnostic also freezes the goal pose, map timestamp and receipt age,
goal-centre cost, start and transformed-goal footprint cell summaries, and
cumulative intent/request/result/path-clear counters. Missing evidence is
reported as `unavailable`, never as clear.

The supplied RViz configuration shows the raw `/plan` in orange, the accepted
`/local_avoidance/path` in green, the temporary goal in magenta, and the Nav2
footprint in cyan. The accepted path is cleared with one empty `Path` after an
invalid result, ineligible intent, or materially changed steering request.
Nav2 owns `/plan`, so an abort cannot clear it; its orange path must be treated
as the **last successful raw path**, not necessarily the current result.

The reactive selector runs only after a direct `nav2_cost_slow` decision. It
compares individual 1.2 m arcs, rejects unknown/outside/cost-99 paths, and
requires two matching correction directions. Forward-left and forward-right
may only reduce the requested correction toward straight. Direct STOP and
CLEAR are never modified. Enforcement changes steering only and keeps the
original SLOW cap, reason, and evidence.

The operator gateway must advertise non-zero authority before reactive
enforcement can alter steering. Shadow mode deliberately evaluates the 0.15
system range even with zero advertised authority. See the
[reactive roadmap](../../../docs/setup/obstacle_avoidance_roadmap.md) for
topics, RViz colours, diagnostics, and validation order.
