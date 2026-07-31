# Post-MVP Roadmap

The August 31 MVP is supervised shared control: an operator requests forward
or bounded-right motion and the system may permit, slow, or stop it.

Deferred work:

- short-distance autonomous route selection around basic indoor obstacles;
- localization and odometry validation suitable for Nav2 or an equivalent
  local planner;
- a separate arbitration interface for operator and autonomous intent;
- left-side sensing for symmetric turning;
- human-aware perception and keep-out zones;
- camera/LiDAR fusion after the LiDAR-only safety baseline passes.

Autonomy must not publish messages pretending to be live operator intent.
It requires an explicit intent source, arbitration state, cancellation path,
and the same fail-closed safety envelope used by supervised control.

