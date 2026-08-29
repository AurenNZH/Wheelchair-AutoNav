# Jetson–Pi Safety Link

Version 3 is a fail-safe UDP heartbeat between the Raspberry Pi keyboard/CAN
process and the Jetson ROS 2 supervisor. It carries operator intent to the
Jetson and a permitted envelope back to the Pi. The envelope wire shape is
unchanged; it remains a single bounded command, never a trajectory.

- Pi-to-Jetson intent: UDP `45450`
- Jetson-to-Pi envelope: UDP `45451`
- Heartbeat: 20 Hz
- Envelope timeout: 200 ms
- Required non-STOP responses before motion: five distinct sequences

Intent example:

```json
{"v":3,"type":"intent","session":"UUID","seq":12,"lateral":0.0,"longitudinal":0.5,"max_steering_assist":0.15,"intent_class":1,"deadman":true}
```

Envelope example:

```json
{"v":3,"type":"envelope","session":"UUID","intent_seq":12,"decision":2,"permitted_forward":0.5,"permitted_steering":0.1,"reason":"nav2_avoidance_cost_clear","map_age_ms":42.0}
```

`max_steering_assist` is explicit operator authority in normalized steering
ratio. Zero is the default and preserves direct shared control, including
keyboard teleoperation. For straight forward intent, non-zero authority permits
the supervisor to choose either side within the cap. For forward-left or
forward-right, it permits only a reduction toward straight by at most the cap;
it cannot cross zero or amplify the requested turn. Reverse and hard turns do
not use path assistance. The Pi validates this contract independently.

In live Nav2 mode, `map_age_ms` is monotonic costmap receipt age. Filtered
LiDAR acquisition age remains a separate Jetson diagnostic and safety gate.
Decisions are `0=STOP`, `1=SLOW`, and `2=CLEAR`. ROS steering is left-positive;
the Pi keyboard/CAN convention is right-positive, so the Pi adapter negates
steering in both directions.

Both ends validate packet size, exact version, type, finite bounds, peer
address, session, sequence, semantic intent, and steering authority. Mixed
v2/v3 deployments, missing fields, malformed or stale packets, wrong sessions,
and wrong senders fail closed.

The physical-JSM workflow can automatically recover exactly
`nav2_cost_stop` and `nav2_turn_cost_stop`: held input stays at zero until five
fresh, distinct, matching SLOW/CLEAR envelopes arrive, and any STOP resets the
count. Other STOP reasons, invalid envelopes, explicit emergency stops, and
keyboard shared control remain latched until operator release.

The protocol is unauthenticated. Use fixed addresses on an isolated trusted
router LAN and never expose the ports to an untrusted network.
