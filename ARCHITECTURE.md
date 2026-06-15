# Architecture & Design Document

This document describes the architecture and design decisions of the wheelchair teleoperation package.

## Design Philosophy

1. **Modular**: Each component has a single responsibility
2. **Extensible**: Easy to add new features (GUI, new control methods)
3. **Safe**: Multiple layers of safety validation
4. **Debuggable**: Comprehensive logging for troubleshooting
5. **SSH-Friendly**: Works well over remote SSH connections

## Component Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   teleoperate_keyboard.py                    │
│                 (Main CLI entry point)                       │
└────────┬──────────────────────────────────────────────────┬──┘
         │                                                  │
         ▼                                                  ▼
    ┌─────────────────┐                          ┌──────────────────┐
    │ KeyboardHandler │                          │ JoystickController│
    │                 │                          │                  │
    │ - Non-blocking  │                          │ - Main control   │
    │   input capture │                          │   loop           │
    │ - Key mapping   │                          │ - Frame timing   │
    │ - Callbacks     │                          │ - Coordination   │
    └────────┬────────┘                          └────────┬─────────┘
             │                                           │
             │ on_joystick_update                        │
             │ on_speed_change                           │ send_joystick_frame()
             │ on_horn                                   │
             │ on_stop                                   │
             │                                           │
             └───────────────────┬─────────────────────┬─┘
                                 │                     │
                    ┌────────────┘                     │
                    │                                  │
                    ▼                                  ▼
             ┌─────────────────────┐         ┌──────────────────┐
             │  SafetyManager      │         │  CANInterface    │
             │                     │         │                  │
             │ - Speed ramping     │         │ - CAN connection │
             │ - Inactivity check  │         │ - Frame formatting
             │ - Accel limits      │         │ - Command sending │
             │ - Input validation  │         │ - Bus monitoring │
             └────────────┬────────┘         └────────┬─────────┘
                          │                           │
                          └──────────────────┬────────┘
                                             │
                                    ┌────────▼────────┐
                                    │   Config        │
                                    │                 │
                                    │ - Load YAML     │
                                    │ - Provide values│
                                    │ - Defaults      │
                                    └─────────────────┘
```

## Control Flow

### Initialization Sequence

```
main()
  ├─ Parse arguments
  ├─ Setup logging
  ├─ Create Config
  ├─ Create CANInterface
  │   └─ Check can0 exists and is UP
  ├─ Create SafetyManager
  ├─ Create JoystickController
  │   └─ Bind to CANInterface & SafetyManager
  ├─ Create KeyboardHandler
  │   └─ Setup callbacks for joystick/speed/horn
  └─ Start all components:
      ├─ keyboard.start()
      │   └─ Start _monitor_loop() in thread
      ├─ controller.start()
          └─ Start _control_loop() in thread
```

### Runtime Operation (Per Cycle)

```
KEYBOARD THREAD (responds to input):
  1. Read single character
  2. Parse key (W/A/S/D/1-5/H/Q/Space)
  3. Update _keys_pressed set
  4. Calculate desired X,Y position
  5. Call on_joystick_update(x, y)
     └─ JoystickController.set_joystick_position(x, y)
        └─ SafetyManager.record_input()

CONTROL LOOP THREAD (every 10ms):
  1. Check if 10ms elapsed since last send
  2. If yes:
     a. SafetyManager.check_inactivity()
     b. SafetyManager.set_target_speed(current_speed)
        └─ Ramp toward target speed
     c. Apply speed scaling to position
     d. CANInterface.send_joystick_frame(x, y)
        └─ Format and send CAN frame
     e. Update telemetry
  3. Sleep 1ms
  4. Repeat

MAIN THREAD (every 2 seconds):
  1. Get telemetry from controller
  2. Display on screen
  3. Sleep 100ms
  4. Repeat
```

### Shutdown Sequence

```
Signal handler (Ctrl+C) or Q key:
  ├─ KeyboardHandler.stop()
  │   └─ Stop _monitor_loop() thread
  ├─ JoystickController.stop()
  │   ├─ Stop _control_loop() thread
  │   └─ Send centering frame (02000100#0000)
  ├─ CANInterface.disconnect()
  └─ Exit main loop
```

## Threading Model

The package uses three threads:

### Thread 1: Keyboard Monitor (Daemon)

```python
# In KeyboardHandler._monitor_unix()
# or KeyboardHandler._monitor_blocking()

Purpose: Capture keyboard input
  - Runs in raw mode (Unix/Linux) for instant feedback
  - Or blocking mode (Windows)
  - Calls _handle_key() for each character

Frequency: Event-driven (one key = one response)
Safety: Daemon thread - exits if main dies
```

### Thread 2: Control Loop (Daemon)

```python
# In JoystickController._control_loop()

Purpose: Send CAN frames at precise intervals
  - Runs every 10ms (matching RNET protocol)
  - Applies safety ramps
  - Coordinates with SafetyManager
  - Calls CANInterface.send_joystick_frame()

Frequency: 100 Hz (every 10ms)
Safety: Daemon thread - exits if main dies
```

### Thread 3: Main (Non-Daemon)

```python
# main() function in teleoperate_keyboard.py

Purpose: Display telemetry and coordinate shutdown
  - Runs telemetry display loop
  - Handles Ctrl+C gracefully
  - Triggers cleanup

Frequency: Every 100ms
Safety: Main thread - all cleanup happens here
```

### Thread Synchronization

Shared state access:

```
KeyboardHandler._keys_pressed
  ├─ Read by: _update_joystick_position() [keyboard thread]
  ├─ Write by: _handle_key() [keyboard thread]
  └─ Access: Set (thread-safe in Python GIL)

JoystickController._desired_x/y
  ├─ Write by: set_joystick_position() [main thread callback]
  ├─ Read by: _control_loop() [control thread]
  └─ Access: Simple assignment (atomic in Python)

SafetyManager._current_speed
  ├─ Write by: _control_loop() [control thread]
  ├─ Read by: _control_loop() [same thread]
  └─ Access: Exclusive

CANInterface.is_connected
  ├─ Set by: connect() [main thread]
  ├─ Read by: send_joystick_frame() [control thread]
  └─ Access: Simple read (atomic boolean)
```

Note: Python's Global Interpreter Lock (GIL) ensures atomicity of simple operations.

## Error Handling Strategy

### Layer 1: Input Validation

```python
# In SafetyManager.validate_joystick_input()
x = max(-100, min(100, x))  # Clamp
y = max(-100, min(100, y))  # Clamp

# Prevents invalid positions from ever reaching CAN
```

### Layer 2: Command Execution

```python
# In CANInterface.send_joystick_frame()
try:
    subprocess.run(["cansend", ...], timeout=1)
    return True
except Exception as e:
    logger.error(f"Failed to send CAN frame: {e}")
    return False

# Catches and logs failures, allows graceful degradation
```

### Layer 3: State Recovery

```python
# In main() finally block
try:
    keyboard.stop()
    controller.stop()
    can_interface.disconnect()
except:
    pass  # Ignore errors during shutdown

# Ensures cleanup happens even if components fail
```

## Performance Considerations

### CAN Bus Bandwidth

```
Joystick frame: 8 bytes = 64 bits
Sent at 10ms = 100 Hz
Bandwidth: 100 * 64 = 6,400 bits/second

CAN bus capacity at 125 Kbps = 125,000 bits/second
Usage: 6,400 / 125,000 = 5.1%

Safe margin: 94.9% available for other devices
```

### CPU Usage (Pi 3)

```
Keyboard thread:
  - Blocked waiting for input
  - No CPU when idle
  - ~1% CPU when processing input

Control loop thread:
  - Sleeps 99% of the time (1ms sleep every 10ms)
  - ~3% CPU when active

Main thread:
  - Sleeps most of time
  - ~1% CPU for telemetry display

Total: ~5-10% CPU usage during operation
```

### Memory Usage

```
Python interpreter: ~10 MB
Imported modules: ~5 MB
Objects & buffers: ~5 MB
Total: ~20-30 MB resident

Minimal footprint - can share Pi with other processes
```

## Extensibility Points

### 1. Add New Keyboard Controls

```python
# In KeyboardHandler._handle_key()
elif key_lower == 'n':
    logger.info("New feature activated")
    if self.on_new_feature:
        self.on_new_feature()

# In main(), set callback:
keyboard.on_new_feature = controller.new_feature_method
```

### 2. Add GUI Interface

```python
# Create GUI thread that calls:
controller.set_joystick_position(x, y)
controller.set_speed(speed)
controller.sound_horn()

# Get state from:
telemetry = controller.get_telemetry()
```

### 3. Add Alternative Control Method

```python
# In can_interface.py, add:
def send_jsmerror_frame(self):
    """Trigger JSMerror control method"""
    # Send error frame to JSM
    self.cansend(self.can_interface, "0C000100#")

# Or add alternative frame sending:
def send_custom_frame(self, frame_id, data):
    """Send any custom CAN frame"""
    frame_str = f"{frame_id:X}#{data}"
    self.cansend(self.can_interface, frame_str)
```

### 4. Add Telemetry Monitoring

```python
# Create monitoring thread:
def monitor_can_bus(self):
    process = subprocess.Popen(
        ["candump", self.can_interface],
        stdout=subprocess.PIPE
    )
    # Parse frames and update telemetry

# Log statistics:
- Battery level
- Motor current
- Distance traveled
- Actual wheelchair acceleration
```

## Testing Strategy

### Unit Testing

```python
# Tests for SafetyManager
def test_acceleration_ramp():
    mgr = SafetyManager(acceleration_rate=100)
    speed = mgr.set_target_speed(100)
    # Should ramp up, not instant
    assert speed < 100

# Tests for input validation
def test_position_clamping():
    x, y = safety_mgr.validate_joystick_input(150, -150)
    assert -100 <= x <= 100
    assert -100 <= y <= 100
```

### Integration Testing

```
1. Verify CAN interface connectivity
2. Send test frames
3. Monitor wheelchair response
4. Check telemetry updates
5. Test emergency stop
6. Verify inactivity timeout
```

### System Testing

```
1. Controlled movement tests
2. Speed ramp verification
3. Timeout behavior
4. SSH lag simulation
5. Extended operation (30+ minutes)
```

## Future Improvements

### Short-term

- [ ] Gamepad/joystick input support
- [ ] Multiple speed profiles (conservative/normal/sport)
- [ ] Configuration profiles (one-touch presets)
- [ ] Frame transmission statistics
- [ ] Wheelchair telemetry display (battery, motor current)

### Medium-term

- [ ] Web GUI interface
- [ ] Multi-wheelchair support
- [ ] Recording and playback of routes
- [ ] Voice control integration
- [ ] Mobile app (if using remote control over network)

### Long-term

- [ ] Integration with ROS2 navigation
- [ ] Autonomous route following
- [ ] Obstacle detection integration
- [ ] Machine learning for user adaptation
- [ ] Real-time video streaming + control

## Design Decisions & Rationale

### Decision: Why use `can-utils` (cansend/candump)?

**Chosen: Subprocess calls to can-utils**

**Alternatives considered:**
1. Direct SocketCAN using Python socket library
2. python-can library
3. CAN kernel driver raw access

**Rationale:**
- can-utils is lightweight and available on all Linux systems
- No additional Python dependencies needed (besides PyYAML)
- Well-tested and reliable
- Easy to debug (candump shows exact frames being sent)
- Works across all CAN interfaces (not driver-specific)

### Decision: Why threading instead of asyncio?

**Chosen: Python threading with daemon threads**

**Alternatives considered:**
1. asyncio (coroutines)
2. multiprocessing (separate processes)
3. Single-threaded with select() loop

**Rationale:**
- Simpler to understand and debug
- Each component is independent
- Keyboard input requires blocking operations
- CAN operations are fast and don't need async
- Smaller memory footprint than multiprocessing

### Decision: Why keyboard input over SSH?

**Chosen: TTY raw mode + event-driven input**

**Rationale:**
- Works over any SSH connection
- No GUI/display server needed
- Responsive feedback (100Hz polling)
- Fallback to blocking mode for Windows
- No external dependencies

### Decision: Why safety ramping?

**Chosen: Gradual speed acceleration**

**Rationale:**
- Wheelchair motors need time to spool up (avoids stalling)
- Smooth motion feels safer and more controlled
- Matches physical joystick behavior
- Reduces current draw spikes (good for battery)
- Essential for smooth video streaming (if added later)

## Conclusion

The architecture emphasizes:
- **Clarity**: Each component has one job
- **Safety**: Multiple validation layers
- **Robustness**: Handles failures gracefully
- **Extensibility**: Easy to add features
- **Performance**: Minimal CPU/memory impact

The design supports the current keyboard teleoperation use case while providing a foundation for future GUI and autonomous features.
