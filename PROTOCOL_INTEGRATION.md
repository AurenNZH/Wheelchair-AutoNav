# RNET Protocol Integration Guide

This document explains how the wheelchair teleoperation package interfaces with the RNET CAN bus protocol.

## RNET Protocol Overview

The RNET (R-Net) protocol is a proprietary CAN bus protocol used by PG Drives Technology (now Curtiss-Wright) for power wheelchair control.

### Physical Layer

- **Protocol**: CAN 2.0B
- **Bitrate**: 125 Kbps
- **Frame Types**: 
  - Standard (11-bit) - Control commands
  - Extended (29-bit) - Data, serial, configuration
- **Connector**: 4-pin R-Net connector
  - Pin 1: CAN Lo
  - Pin 2: CAN Hi
  - Pin 3: +24VDC
  - Pin 4: GND

## Joystick Control

The primary mechanism for teleoperation is the joystick position frame.

### Frame Structure

```
CAN ID:  0x02000100 (Extended frame)
Period:  Every 10 milliseconds (100 Hz)
Data:    2 bytes - X position, Y position

Byte 0 - X-axis (horizontal):
  0x00           = Center/stop
  0x01-0x63      = Right (+1 to +99)
  0x64           = Full right (+100)
  0xFF-0x9D      = Left (-1 to -99, two's complement)
  0x9C           = Full left (-100)

Byte 1 - Y-axis (vertical):
  0x00           = Center/stop
  0x01-0x63      = Forward (+1 to +99)
  0x64           = Full forward (+100)
  0xFF-0x9D      = Reverse (-1 to -99, two's complement)
  0x9C           = Full reverse (-100)
```

### Package Implementation

In `wheelchair_teleop/can_interface.py`:

```python
def send_joystick_frame(self, x_pos: int, y_pos: int) -> bool:
    """Send joystick position frame"""
    # Map from -100..+100 to CAN byte representation
    x_byte = x_pos & 0xFF  # Python int -> two's complement
    y_byte = y_pos & 0xFF
    
    frame_str = f"02000100#{x_byte:02X}{y_byte:02X}"
    # Send via cansend tool
```

### Example Frames

```
Direction              X Byte  Y Byte  Complete Frame
───────────────────────────────────────────────────────
Centered               0x00    0x00    02000100#0000
Forward (half)         0x00    0x32    02000100#0032
Forward (full)         0x00    0x64    02000100#0064
Reverse (full)         0x00    0x9C    02000100#009C
Right (full)           0x64    0x00    02000100#6400
Left (full)            0x9C    0x00    02000100#9C00
Forward-Right          0x32    0x32    02000100#3232
Reverse-Left           0x9C    0x9C    02000100#9C9C
```

## Control Methods

The package supports different strategies for taking control of the wheelchair.

### Method 1: FollowJSM (Currently Implemented)

**How it works:**
1. Monitor CAN bus for legitimate joystick frames from JSM (0x02000100#...)
2. Immediately after JSM frame arrives, send spoofed frame with our position
3. If sent within ~1ms, the Power Module (PM) accepts it
4. Repeat every 10ms

**Code location**: `wheelchair_teleop/can_interface.py` - `send_joystick_frame()`

**Advantages:**
- JSM can still provide input (graceful coexistence)
- Safest method (user can take control by manipulating JSM)
- Works with JSM present or absent

**Disadvantages:**
- Timing-sensitive (must send within 1ms of JSM frame)
- Occasional control drops if timing fails (rare)

**Implementation note:** Currently, this package sends frames continuously without waiting for JSM frames. This works because the PM accepts continuous stream of joystick frames. For true FollowJSM with JSM monitoring, uncomment the monitoring code in `teleoperate_keyboard.py`.

### Method 2: JSMerror (Optional - Not Yet Implemented)

Trigger a network error on the JSM to make it stop sending frames, then inject our own.

**Advantages:**
- More reliable control (no JSM frames to conflict)
- JSM disabled but can still control speed

**Disadvantages:**
- More aggressive (JSM effectively offline)
- Requires JSM to be present

### Method 3: EmulateJSM (Optional - Not Yet Implemented)

Replay complete JSM startup handshake to impersonate a JSM device.

**Advantages:**
- Works with or without physical JSM
- Complete control

**Disadvantages:**
- Requires known JSM serial number
- More complex authentication

## Speed Control (Optional)

While the teleoperation package focuses on joystick control, speed can be adjusted via separate frame.

### Frame Structure

```
CAN ID:  0x0A040100
Data:    1 byte - Speed percentage

Byte 0 - Speed:
  0x00           = 0% (minimum safe speed)
  0x32           = 50% (medium)
  0x64           = 100% (maximum)
  0x01-0x63      = 1-99%
```

### In Package

```python
def set_speed(self, speed_percent: int) -> bool:
    """Set wheelchair max speed"""
    speed_byte = int(speed_percent * 0x64 / 100)
    frame_str = f"A040100#{speed_byte:02X}"
    # Send via cansend
```

## Safety Integration

The safety manager implements RNET protocol requirements:

### Frame Timing

```
RNET Requirement:
  - Joystick frames every 10ms
  - Jitter: ±2ms acceptable

Package Implementation:
  min_frame_interval_ms: 10.0
  send_interval_ms: 10.0
  
  Enforced via:
  - threading.Thread (control_loop)
  - time.sleep() with precise timing
```

### Acceleration Ramping

RNET devices expect smooth velocity changes (not instantaneous).

```
Package: acceleration_rate parameter
Default: 50%/second
  - At 50% accel, 0→100% takes 2 seconds
  - Prevents sudden motor current spikes
```

### Inactivity Timeout

Safety feature to stop wheelchair if teleoperation input is lost.

```
Package: inactivity_timeout parameter
Default: 5 seconds
  - If no keyboard input for 5s, chair stops
  - Compensates for SSH connection loss
```

## Communication Stack

```
Layer 5 - Application
  └─ User Keyboard Input (W/A/S/D)
       │
Layer 4 - Teleoperation Logic  
  └─ JoystickController (convert input to CAN frames)
       │
       ├─ Speed ramping (SafetyManager)
       ├─ Input validation
       └─ Frame scheduling
       │
Layer 3 - CAN Interface
  └─ CANInterface (format and send frames)
       │
       ├─ Convert position to CAN bytes
       ├─ Build frame string (cansend format)
       └─ Call cansend command
       │
Layer 2 - CAN Bus (SocketCAN)
  └─ Linux SocketCAN (can-utils)
       │
       ├─ candump (monitor frames)
       └─ cansend (transmit frames)
       │
Layer 1 - Physical CAN Bus
  └─ Pi CAN DUO → RNET Connector → Wheelchair CAN Bus
```

## Monitoring & Debugging

### Monitor CAN Bus

```bash
# View all RNET traffic
candump can0 -L

# View just joystick frames
candump can0 | grep "02000100"

# Save to file
candump can0 -l > rnet_traffic.log
```

### Expected Frame Sequence

During teleoperation:

```
# Initial frames when wheelchair powers on:
(various startup handshake frames...)

# Once ready, JSM sends joystick frames every 10ms:
02000100#0000  (centered)
02000100#0000
02000100#0000
...

# When teleoperation active, our frames added to stream:
02000100#3200  (our forward command)
02000100#3200
02000100#6400  (faster)
02000100#6400
...

# When we send speed command:
0A040100#32    (50% speed)

# When centered (stop):
02000100#0000
02000100#0000
```

### Decode Custom Frames

```python
#!/usr/bin/env python3
def decode_joystick_frame(frame_hex):
    """Decode joystick frame"""
    data = bytes.fromhex(frame_hex)
    x = data[0] if data[0] < 128 else data[0] - 256
    y = data[1] if data[1] < 128 else data[1] - 256
    return x, y

# Example
x, y = decode_joystick_frame("3232")
print(f"Position: X={x}, Y={y}")  # Output: Position: X=50, Y=50
```

## Protocol Compatibility

### What We Implement
- ✓ Joystick position frame (02000100)
- ✓ Speed control frame (0A040100)
- ✓ Horn command (0C0401XX)
- ✓ CAN frame timing (10ms)
- ✓ Multi-axis control (X+Y simultaneous)

### What We Don't Implement
- ✗ Serial authentication (not needed for FollowJSM)
- ✗ Parameter exchange (0x78X/0x79X)
- ✗ Device enumeration
- ✗ Configuration memory read/write
- ✗ Programmer protocol

### Interaction with JSM

```
Physical Joystick ──┐
                    ├──→ [PM] Wheelchair Controller
RNET Teleoperation ─┘
                    
The PM accepts joystick commands from either source.
With FollowJSM:
  - Both JSM and teleoperation can send frames
  - PM takes latest/fastest frame
  - User has priority (physical joystick override)
```

## Fault Handling

### CAN Frame Send Fails

```python
# In can_interface.py:
try:
    subprocess.run(["cansend", ...], timeout=1)
except Exception as e:
    logger.error(f"Failed to send: {e}")
    return False
```

If frame sending fails:
1. Logged at ERROR level
2. Teleoperation continues (doesn't crash)
3. Next frame sent on next cycle

### CAN Interface Down

```python
# Checked before sending
if not self.is_connected:
    logger.error("Not connected to CAN interface")
    return False
```

Recovery:
1. User must fix CAN interface
2. Restart teleoperation program
3. Program verifies connection on startup

### SSH Connection Loss

The safety manager handles this:
```python
# Inactivity timeout triggers if:
# - SSH connection drops
# - Keyboard input stops
# - 5 seconds pass without input
```

Result: Wheelchair stops automatically (safe failure)

## Future Protocol Extensions

Placeholder for adding new RNET features:

```python
# In can_interface.py - ready to add:

def set_lights(self, lights_bitmap: int) -> bool:
    """Turn on lights (0C0004XX)"""
    pass

def get_battery_level(self) -> int:
    """Read battery (0x1C0C0100)"""
    pass

def change_mode(self, mode: int) -> bool:
    """Switch drive mode (0x061)"""
    pass

def read_parameters(self, pointer: int, subpointer: int) -> bytes:
    """Read config via parameter exchange"""
    pass
```

## References

For detailed protocol specification, see:
- https://github.com/redragonx/open-rnet/blob/main/docs/RNET_PROTOCOL_GUIDE.md
- https://github.com/redragonx/open-rnet/blob/main/CLAUDE.md#rnet-protocol-notes

## Security Considerations

⚠️ **IMPORTANT SECURITY NOTES:**

1. **No Encryption**: All RNET frames are in plaintext
   - Anyone with CAN access can see/replay frames
   - No authentication beyond serial number XOR

2. **Weak Authentication**: XOR-based (not cryptographic)
   - Challenge values are ignored
   - Serial numbers can be extracted from captures

3. **No Signing**: Firmware and frames not signed
   - Can't verify frames are legitimate
   - Anyone with CAN adapter can inject commands

4. **Implication**: This teleoperation system has the same security level as the original JSM
   - Use only on trusted networks
   - Don't expose CAN bus over internet
   - SSH provides network-level security

## Conclusion

This teleoperation package cleanly integrates with RNET protocol:
- Uses standard joystick frame format
- Respects protocol timing requirements
- Implements safety systems
- Provides clean Python abstraction

The modular design allows easy extension for future RNET features while maintaining stability and safety.
