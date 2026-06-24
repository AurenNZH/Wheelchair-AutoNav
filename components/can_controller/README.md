# CAN Controller Component

Raspberry Pi runtime for keyboard-based remote control of powered wheelchairs via RNET CAN bus over SSH.

This component was migrated from the original single-package repository. The current monorepo paths are:

```text
components/can_controller/
├── README.md
├── requirements.txt
├── scripts/
│   ├── setup_utils.py
│   └── teleoperate_keyboard.py
├── src/
│   └── wheelchair_teleop/
└── tests/

configs/wheelchair/default.yaml
```

Run from the repository root:

```bash
cd components/can_controller
pip install -r requirements.txt
python scripts/setup_utils.py
python scripts/teleoperate_keyboard.py --config ../../configs/wheelchair/default.yaml
```

The older notes below still describe the CAN/RNET behavior and safety model, but some command paths have changed to match the monorepo layout.

## Original Package Notes

Keyboard-based remote control of powered wheelchairs via RNET CAN bus over SSH.

## Overview

This package enables you to teleoperate your powered wheelchair from a remote PC via SSH connection to your Raspberry Pi, using keyboard controls. The Pi acts as a bridge between your PC and the wheelchair's RNET CAN bus.

### Project Context

- **Hardware Setup**: Raspberry Pi 3 with Pi CAN DUO shield, connected to RNET CAN bus
- **Control Method**: Keyboard input over SSH (designed for eventual GUI integration)
- **Protocol**: RNET CAN bus (125 Kbps, extended frames)
- **Status**: Initial release - suitable for controlled testing environments

## Architecture

```
Your PC                Raspberry Pi 3              Wheelchair
┌──────────┐          ┌──────────────┐            ┌──────────┐
│ Keyboard │          │  can0 (Pi    │            │  RNET    │
│   Input  │ ─SSH──> │  CAN DUO)    │ ─CAN bus─> │  Control │
│          │          │              │            │          │
└──────────┘          └──────────────┘            └──────────┘
                           │
                      ./teleoperate_keyboard.py
                      (main CLI program)
```

### Package Components

```
wheelchair_teleop/
├── can_interface.py       # Low-level CAN communication via can-utils
├── keyboard_handler.py    # Non-blocking keyboard input processing
├── joystick_controller.py # Main control logic & CAN frame transmission
├── safety.py              # Safety limits, acceleration ramps, timeouts
├── config.py              # Configuration file handling (YAML)
└── __init__.py            # Package initialization

teleoperate_keyboard.py    # Main CLI entry point
config_default.yaml        # Default configuration template
requirements.txt           # Python dependencies
README.md                  # This file
```

## Installation & Setup

### 1. Prerequisites

**On Raspberry Pi:**
- Raspberry Pi 3 or newer with Pi CAN DUO shield (or equivalent CAN interface)
- Raspbian/Raspberry Pi OS with Python 3.7+
- `can-utils` package for CAN communication
- SSH server enabled (default)

**On Your PC:**
- SSH client (built-in on Linux/Mac, use PuTTY or OpenSSH on Windows)
- Network connectivity to Raspberry Pi

### 2. CAN Interface Setup (Raspberry Pi)

The Pi CAN DUO shield uses SPI-based CAN controller. Verify it's configured:

#### Check current setup:

```bash
# List CAN interfaces
ip link show | grep can

# Check if can0 is up
ifconfig can0
```

#### If CAN interface is NOT UP, configure it:

```bash
# Bring up the interface (temporary - lost on reboot)
sudo ip link set can0 up type can bitrate 125000

# Verify it's up
ifconfig can0

# For permanent configuration, edit /boot/config.txt (or /boot/firmware/config.txt on newer OS)
# Add these lines if not already present:
# dtparam=spi=on
# dtoverlay=mcp2515-can0-overlay,oscillator=16000000,interrupt=25
# dtoverlay=spi-bcm2835-overlay

# Then reboot: sudo reboot
```

#### Test CAN bus connectivity:

```bash
# Monitor CAN traffic (should show frames if wheelchair is powered on)
candump can0 -L

# Should see frames like: 02000100#0000 (joystick centered)
```

### 3. Install Teleoperation Package

**On Raspberry Pi:**

```bash
# Clone or download this package
cd /path/to/wheelchair_teleop

# Install dependencies
pip install -r requirements.txt

# Make main script executable
chmod +x teleoperate_keyboard.py

# Optional: Create symlink for easy access
sudo ln -s $(pwd)/teleoperate_keyboard.py /usr/local/bin/wheelchair-teleop
```

### 4. Network & SSH Setup

**Enable SSH on Raspberry Pi (if not already enabled):**

```bash
# Using Raspberry Pi OS with raspi-config
sudo raspi-config
# Navigate to: Interfacing Options → SSH → Enable
# Or manually:
sudo systemctl enable ssh
sudo systemctl start ssh
```

**From your PC, test SSH connection:**

```bash
# Connect to your Pi
ssh pi@<raspberry_pi_ip>

# Or if using different username/port:
ssh -p 22 username@<raspberry_pi_ip>
```

## Usage

### Quick Start (Local Testing on Pi)

```bash
# On Raspberry Pi (local or via SSH)
cd /path/to/wheelchair_teleop

# Run with default settings
./teleoperate_keyboard.py

# Or with Python
python3 teleoperate_keyboard.py
```

### Remote Operation (Recommended)

```bash
# On your PC
ssh pi@192.168.1.100

# On Pi (in SSH session)
./teleoperate_keyboard.py

# Or with specific CAN interface
./teleoperate_keyboard.py --can-interface can0

# Or with custom config
./teleoperate_keyboard.py --config my_config.yaml
```

### Command-Line Options

```bash
teleoperate_keyboard.py --help

Options:
  --config FILE              Path to YAML configuration file
  --can-interface NAME       CAN interface (default: can0)
  --max-speed PERCENT        Max speed 0-100% (default: 100)
  --no-safety                Disable safety features (NOT recommended)
  --log-level LEVEL          DEBUG, INFO, WARNING, ERROR, CRITICAL
  --log-file FILE            Write logs to file
```

### Keyboard Controls

Once the program is running:

```
MOVEMENT:
  W or ↑ Up         = Move Forward
  S or ↓ Down       = Move Backward
  A or ← Left       = Turn Left  
  D or → Right      = Turn Right

SPEED (0-100%):
  1                 = 20% speed
  2                 = 40% speed
  3                 = 60% speed
  4                 = 80% speed
  5                 = 100% speed

OTHER:
  H                 = Sound Horn
  Space             = EMERGENCY STOP (immediately center joystick)
  Q                 = Quit Program

COMBINING KEYS:
  W + D             = Move forward and turn right
  S + A             = Move backward and turn left
  Pressing forward + backward = Cancelled (no movement)
```

## Configuration

### Using Configuration File

Create `config.yaml`:

```yaml
wheelchair:
  can_interface: can0
  device_slot: 1
  max_speed: 80              # Start at 80% for safety

safety:
  acceleration_rate: 50.0    # How quickly to ramp speed
  inactivity_timeout: 3.0    # Stop after 3 seconds of inactivity
  min_frame_interval_ms: 10.0

control:
  send_interval_ms: 10.0

logging:
  level: INFO
  file: /tmp/wheelchair.log
```

Run with config:

```bash
./teleoperate_keyboard.py --config config.yaml
```

### Key Configuration Options

| Parameter | Default | Recommended | Notes |
|-----------|---------|-------------|-------|
| `max_speed` | 100% | 50-80% | Start low for testing |
| `acceleration_rate` | 50%/s | 30-50%/s | Smoother = higher value |
| `inactivity_timeout` | 5s | 3-5s | Auto-stop if no input |
| `min_frame_interval_ms` | 10ms | 10ms | DO NOT CHANGE - RNET protocol requirement |

## Safety Systems

The package includes several safety features:

### 1. Acceleration Ramping
- No instantaneous maximum speed
- Configurable ramp rate (default: 50%/second)
- Prevents sudden jerking motions

### 2. Inactivity Timeout
- Automatically stops wheelchair if no input received
- Default: 5 seconds
- Can be disabled (not recommended)

### 3. CAN Frame Timing
- Respects RNET protocol requirement (10ms minimum between frames)
- Prevents bus flooding or protocol violations

### 4. Speed Limits
- Configurable maximum speed (0-100%)
- Can be set lower for cautious operation

### 5. Emergency Stop
- Press Space to immediately center joystick
- Sends centering frame to wheelchair

## Testing Procedure

### Phase 1: Verification (Stationary)

```bash
1. Elevate wheelchair (wheels off ground)
2. Connect to Pi via SSH
3. Run: ./teleoperate_keyboard.py --max-speed 20
4. Press '1' to set speed to 20%
5. Press 'W' briefly - wheelchair should move forward slightly
6. Press Space - wheelchair should stop
7. Try other directions (A/S/D)
8. If everything works, proceed to Phase 2
```

### Phase 2: Controlled Movement (Low Speed)

```bash
1. Place wheelchair on ground in empty space
2. Start teleoperation with 40% max speed
3. Move forward slowly in straight line
4. Test reversing (S key)
5. Test turning (A/D keys)
6. Use Space bar emergency stop several times
7. If stable, proceed to Phase 3
```

### Phase 3: Full Operation

```bash
1. Increase max speed incrementally (40% → 60% → 80% → 100%)
2. Test combined movements (forward + turning)
3. Test speed changes while moving (press 5, then 1, etc.)
4. Test longer duration operations
5. Document any issues or unexpected behavior
```

## RNET Protocol Details

### Joystick Position Frame

The wheelchair is controlled via CAN frames with ID `02000100` (extended frame):

```
Frame ID: 0x02000100
Data:     2 bytes (XxYy)

Where:
  Xx = X-axis position (signed int8)
       0x00  = center/stop
       0x64  = +100 (full right)
       0x9C  = -100 (full left)
  
  Yy = Y-axis position (signed int8)
       0x00  = center/stop
       0x64  = +100 (full forward)
       0x9C  = -100 (full reverse)

Timing: Every 10 milliseconds
```

### Frame Examples

```
02000100#0000   Joystick centered (stop)
02000100#6400   Move forward (Y=+100)
02000100#9C00   Move backward (Y=-100)
02000100#0064   Turn right (X=+100)
02000100#009C   Turn left (X=-100)
02000100#3232   Diagonal movement (forward-right)
```

### Speed Control Frame (Optional)

```
Frame ID: 0A040100
Data:     1 byte (Pp)

Where:
  Pp = Speed percentage (0x00-0x64)
  0x00 = 0% (slow)
  0x32 = 50% (medium)
  0x64 = 100% (fast)
```

## Control Methods

This implementation uses the **FollowJSM** method:

1. **What it does**: Sends joystick frames that the wheelchair accepts
2. **Compatibility**: Works when JSM (Joystick Module) is present
3. **Safety**: JSM can still provide steering input if allowed
4. **Timing**: Frames sent every 10ms (RNET protocol requirement)

### Future: Alternative Methods

The framework supports adding alternative control methods:

- **JSMerror**: Force JSM offline, take complete control (more aggressive)
- **EmulateJSM**: Spoof complete JSM startup sequence (requires serial number)

## Logging & Diagnostics

### View Logs

```bash
# On-screen logs (verbosity configurable)
./teleoperate_keyboard.py --log-level DEBUG

# Write to file
./teleoperate_keyboard.py --log-file /tmp/wheelchair.log

# View live logs
tail -f /tmp/wheelchair.log
```

### Monitor CAN Bus

Open another terminal on Pi:

```bash
# View all CAN traffic
candump can0 -L

# View only joystick frames
candump can0 -v | grep "02000100"

# Log to file
candump can0 -l > can_traffic.log

# Analyze saved log
candump -t 0 -l < can_traffic.log
```

### Debug Frame Sending

```bash
./teleoperate_keyboard.py --log-level DEBUG --log-file debug.log

# Check what frames are being sent
grep "Frame sent" debug.log
```

## Troubleshooting

### Problem: "CAN interface not found"

**Solution:**

```bash
# Check if can0 exists
ip link show can0

# If not found, bring it up
sudo ip link set can0 up type can bitrate 125000

# For permanent fix, edit /boot/config.txt and reboot
```

### Problem: SSH timeout during long operations

**Solution:**

```bash
# Add to ~/.ssh/config on your PC
Host raspberry_pi
    HostName 192.168.1.100
    User pi
    ServerAliveInterval 60    # Keep-alive every 60 seconds
    ServerAliveCountMax 10
```

### Problem: Keyboard input not responding

**Solution:**

```bash
# Verify terminal is in raw mode (should see in debug logs)
./teleoperate_keyboard.py --log-level DEBUG

# If using over SSH, ensure terminal is not buffered
ssh -t pi@192.168.1.100 'cd wheelchair_teleop && python3 teleoperate_keyboard.py'
```

### Problem: Wheelchair moves slowly or not at all

**Check:**

```bash
# Verify can0 is transmitting frames
candump can0 -L

# Check if joystick position frame exists (should see 02000100#...)
# If no frames appear, teleoperation is not sending

# Check logs
./teleoperate_keyboard.py --log-level DEBUG

# Verify max-speed is not set too low
./teleoperate_keyboard.py --max-speed 100
```

### Problem: Wheelchair doesn't respond to commands

**Checklist:**

1. Is wheelchair powered on? (indicator lights on)
2. Is CAN bus connected? (candump showing frames)
3. Are teleoperation frames being sent? (DEBUG log shows "Frame sent")
4. Is joystick connected? (wheelchair may prioritize physical joystick)
5. Is wheelchair in drive mode? (not in seating/other mode)

## Performance Considerations

### CAN Bus Timing

- **Joystick Update Rate**: 10ms (100 Hz) - respects RNET protocol
- **Keyboard Poll Rate**: 100ms typical (100 Hz)
- **Speed Ramp Update**: Every 10ms
- **Telemetry Display**: Every 2 seconds

### Network Latency Impact

Over SSH, typical latencies:

- **Local Network**: <10ms - negligible
- **LTE/4G**: 50-100ms - noticeable but acceptable
- **High Latency**: >200ms - may cause control lag

The safety system compensates for latency by:
- Ramping acceleration smoothly
- Maintaining current state during input gaps
- Sending continuous frames (not waiting for acknowledgment)

### CPU Usage

- **Raspberry Pi 3**: ~5-15% CPU usage during operation
- **Memory**: ~20-30 MB (Python runtime + modules)
- **Can share Pi with other processes

## Future Enhancements

Planned features (not yet implemented):

- [ ] GUI interface (PyQt5 or web-based)
- [ ] Real-time telemetry dashboard (speed, position, battery)
- [ ] Control recording/playback for autonomous routes
- [ ] Alternative control methods (JSMerror, EmulateJSM)
- [ ] Gamepad/joystick input support
- [ ] Bluetooth remote control
- [ ] Multi-wheelchair support
- [ ] Configuration persistence and profiles

## Contributing

Found a bug? Have an improvement?

1. Document the issue
2. Include debug logs (`--log-level DEBUG`)
3. CAN bus traces (if possible)

## Safety & Disclaimer

**IMPORTANT**: Power wheelchairs are critical medical devices.

- **Always test in a controlled environment first**
- **Elevate the wheelchair during initial testing**
- **Have a spotter present during operations**
- **Keep a physical emergency stop available**
- **Do not operate in public spaces during testing**
- **Implement additional safety measures as needed**

This software is provided as-is for accessibility research and development. The authors are not responsible for injury, damage, or misuse.

## References

- [RNET Protocol Documentation](https://github.com/redragonx/open-rnet/blob/main/docs/RNET_PROTOCOL_GUIDE.md)
- [Open R-Net Project](https://github.com/redragonx/open-rnet/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [can-utils](https://github.com/linux-can/can-utils)

## License

This package is released under the same license as the Open R-Net project (GPLv3).

## Support

For questions about:

- **RNET Protocol**: See [RNET documentation](https://github.com/redragonx/open-rnet/)
- **This Package**: Check the GitHub issues or documentation
- **Wheelchair Safety**: Consult your wheelchair manufacturer

---

**Built for accessibility. Tested with care. Used responsibly.**
