#!/usr/bin/env python3
"""
Wheelchair Teleoperation - Main CLI Entry Point
Keyboard-based remote control of powered wheelchairs via RNET CAN bus over SSH
"""

import sys
import logging
import argparse
import time
import signal
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from wheelchair_teleop import (
    CANInterface,
    KeyboardHandler,
    JoystickController,
    SafetyManager,
)
from wheelchair_teleop.config import Config


def setup_logging(log_level: str, log_file: str = None):
    """Setup logging configuration."""
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)
    
    log_format = "[%(asctime)s] %(levelname)-8s [%(name)s] %(message)s"
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    
    root_logger = logging.getLogger()
    root_logger.setLevel(numeric_level)
    root_logger.addHandler(console_handler)
    
    # File handler if specified
    if log_file:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(logging.Formatter(log_format))
        root_logger.addHandler(file_handler)


def print_banner():
    """Print startup banner."""
    print("""
╔════════════════════════════════════════════════════════════════╗
║         Wheelchair Teleoperation System v0.1.0                ║
║     Keyboard Control via SSH - RNET CAN Bus Protocol          ║
╚════════════════════════════════════════════════════════════════╝
    """)


def print_controls():
    """Print control information."""
    print("""
KEYBOARD CONTROLS:
─────────────────────────────────────────────────────────────────
Movement:
  W / ↑              - Move Forward
  S / ↓              - Move Backward  
  A / ←              - Turn Left
  D / →              - Turn Right
  Space              - EMERGENCY STOP (center joystick immediately)

Speed Control (0-100%):
  1                  - 20% speed
  2                  - 40% speed
  3                  - 60% speed
  4                  - 80% speed
  5                  - 100% speed

Other:
  H                  - Sound Horn
  Q                  - Quit Program

IMPORTANT SAFETY:
─────────────────────────────────────────────────────────────────
• Always test in a CONTROLLED ENVIRONMENT first
• Have the wheelchair elevated (wheels off ground) during testing
• Keep a physical emergency stop button available
• Ensure a spotter is present during operations
• Start with SPEED PRESET 1 (20%) for initial testing
    """)


def print_telemetry(controller: JoystickController):
    """Print current telemetry."""
    telemetry = controller.get_telemetry()
    
    print("\n┌─ TELEMETRY ──────────────────────────────────────────┐")
    print(f"│ CAN Connected:       {telemetry['can_connected']}")
    print(f"│ Desired Position:    X={telemetry['desired_position']['x']:4d}  Y={telemetry['desired_position']['y']:4d}")
    print(f"│ Actual Position:     X={telemetry['actual_position']['x']:4d}  Y={telemetry['actual_position']['y']:4d}")
    print(f"│ Current Speed:       {telemetry['speed_percent']:3d}%")
    print(f"│ Moving:              {telemetry['is_moving']}")
    print("└───────────────────────────────────────────────────────┘\n", end="")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description="Wheelchair Teleoperation via Keyboard"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path to YAML configuration file",
        default=None
    )
    parser.add_argument(
        "--can-interface",
        type=str,
        default=None,
        help="CAN interface name (default: can0)"
    )
    parser.add_argument(
        "--max-speed",
        type=int,
        default=None,
        help="Maximum allowed speed 0-100%% (default: 100)"
    )
    parser.add_argument(
        "--no-safety",
        action="store_true",
        help="Disable safety features (NOT RECOMMENDED)"
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Logging level"
    )
    parser.add_argument(
        "--log-file",
        type=str,
        default=None,
        help="Log file path"
    )
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging(args.log_level, args.log_file)
    logger = logging.getLogger(__name__)
    
    print_banner()
    
    try:
        # Load configuration
        logger.info("Loading configuration...")
        config = Config(args.config)
        
        # Override with command-line arguments
        if args.can_interface:
            config.data["wheelchair"]["can_interface"] = args.can_interface
        if args.max_speed is not None:
            config.data["wheelchair"]["max_speed"] = args.max_speed
        
        # Initialize components
        logger.info("Initializing wheelchair teleoperation system...")
        
        # CAN Interface
        can_interface = CANInterface(
            can_interface=config.get("wheelchair.can_interface"),
            device_slot=config.get("wheelchair.device_slot")
        )
        
        if not can_interface.connect():
            logger.error("Failed to connect to CAN interface")
            print("ERROR: Could not connect to CAN interface")
            print(f"Make sure {config.get('wheelchair.can_interface')} is up:")
            print(f"  sudo ip link set {config.get('wheelchair.can_interface')} up type can bitrate 125000")
            return 1
        
        # Safety Manager
        safety_config = config.data.get("safety", {})
        safety_manager = SafetyManager(
            max_speed=config.get("wheelchair.max_speed"),
            acceleration_rate=safety_config.get("acceleration_rate", 50.0),
            inactivity_timeout=safety_config.get("inactivity_timeout", 5.0),
            min_frame_interval_ms=safety_config.get("min_frame_interval_ms", 10.0)
        )
        
        if args.no_safety:
            logger.warning("Safety features DISABLED")
            safety_manager.inactivity_timeout = 0
            safety_manager.acceleration_rate = 1000.0
        
        # Joystick Controller
        controller = JoystickController(
            can_interface=can_interface,
            safety_manager=safety_manager,
            send_interval_ms=config.get("control.send_interval_ms", 10.0)
        )
        
        # Keyboard Handler
        keyboard = KeyboardHandler(
            on_joystick_update=controller.set_joystick_position,
            on_speed_change=controller.set_speed,
            on_horn=controller.sound_horn,
            on_stop=lambda: None  # Will handle in signal handler
        )
        
        # Start systems
        logger.info("Starting teleoperation systems...")
        keyboard.start()
        controller.start()
        
        # Print controls
        print_controls()
        print("System ready. Press Q to quit.\n")
        
        # Main loop with telemetry updates
        print_telemetry(controller)
        
        last_telemetry_time = time.time()
        telemetry_update_interval = 2.0  # Update every 2 seconds
        
        try:
            while True:
                current_time = time.time()
                
                # Update telemetry every N seconds
                if current_time - last_telemetry_time >= telemetry_update_interval:
                    print_telemetry(controller)
                    last_telemetry_time = current_time
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received")
        
        except Exception as e:
            logger.error(f"Error in main loop: {e}", exc_info=True)
    
    finally:
        # Cleanup
        logger.info("Shutting down...")
        try:
            keyboard.stop()
        except:
            pass
        
        try:
            controller.stop()
        except:
            pass
        
        try:
            can_interface.disconnect()
        except:
            pass
        
        print("\nTeleoperation system stopped. Wheelchair joystick centered.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
