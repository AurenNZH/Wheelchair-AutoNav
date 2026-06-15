"""
Joystick Controller Module
Main teleoperation logic coordinating keyboard input, CAN communication, and safety
"""

import logging
import time
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class JoystickController:
    """
    Main teleoperation controller.
    
    Coordinates:
    1. Keyboard input -> desired joystick position
    2. Desired position + safety limits -> ramped actual position
    3. Actual position -> CAN frame transmission
    """
    
    def __init__(self, can_interface, safety_manager, 
                 send_interval_ms: float = 10.0):
        """
        Initialize joystick controller.
        
        Args:
            can_interface: CANInterface instance
            safety_manager: SafetyManager instance
            send_interval_ms: Time between CAN frame sends (RNET requires 10ms)
        """
        self.can_interface = can_interface
        self.safety_manager = safety_manager
        self.send_interval_ms = send_interval_ms / 1000.0  # Convert to seconds
        
        self._desired_x = 0
        self._desired_y = 0
        self._current_x = 0
        self._current_y = 0
        self._current_speed = 50
        
        self._send_thread = None
        self._stop_sending = False
        self._is_running = False
        
        logger.info(f"JoystickController initialized with {send_interval_ms}ms send interval")
    
    def start(self):
        """Start the teleoperation control loop."""
        if self._is_running:
            logger.warning("Control loop already running")
            return
        
        self._stop_sending = False
        self._is_running = True
        
        self._send_thread = threading.Thread(
            target=self._control_loop,
            daemon=True
        )
        self._send_thread.start()
        logger.info("Joystick controller started")
    
    def stop(self):
        """Stop the control loop and center the joystick."""
        self._stop_sending = True
        if self._send_thread:
            self._send_thread.join(timeout=2)
        
        # Send centering frame
        self.can_interface.send_joystick_frame(0, 0)
        self._is_running = False
        logger.info("Joystick controller stopped")
    
    def set_joystick_position(self, x: int, y: int):
        """
        Update desired joystick position from keyboard input.
        
        Args:
            x: X-axis position (-100 to +100)
            y: Y-axis position (-100 to +100)
        """
        self._desired_x, self._desired_y = self.safety_manager.validate_joystick_input(x, y)
        self.safety_manager.record_input()
    
    def set_speed(self, speed_percent: int):
        """
        Update current speed setting.
        
        Args:
            speed_percent: Speed (0-100%)
        """
        self._current_speed = max(0, min(100, speed_percent))
        logger.debug(f"Speed set to {self._current_speed}%")
    
    def sound_horn(self):
        """Sound the wheelchair horn."""
        try:
            threading.Thread(
                target=self.can_interface.sound_horn,
                daemon=True
            ).start()
        except Exception as e:
            logger.error(f"Failed to sound horn: {e}")
    
    def emergency_stop(self):
        """
        Trigger emergency stop - immediately center joystick.
        """
        logger.critical("Emergency stop triggered")
        self.safety_manager.emergency_stop()
        self._desired_x = 0
        self._desired_y = 0
        self._current_x = 0
        self._current_y = 0
        
        # Send centering frame immediately
        self.can_interface.send_joystick_frame(0, 0)
    
    def _control_loop(self):
        """Internal: Main control loop that sends CAN frames."""
        logger.info("Control loop started")
        last_send_time = time.time()
        
        try:
            while not self._stop_sending:
                current_time = time.time()
                time_since_send = current_time - last_send_time
                
                # Send frame at configured interval
                if time_since_send >= self.send_interval_ms:
                    # Check for inactivity timeout
                    if self.safety_manager.check_inactivity():
                        self._desired_x = 0
                        self._desired_y = 0
                    
                    # Update speed with ramping
                    actual_speed = self.safety_manager.set_target_speed(
                        self._current_speed
                    )
                    
                    # Apply speed scaling to joystick position
                    # At 50% speed, send full position; at lower speeds, reduce position
                    if actual_speed > 0:
                        speed_factor = actual_speed / 100.0
                    else:
                        speed_factor = 0
                    
                    scaled_x = int(self._desired_x * speed_factor)
                    scaled_y = int(self._desired_y * speed_factor)
                    
                    # Send CAN frame
                    if self.can_interface.send_joystick_frame(scaled_x, scaled_y):
                        self._current_x = scaled_x
                        self._current_y = scaled_y
                        
                        logger.debug(
                            f"Frame sent: X={scaled_x} Y={scaled_y} "
                            f"(speed={actual_speed}%)"
                        )
                    
                    last_send_time = current_time
                
                # Sleep briefly to avoid busy waiting
                time.sleep(0.001)
        
        except Exception as e:
            logger.error(f"Control loop error: {e}")
        finally:
            logger.info("Control loop stopped")
    
    def get_telemetry(self) -> dict:
        """
        Get current telemetry information.
        
        Returns:
            Dictionary with current state
        """
        return {
            "desired_position": {
                "x": self._desired_x,
                "y": self._desired_y,
            },
            "actual_position": {
                "x": self._current_x,
                "y": self._current_y,
            },
            "speed_percent": self.safety_manager.get_current_speed(),
            "is_moving": (self._current_x != 0 or self._current_y != 0),
            "can_connected": self.can_interface.is_connected,
        }
