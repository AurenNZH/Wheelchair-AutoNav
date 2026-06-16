"""
Safety Management Module
Implements safety limits and emergency stop logic
"""

import logging
import time
from typing import Optional

logger = logging.getLogger(__name__)


class SafetyManager:
    """
    Implement safety constraints for wheelchair teleoperation.
    
    Safety Features:
    - Acceleration ramps (no sudden max speed)
    - Maximum speed limits
    - Inactivity timeout
    - Frequency limiting for CAN messages
    """
    
    def __init__(self, 
                 max_speed: int = 100,
                 acceleration_rate: float = 50.0,
                 inactivity_timeout: float = 5.0,
                 min_frame_interval_ms: float = 50.0):
        """
        Initialize safety manager.
        
        Args:
            max_speed: Maximum allowed speed (0-100%)
            acceleration_rate: Speed increase per second (0-100)
            inactivity_timeout: Seconds before auto-stop (0 to disable)
            min_frame_interval_ms: Minimum milliseconds between CAN frames
        """
        self.max_speed = max(0, min(100, max_speed))
        self.acceleration_rate = acceleration_rate
        self.inactivity_timeout = inactivity_timeout
        self.min_frame_interval_ms = min_frame_interval_ms / 1000.0  # Convert to seconds
        
        self._current_speed = 0
        self._target_speed = 0
        self._last_input_time = time.time()
        self._last_frame_time = 0
        self._last_inactivity_warning_time = 0
        self._is_stopped = True
        
        logger.info(f"Safety Manager initialized: max_speed={self.max_speed}%, "
                   f"accel={acceleration_rate}%/s, timeout={inactivity_timeout}s")
    
    def set_target_speed(self, speed_percent: int) -> int:
        """
        Set target speed with acceleration ramping.
        
        Args:
            speed_percent: Target speed (0-100)
        
        Returns:
            Current actual speed after ramping
        """
        # Clamp to max speed
        self._target_speed = max(0, min(self.max_speed, speed_percent))
        
        # Ramp current speed toward target
        current_time = time.time()
        time_delta = current_time - self._last_frame_time if self._last_frame_time else 0.01
        
        if time_delta > 0:
            max_change = self.acceleration_rate * time_delta
            
            if self._current_speed < self._target_speed:
                self._current_speed = min(
                    self._current_speed + max_change,
                    self._target_speed
                )
            elif self._current_speed > self._target_speed:
                self._current_speed = max(
                    self._current_speed - max_change,
                    self._target_speed
                )
        
        self._last_frame_time = current_time
        return int(self._current_speed)
    
    def get_current_speed(self) -> int:
        """Get current ramped speed."""
        return int(self._current_speed)
    
    def record_input(self):
        """Record that user input was received (for inactivity timeout)."""
        self._last_input_time = time.time()
        self._last_inactivity_warning_time = 0
    
    def check_inactivity(self) -> bool:
        """
        Check if inactivity timeout exceeded.
        
        Returns:
            True if timeout exceeded and chair should stop
        """
        if self.inactivity_timeout <= 0:
            return False
        
        time_since_input = time.time() - self._last_input_time
        
        if time_since_input > self.inactivity_timeout:
            current_time = time.time()
            if current_time - self._last_inactivity_warning_time >= 1.0:
                logger.warning(f"Inactivity timeout exceeded ({time_since_input:.1f}s)")
                self._last_inactivity_warning_time = current_time
            return True
        
        return False
    
    def should_send_frame(self) -> bool:
        """
        Check if enough time has passed to send next CAN frame.
        Used to respect RNET protocol timing (10ms minimum).
        
        Returns:
            True if frame should be sent, False if too soon
        """
        time_since_last = time.time() - self._last_frame_time
        
        if time_since_last >= self.min_frame_interval_ms:
            self._last_frame_time = time.time()
            return True
        
        return False
    
    def emergency_stop(self):
        """Trigger emergency stop."""
        logger.critical("EMERGENCY STOP")
        self._current_speed = 0
        self._target_speed = 0
        self._is_stopped = True
    
    def is_stopped(self) -> bool:
        """Check if chair is at zero speed."""
        return self._current_speed == 0
    
    def validate_joystick_input(self, x: int, y: int) -> tuple:
        """
        Validate and potentially limit joystick input.
        
        Args:
            x: X-axis position
            y: Y-axis position
        
        Returns:
            Validated (x, y) tuple
        """
        # Clamp to valid range
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        
        return x, y
