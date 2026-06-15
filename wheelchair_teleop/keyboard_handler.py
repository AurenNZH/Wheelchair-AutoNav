"""
Keyboard Input Handler Module
Non-blocking keyboard input handling for teleoperation over SSH
"""

import sys
import threading
import logging
from typing import Callable, Optional, Set

logger = logging.getLogger(__name__)


class KeyboardHandler:
    """
    Handle non-blocking keyboard input for wheelchair teleoperation.
    
    Key Bindings:
    - WASD: Drive (W=forward, A=left, S=backward, D=right)
    - Arrow Keys: Alternative drive control
    - 1-5: Speed presets (20%, 40%, 60%, 80%, 100%)
    - H: Sound horn
    - Q: Quit
    - Space: Emergency stop (center joystick)
    """
    
    # Speed presets (percentage)
    SPEED_PRESETS = {
        '1': 20,
        '2': 40,
        '3': 60,
        '4': 80,
        '5': 100,
    }
    
    def __init__(self, on_joystick_update: Optional[Callable] = None,
                 on_speed_change: Optional[Callable] = None,
                 on_horn: Optional[Callable] = None,
                 on_stop: Optional[Callable] = None):
        """
        Initialize keyboard handler.
        
        Args:
            on_joystick_update: Callback when joystick position changes
            on_speed_change: Callback when speed changes
            on_horn: Callback when horn is activated
            on_stop: Callback when stop is requested
        """
        self.on_joystick_update = on_joystick_update
        self.on_speed_change = on_speed_change
        self.on_horn = on_horn
        self.on_stop = on_stop
        
        self._keys_pressed: Set[str] = set()
        self._monitor_thread = None
        self._stop_monitoring = False
        self._last_x = 0
        self._last_y = 0
        self._current_speed = 50
        
        # Try to import tty/termios for Unix-like systems (Linux, Mac)
        try:
            import tty
            import termios
            self._has_tty = True
        except ImportError:
            self._has_tty = False
            logger.warning("TTY module not available (Windows?). Keyboard input may be limited.")
    
    def start(self):
        """Start keyboard monitoring."""
        if self._monitor_thread and self._monitor_thread.is_alive():
            logger.warning("Keyboard monitoring already active")
            return
        
        self._stop_monitoring = False
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True
        )
        self._monitor_thread.start()
        logger.info("Keyboard monitoring started")
    
    def stop(self):
        """Stop keyboard monitoring."""
        self._stop_monitoring = True
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        self._keys_pressed.clear()
        logger.info("Keyboard monitoring stopped")
    
    def _monitor_loop(self):
        """Internal: Monitor keyboard input."""
        if self._has_tty:
            self._monitor_unix()
        else:
            self._monitor_blocking()
    
    def _monitor_unix(self):
        """Monitor keyboard on Unix-like systems using raw input."""
        import tty
        import termios
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            # Set terminal to raw mode
            tty.setraw(fd)
            logger.info("Terminal set to raw mode for keyboard input")
            
            while not self._stop_monitoring:
                try:
                    # Read single character with timeout
                    import select
                    ready = select.select([sys.stdin], [], [], 0.1)
                    
                    if ready[0]:
                        ch = sys.stdin.read(1)
                        if ch:
                            self._handle_key(ch)
                
                except Exception as e:
                    if not self._stop_monitoring:
                        logger.debug(f"Keyboard read error: {e}")
        
        finally:
            # Restore terminal settings
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                logger.info("Terminal settings restored")
            except:
                pass
    
    def _monitor_blocking(self):
        """Monitor keyboard with blocking input (Windows/fallback)."""
        logger.info("Using blocking keyboard input mode")
        
        while not self._stop_monitoring:
            try:
                ch = sys.stdin.read(1)
                if ch:
                    self._handle_key(ch)
            except Exception as e:
                if not self._stop_monitoring:
                    logger.debug(f"Keyboard error: {e}")
    
    def _handle_key(self, key: str):
        """
        Process keyboard input.
        
        Args:
            key: Single character from keyboard
        """
        key_lower = key.lower()
        
        # Check for quit command
        if key_lower == 'q':
            logger.info("Quit command received")
            if self.on_stop:
                self.on_stop()
            self._stop_monitoring = True
            return
        
        # Check for emergency stop (Space)
        if key == ' ':
            logger.info("Emergency stop - centering joystick")
            self._keys_pressed.clear()
            if self.on_joystick_update:
                self.on_joystick_update(0, 0)
            return
        
        # Drive controls: WASD or Arrow keys
        if key_lower == 'w':
            self._keys_pressed.add('forward')
        elif key_lower == 's':
            self._keys_pressed.add('backward')
        elif key_lower == 'a':
            self._keys_pressed.add('left')
        elif key_lower == 'd':
            self._keys_pressed.add('right')
        
        # Speed presets
        elif key in self.SPEED_PRESETS:
            speed = self.SPEED_PRESETS[key]
            self._current_speed = speed
            logger.info(f"Speed set to {speed}%")
            if self.on_speed_change:
                self.on_speed_change(speed)
        
        # Horn
        elif key_lower == 'h':
            logger.info("Horn activated")
            if self.on_horn:
                self.on_horn()
        
        # Update joystick position based on pressed keys
        self._update_joystick_position()
    
    def _update_joystick_position(self):
        """Update joystick position based on currently pressed keys."""
        x_pos = 0
        y_pos = 0
        
        # Calculate X-axis (left-right)
        if 'left' in self._keys_pressed:
            x_pos -= 100
        if 'right' in self._keys_pressed:
            x_pos += 100
        
        # Handle both pressed simultaneously -> cancel out
        if x_pos < 0 and x_pos > 0:
            x_pos = 0
        
        # Calculate Y-axis (forward-backward)
        if 'forward' in self._keys_pressed:
            y_pos += 100
        if 'backward' in self._keys_pressed:
            y_pos -= 100
        
        # Handle both pressed simultaneously -> cancel out
        if y_pos > 0 and y_pos < 0:
            y_pos = 0
        
        # Only update if changed
        if x_pos != self._last_x or y_pos != self._last_y:
            self._last_x = x_pos
            self._last_y = y_pos
            
            if self.on_joystick_update:
                self.on_joystick_update(x_pos, y_pos)
            
            if x_pos != 0 or y_pos != 0:
                logger.debug(f"Joystick: X={x_pos}, Y={y_pos}")
    
    def get_current_position(self) -> tuple:
        """
        Get current joystick position.
        
        Returns:
            Tuple of (x_pos, y_pos)
        """
        return self._last_x, self._last_y
    
    def get_current_speed(self) -> int:
        """Get current speed setting."""
        return self._current_speed
