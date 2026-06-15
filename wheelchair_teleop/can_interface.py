"""
CAN Bus Interface Module
Handles low-level CAN frame sending and monitoring via SocketCAN
"""

import subprocess
import struct
import time
import threading
import logging
from typing import Optional, Callable
from queue import Queue

logger = logging.getLogger(__name__)


class CANInterface:
    """
    Wrapper around SocketCAN for RNET wheelchair communication.
    
    RNET uses CAN 2.0B @ 125 Kbps with:
    - Standard frames (11-bit): Control, mode, profile
    - Extended frames (29-bit): Data, serial, config
    
    Joystick control frame format:
    02000X00#XxYy
      X = device slot (1 for JSM)
      Xx = X-axis position (signed int8)
      Yy = Y-axis position (signed int8)
    """
    
    # Frame IDs (from RNET specification)
    JOYSTICK_FRAME_ID = 0x02000100  # Extended frame for JSM (slot 1)
    SPEED_FRAME_ID = 0x0A040100      # Speed control
    HORN_START = 0x0C040100           # Horn start
    HORN_STOP = 0x0C040101            # Horn stop
    
    # Position value mappings
    CENTER_POS = 0x00
    POS_100 = 0x64                    # +100 (full positive)
    NEG_100 = 0x9C                    # -100 (full negative)
    
    def __init__(self, can_interface: str = "can0", device_slot: int = 1):
        """
        Initialize CAN interface.
        
        Args:
            can_interface: CAN interface name (default: can0)
            device_slot: RNET device slot for joystick (usually 1 for JSM)
        """
        self.can_interface = can_interface
        self.device_slot = device_slot
        self.is_connected = False
        self._monitor_thread = None
        self._stop_monitoring = False
        self.on_frame_received: Optional[Callable] = None
        self._rx_queue = Queue()
        self._last_send_time = 0
        
    def connect(self) -> bool:
        """
        Verify CAN interface is available and up.
        
        Returns:
            True if interface is ready, False otherwise
        """
        try:
            # Check if interface exists and is up
            result = subprocess.run(
                ["ip", "link", "show", self.can_interface],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode != 0:
                logger.error(f"CAN interface {self.can_interface} not found")
                return False
            
            if "UP" not in result.stdout:
                logger.warning(f"CAN interface {self.can_interface} is not UP")
                return False
            
            self.is_connected = True
            logger.info(f"Connected to CAN interface: {self.can_interface}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to CAN interface: {e}")
            return False
    
    def send_joystick_frame(self, x_pos: int, y_pos: int) -> bool:
        """
        Send joystick position frame to wheelchair.
        
        The FollowJSM method: send frame immediately after JSM frame is detected.
        This frame tells the wheelchair controller the desired joystick position.
        
        Args:
            x_pos: X-axis position (-100 to +100)
            y_pos: Y-axis position (-100 to +100)
        
        Returns:
            True if frame sent successfully
        """
        if not self.is_connected:
            logger.error("Not connected to CAN interface")
            return False
        
        # Clamp values to valid range
        x_pos = max(-100, min(100, x_pos))
        y_pos = max(-100, min(100, y_pos))
        
        # Convert signed int8 to unsigned byte representation
        # Python: int -> two's complement -> hex
        x_byte = x_pos & 0xFF
        y_byte = y_pos & 0xFF
        
        # Build frame: 02000100#XxYy
        frame_id = 0x02000100
        frame_data = f"{x_byte:02X}{y_byte:02X}"
        frame_str = f"{frame_id:X}#{frame_data}"
        
        try:
            subprocess.run(
                ["cansend", self.can_interface, frame_str],
                capture_output=True,
                timeout=1
            )
            logger.debug(f"Sent joystick frame: {frame_str} (X={x_pos}, Y={y_pos})")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send CAN frame: {e}")
            return False
    
    def set_speed(self, speed_percent: int) -> bool:
        """
        Set wheelchair maximum speed (0-100%).
        
        Frame format: 0A040100#Pp
        where Pp = speed as hex (0x00-0x64)
        
        Args:
            speed_percent: Speed percentage (0-100)
        
        Returns:
            True if frame sent successfully
        """
        if not self.is_connected:
            return False
        
        speed_percent = max(0, min(100, speed_percent))
        speed_byte = int(speed_percent * 0x64 / 100)
        
        frame_str = f"A040100#{speed_byte:02X}"
        
        try:
            subprocess.run(
                ["cansend", self.can_interface, frame_str],
                capture_output=True,
                timeout=1
            )
            logger.debug(f"Sent speed frame: {frame_str} ({speed_percent}%)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set speed: {e}")
            return False
    
    def sound_horn(self, duration_ms: int = 200) -> bool:
        """
        Sound the wheelchair horn.
        
        Args:
            duration_ms: Horn duration in milliseconds
        
        Returns:
            True if horn command sent successfully
        """
        if not self.is_connected:
            return False
        
        try:
            # Send horn start
            subprocess.run(
                ["cansend", self.can_interface, "C040100#"],
                capture_output=True,
                timeout=1
            )
            logger.debug("Horn: START")
            
            # Wait for specified duration
            time.sleep(duration_ms / 1000.0)
            
            # Send horn stop
            subprocess.run(
                ["cansend", self.can_interface, "C040101#"],
                capture_output=True,
                timeout=1
            )
            logger.debug("Horn: STOP")
            return True
            
        except Exception as e:
            logger.error(f"Failed to sound horn: {e}")
            return False
    
    def start_monitoring(self, callback: Optional[Callable] = None):
        """
        Start monitoring CAN bus for incoming frames.
        Useful for detecting JSM frames (FollowJSM method).
        
        Args:
            callback: Optional callback function for received frames
        """
        if callback:
            self.on_frame_received = callback
        
        if self._monitor_thread and self._monitor_thread.is_alive():
            logger.warning("Monitoring already active")
            return
        
        self._stop_monitoring = False
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            daemon=True
        )
        self._monitor_thread.start()
        logger.info("CAN monitoring started")
    
    def stop_monitoring(self):
        """Stop the CAN bus monitoring thread."""
        self._stop_monitoring = True
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        logger.info("CAN monitoring stopped")
    
    def _monitor_loop(self):
        """Internal: Monitor CAN bus for incoming frames."""
        try:
            process = subprocess.Popen(
                ["candump", self.can_interface, "-t", "0", "-l"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            
            while not self._stop_monitoring:
                try:
                    line = process.stdout.readline()
                    if not line:
                        break
                    
                    # Parse candump output format:
                    # (timestamp) interface_name frame_id#data
                    # Example: (123.456) can0 02000100#0000
                    
                    if "#" in line:
                        frame_data = {
                            "raw": line.strip(),
                            "timestamp": time.time()
                        }
                        
                        self._rx_queue.put(frame_data)
                        
                        if self.on_frame_received:
                            self.on_frame_received(frame_data)
                
                except Exception as e:
                    if not self._stop_monitoring:
                        logger.debug(f"Monitor loop error: {e}")
            
            process.terminate()
            
        except Exception as e:
            logger.error(f"CAN monitoring failed: {e}")
    
    def get_received_frames(self, max_frames: int = 10) -> list:
        """
        Get buffered received CAN frames.
        
        Returns:
            List of frame dictionaries
        """
        frames = []
        while len(frames) < max_frames:
            try:
                frame = self._rx_queue.get_nowait()
                frames.append(frame)
            except:
                break
        return frames
    
    def disconnect(self):
        """Disconnect and cleanup."""
        self.stop_monitoring()
        self.is_connected = False
        logger.info("CAN interface disconnected")
