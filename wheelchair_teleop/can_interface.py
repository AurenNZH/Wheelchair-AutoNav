"""
CAN Bus Interface Module
Handles low-level CAN frame sending and monitoring via SocketCAN
"""

import subprocess
import struct
import time
import threading
import logging
import select
import socket
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

    # SocketCAN constants
    CAN_EFF_FLAG = 0x80000000
    CAN_RTR_FLAG = 0x40000000
    CAN_ERR_FLAG = 0x20000000
    CAN_EFF_MASK = 0x1FFFFFFF
    CAN_SFF_MASK = 0x000007FF
    CAN_FRAME_FORMAT = "=IB3x8s"
    CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
    
    # Position value mappings
    CENTER_POS = 0x00
    POS_100 = 0x64                    # +100 (full positive)
    NEG_100 = 0x9C                    # -100 (full negative)
    
    def __init__(self, can_interface: str = "can0", device_slot: int = 1,
                 gateway_interface: Optional[str] = None,
                 gateway_enabled: bool = False):
        """
        Initialize CAN interface.
        
        Args:
            can_interface: CAN interface name used for injected frames (default: can0)
            device_slot: RNET device slot for joystick (usually 1 for JSM)
            gateway_interface: Optional second CAN interface connected to the physical JSM side
            gateway_enabled: Forward frames between can_interface and gateway_interface
        """
        self.can_interface = can_interface
        self.device_slot = device_slot
        self.gateway_interface = gateway_interface
        self.gateway_enabled = gateway_enabled and bool(gateway_interface)
        self.is_connected = False
        self.is_gateway_running = False
        self._monitor_thread = None
        self._gateway_thread = None
        self._stop_monitoring = False
        self._stop_gateway = False
        self.on_frame_received: Optional[Callable] = None
        self._rx_queue = Queue()
        self._last_send_time = 0
        self._tx_socket = None
        self._teleop_active = False
        self._gateway_stats = {
            "forwarded_to_controller": 0,
            "forwarded_to_joystick": 0,
            "suppressed_joystick": 0,
            "errors": 0,
        }
        
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

            if self.gateway_enabled:
                gateway_result = subprocess.run(
                    ["ip", "link", "show", self.gateway_interface],
                    capture_output=True,
                    text=True,
                    timeout=2
                )

                if gateway_result.returncode != 0:
                    logger.warning(
                        f"Gateway interface {self.gateway_interface} not found; "
                        "CAN passthrough disabled"
                    )
                    self.gateway_enabled = False
                elif "UP" not in gateway_result.stdout:
                    logger.warning(
                        f"Gateway interface {self.gateway_interface} is not UP; "
                        "CAN passthrough disabled"
                    )
                    self.gateway_enabled = False
            
            self._tx_socket = self._open_can_socket(self.can_interface, loopback=False)
            self.is_connected = True
            logger.info(f"Connected to CAN interface: {self.can_interface}")

            if self.gateway_enabled:
                self.start_gateway()

            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to CAN interface: {e}")
            return False

    def _open_can_socket(self, interface_name: str, loopback: bool = True):
        """Open a raw SocketCAN socket bound to an interface."""
        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

        # Disable local loopback for injected frames so the bridge does not echo
        # teleop commands back to the physical joystick side.
        if not loopback:
            sol_can_raw = getattr(socket, "SOL_CAN_RAW", 101)
            can_raw_loopback = getattr(socket, "CAN_RAW_LOOPBACK", 3)
            can_socket.setsockopt(sol_can_raw, can_raw_loopback, struct.pack("I", 0))

        can_socket.bind((interface_name,))
        return can_socket

    def _build_can_frame(self, frame_id: int, data: bytes,
                         extended: bool = True) -> bytes:
        """Pack a SocketCAN can_frame."""
        can_id = frame_id
        if extended:
            can_id |= self.CAN_EFF_FLAG

        data = data[:8]
        return struct.pack(
            self.CAN_FRAME_FORMAT,
            can_id,
            len(data),
            data.ljust(8, b"\x00")
        )

    def _unpack_can_frame(self, frame: bytes) -> tuple:
        """Unpack a SocketCAN can_frame into (can_id_with_flags, data)."""
        can_id, can_dlc, data = struct.unpack(self.CAN_FRAME_FORMAT, frame[:self.CAN_FRAME_SIZE])
        return can_id, data[:can_dlc]

    def _send_raw_frame(self, interface_name: str, frame_id: int, data: bytes,
                        extended: bool = True) -> bool:
        """Send a CAN frame using SocketCAN, falling back to cansend if needed."""
        try:
            frame = self._build_can_frame(frame_id, data, extended=extended)

            if interface_name == self.can_interface and self._tx_socket:
                self._tx_socket.send(frame)
            else:
                with self._open_can_socket(interface_name, loopback=False) as can_socket:
                    can_socket.send(frame)

            return True

        except Exception as socket_error:
            logger.debug(f"SocketCAN send failed, trying cansend: {socket_error}")

            if extended:
                frame_str = f"{frame_id:08X}#{data.hex().upper()}"
            else:
                frame_str = f"{frame_id:X}#{data.hex().upper()}"

            try:
                result = subprocess.run(
                    ["cansend", interface_name, frame_str],
                    capture_output=True,
                    timeout=1
                )
                if result.returncode != 0:
                    logger.error(f"cansend failed for {frame_str}: {result.stderr}")
                    return False
                return True
            except Exception as cansend_error:
                logger.error(f"Failed to send CAN frame: {cansend_error}")
                return False

    def set_teleop_active(self, active: bool):
        """
        Mark whether keyboard teleoperation is currently commanding movement.

        While active, joystick-position frames from the physical JSM side are not
        forwarded to the controller side. Other RNET traffic continues to pass.
        """
        if self._teleop_active != active:
            logger.debug(f"Teleop active state changed: {active}")
        self._teleop_active = active
    
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
        
        frame_data = bytes([x_byte, y_byte])
        
        if self._send_raw_frame(self.can_interface, self.JOYSTICK_FRAME_ID, frame_data):
            logger.debug(
                f"Sent joystick frame: {self.JOYSTICK_FRAME_ID:08X}#{frame_data.hex().upper()} "
                f"(X={x_pos}, Y={y_pos})"
            )
            return True

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

        if self._send_raw_frame(self.can_interface, self.SPEED_FRAME_ID, bytes([speed_byte])):
            logger.debug(f"Sent speed frame: {self.SPEED_FRAME_ID:08X}#{speed_byte:02X} ({speed_percent}%)")
            return True

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
            self._send_raw_frame(self.can_interface, self.HORN_START, b"")
            logger.debug("Horn: START")
            
            # Wait for specified duration
            time.sleep(duration_ms / 1000.0)
            
            # Send horn stop
            self._send_raw_frame(self.can_interface, self.HORN_STOP, b"")
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

    def start_gateway(self):
        """Start bidirectional forwarding between the two CAN interfaces."""
        if not self.gateway_enabled:
            logger.info("CAN gateway not enabled")
            return

        if self._gateway_thread and self._gateway_thread.is_alive():
            logger.warning("CAN gateway already active")
            return

        self._stop_gateway = False
        self._gateway_thread = threading.Thread(
            target=self._gateway_loop,
            daemon=True
        )
        self._gateway_thread.start()
        self.is_gateway_running = True
        logger.info(
            f"CAN gateway started: {self.gateway_interface} <-> {self.can_interface}"
        )

    def stop_gateway(self):
        """Stop bidirectional CAN forwarding."""
        self._stop_gateway = True
        if self._gateway_thread:
            self._gateway_thread.join(timeout=2)
        self.is_gateway_running = False
        logger.info("CAN gateway stopped")

    def _is_joystick_position_frame(self, can_id: int) -> bool:
        """Return True for the RNET JSM joystick position frame."""
        if can_id & self.CAN_ERR_FLAG:
            return False

        if can_id & self.CAN_EFF_FLAG:
            return (can_id & self.CAN_EFF_MASK) == self.JOYSTICK_FRAME_ID

        return (can_id & self.CAN_SFF_MASK) == self.JOYSTICK_FRAME_ID

    def _gateway_loop(self):
        """Forward frames in both directions, replacing JSM movement when active."""
        controller_socket = None
        joystick_socket = None

        try:
            controller_socket = self._open_can_socket(self.can_interface)
            joystick_socket = self._open_can_socket(self.gateway_interface)
            sockets = {
                controller_socket: (self.can_interface, joystick_socket, "forwarded_to_joystick"),
                joystick_socket: (self.gateway_interface, controller_socket, "forwarded_to_controller"),
            }

            while not self._stop_gateway:
                readable, _, _ = select.select(list(sockets.keys()), [], [], 0.1)

                for source_socket in readable:
                    source_name, dest_socket, stat_key = sockets[source_socket]

                    try:
                        frame = source_socket.recv(self.CAN_FRAME_SIZE)
                        if not frame:
                            continue

                        can_id, _ = self._unpack_can_frame(frame)

                        # During keyboard teleop, keep all RNET housekeeping traffic
                        # flowing but prevent centered physical-JSM frames from racing
                        # against the injected command on the controller side.
                        if (
                            source_name == self.gateway_interface
                            and self._teleop_active
                            and self._is_joystick_position_frame(can_id)
                        ):
                            self._gateway_stats["suppressed_joystick"] += 1
                            continue

                        dest_socket.send(frame)
                        self._gateway_stats[stat_key] += 1

                    except Exception as e:
                        self._gateway_stats["errors"] += 1
                        if not self._stop_gateway:
                            logger.debug(f"CAN gateway forwarding error: {e}")

        except Exception as e:
            self._gateway_stats["errors"] += 1
            logger.error(f"CAN gateway failed: {e}")
        finally:
            for can_socket in (controller_socket, joystick_socket):
                if can_socket:
                    try:
                        can_socket.close()
                    except Exception:
                        pass
            self.is_gateway_running = False
    
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

    def get_gateway_stats(self) -> dict:
        """Return current CAN gateway counters."""
        return dict(self._gateway_stats)
    
    def disconnect(self):
        """Disconnect and cleanup."""
        self.stop_gateway()
        self.stop_monitoring()
        if self._tx_socket:
            try:
                self._tx_socket.close()
            except Exception:
                pass
            self._tx_socket = None
        self.is_connected = False
        logger.info("CAN interface disconnected")
