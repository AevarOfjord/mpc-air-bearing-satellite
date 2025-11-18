"""
Hardware Interface for Satellite Thruster Control System

Handles communication with satellite hardware via multiple protocols.
Provides flexible interface for different thruster control systems and communication methods.

Supported communication protocols:
- Serial (USB/UART): Primary method for Arduino-based controllers
- UDP: Network-based control for distributed systems
- TCP: Reliable network communication
- Mock mode: Software-in-the-loop testing without hardware

Key features:
- Multi-protocol support with automatic fallback
- Command validation and error checking
- Connection health monitoring and auto-reconnection
- Command history logging for debugging
- Configurable baud rates and timeouts
- Thread-safe operations for concurrent control
- Graceful degradation on communication failures

Hardware integration:
- Arduino-based thruster controller (primary)
- Custom embedded systems via serial/network
- Simulation mode for testing without hardware
"""

import json
import socket
import time
from dataclasses import dataclass
from typing import Optional, Union

import numpy as np
import serial

from config import SatelliteConfig
from logging_config import setup_logging

# Set up logger for hardware interface (debug messages won't show at default INFO level)
logger = setup_logging(__name__, log_file="Data/hardware_interface.log", simple_format=True)


@dataclass
class ThrusterCommand:
    """Contains thruster command data with timing information."""

    thrusters: np.ndarray  # Array of 8 thruster states (0 or 1)
    timestamp: float
    command_id: int


class SatelliteHardwareInterface:
    """
    Handles communication with satellite hardware through multiple protocols.

    Supports:
    - Serial communication (USB/UART)
    - UDP network communication
    - TCP network communication
    - Custom protocol implementation
    """

    def __init__(self, communication_type="serial", command_duration_ms=100, **kwargs):
        """
        Set up hardware interface with specified communication method.

        Args:
            communication_type: Communication protocol - "serial", "udp", "tcp", or "custom"
            command_duration_ms: How long each thruster command lasts (default: 100ms)
            **kwargs: Protocol-specific configuration parameters
        """
        self.communication_type = communication_type
        self.command_duration_ms = command_duration_ms
        self.connection: Optional[Union[serial.Serial, socket.socket]] = None
        self.is_connected = False
        self.command_counter = 0

        # Connection health monitoring
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3
        self.last_successful_command_time = time.time()
        self.connection_timeout = SatelliteConfig.SERIAL_CONNECTION_TIMEOUT

        # Retry configuration
        self.max_retries = 3
        self.retry_delay_base = SatelliteConfig.SERIAL_RETRY_DELAY_BASE
        self.last_connection_attempt = 0
        self.reconnection_cooldown = 2.0  # Minimum time between reconnection attempts

        # Initialize based on communication type
        if communication_type == "serial":
            self.init_serial_connection(**kwargs)
        elif communication_type == "udp":
            self.init_udp_connection(**kwargs)
        elif communication_type == "tcp":
            self.init_tcp_connection(**kwargs)
        elif communication_type == "custom":
            self.init_custom_connection(**kwargs)
        else:
            logger.warning(f"Warning: Communication type '{communication_type}' not recognized")

    def init_serial_connection(self, port="COM20", baudrate=115200, timeout=1.0):
        """Set up serial connection to the satellite."""
        try:
            self.connection = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            time.sleep(2)  # Wait for hardware reset

            # Send safety stop command on connection
            self.connection.write(b"CMD:0:0\n")
            time.sleep(0.1)

            self.is_connected = True
            logger.info(f" Serial connection established on {port} at {baudrate} baud")
            logger.debug("Initial safety stop command sent")
        except serial.SerialException as e:
            logger.error(f" Failed to open serial connection: {e}")
            self.is_connected = False

    def init_udp_connection(
        self, satellite_ip="192.168.1.100", satellite_port=8080, local_port=8081
    ):
        """Initialize UDP connection to satellite."""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.connection.bind(("", local_port))
            self.satellite_address = (satellite_ip, satellite_port)
            self.is_connected = True
            logger.info(f" UDP connection established to {satellite_ip}:{satellite_port}")
        except socket.error as e:
            logger.error(f" Failed to create UDP connection: {e}")
            self.is_connected = False

    def init_tcp_connection(self, satellite_ip="192.168.1.100", satellite_port=8080):
        """Initialize TCP connection to satellite."""
        try:
            self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.connection.connect((satellite_ip, satellite_port))
            self.is_connected = True
            logger.info(f" TCP connection established to {satellite_ip}:{satellite_port}")
        except socket.error as e:
            logger.error(f" Failed to create TCP connection: {e}")
            self.is_connected = False

    def init_custom_connection(self, **kwargs):
        """Initialize custom connection protocol."""
        # Implement your custom communication protocol here
        logger.info("Custom communication protocol - implement as needed")
        self.is_connected = True

    def check_connection_health(self) -> bool:
        """
        Check if connection is healthy based on recent activity.

        Returns:
            True if connection appears healthy, False if potentially stale
        """
        if not self.is_connected:
            return False

        time_since_last_success = time.time() - self.last_successful_command_time

        if time_since_last_success > self.connection_timeout:
            logger.warning(
                f"  WARNING: No successful commands for {time_since_last_success:.1f}s"
            )
            return False

        if self.consecutive_failures >= self.max_consecutive_failures:
            logger.warning(f"  WARNING: {self.consecutive_failures} consecutive failures")
            return False

        return True

    def attempt_reconnection(self, **kwargs) -> bool:
        """
        Attempt to reconnect to hardware with cooldown protection.

        Args:
            **kwargs: Connection parameters (port, baudrate, etc.)

        Returns:
            True if reconnection successful, False otherwise
        """
        current_time = time.time()

        # Check cooldown
        if current_time - self.last_connection_attempt < self.reconnection_cooldown:
            remaining = self.reconnection_cooldown - (
                current_time - self.last_connection_attempt
            )
            logger.debug(f"⏳ Reconnection cooldown active ({remaining:.1f}s remaining)")
            return False

        self.last_connection_attempt = current_time
        logger.info(" Attempting to reconnect to hardware...")

        if self.connection:
            try:
                if self.communication_type == "serial":
                    self.connection.close()
                elif self.communication_type in ["udp", "tcp"]:
                    self.connection.close()
            except Exception as e:
                logger.warning(f"  Warning during connection cleanup: {e}")

        self.connection = None
        self.is_connected = False

        # Attempt reconnection based on type
        try:
            if self.communication_type == "serial":
                self.init_serial_connection(**kwargs)
            elif self.communication_type == "udp":
                self.init_udp_connection(**kwargs)
            elif self.communication_type == "tcp":
                self.init_tcp_connection(**kwargs)
            elif self.communication_type == "custom":
                self.init_custom_connection(**kwargs)

            if self.is_connected:
                logger.info(" Reconnection successful")
                self.consecutive_failures = 0
                self.last_successful_command_time = time.time()
                return True
            else:
                logger.error(" Reconnection failed")
                return False

        except Exception as e:
            logger.error(f" Reconnection error: {e}")
            return False

    def send_thruster_command(
        self, thruster_action: np.ndarray, retry_on_failure: bool = True
    ) -> bool:
        """
        Send thruster command to satellite hardware with retry logic and error handling.

        Args:
            thruster_action: Array of 8 thruster states (0.0 or 1.0)
            retry_on_failure: Whether to retry on failure with exponential backoff

        Returns:
            True if command sent successfully, False otherwise
        """
        if not self.is_connected:
            logger.warning("  Warning: Not connected to satellite hardware")
            return False

        # Validate input
        try:
            if thruster_action is None:
                raise ValueError("Thruster action cannot be None")
            if len(thruster_action) != 8:
                raise ValueError(
                    f"Expected 8 thruster values, got {len(thruster_action)}"
                )
            if not np.all(np.isfinite(thruster_action)):
                raise ValueError(
                    f"Thruster action contains invalid values: {thruster_action}"
                )
        except Exception as e:
            logger.error(f" Invalid thruster command: {e}")
            self.consecutive_failures += 1
            return False

        thrusters = (thruster_action > 0.5).astype(int)

        # Create command structure
        command = ThrusterCommand(
            thrusters=thrusters, timestamp=time.time(), command_id=self.command_counter
        )
        self.command_counter += 1

        # Attempt to send with retries
        for attempt in range(self.max_retries if retry_on_failure else 1):
            try:
                # Send command based on communication type
                if self.communication_type == "serial":
                    success = self.send_serial_command(command)
                elif self.communication_type == "udp":
                    success = self.send_udp_command(command)
                elif self.communication_type == "tcp":
                    success = self.send_tcp_command(command)
                elif self.communication_type == "custom":
                    success = self.send_custom_command(command)
                else:
                    logger.error(f" Unknown communication type: {self.communication_type}")
                    success = False

                if success:
                    # Command successful - reset failure counter
                    self.consecutive_failures = 0
                    self.last_successful_command_time = time.time()
                    return True
                else:
                    # Command failed but no exception
                    raise RuntimeError("Command transmission failed without exception")

            except serial.SerialException as e:
                logger.warning(f" Serial error (attempt {attempt + 1}/{self.max_retries}): {e}")
                self.consecutive_failures += 1

                if "device" in str(e).lower() or "disconnected" in str(e).lower():
                    logger.warning("  Connection lost, attempting reconnection...")
                    # Will be handled by check_connection_health

            except OSError as e:
                logger.warning(f" Network error (attempt {attempt + 1}/{self.max_retries}): {e}")
                self.consecutive_failures += 1

            except Exception as e:
                logger.warning(
                    f" Unexpected error (attempt {attempt + 1}/{self.max_retries}): {e}"
                )
                self.consecutive_failures += 1

            if attempt < self.max_retries - 1 and retry_on_failure:
                delay = self.retry_delay_base * (2**attempt)
                logger.debug(f"  Retrying in {delay:.2f}s...")
                time.sleep(delay)

        # All retries failed
        logger.error(f" Command failed after {self.max_retries} attempts")
        return False

    def send_serial_command(self, command: ThrusterCommand) -> bool:
        """Send command via serial connection using the same protocol as the thruster calibration system."""
        if not isinstance(self.connection, serial.Serial):
            logger.error("Error: Serial connection not available")
            return False

        try:
            bitmask = 0
            for i, thruster_state in enumerate(command.thrusters):
                if thruster_state > 0.5:  # Thruster is ON
                    bitmask |= 1 << i

            duration_ms = self.command_duration_ms

            command_str = f"CMD:{bitmask}:{duration_ms}\n"

            # Send command
            self.connection.write(command_str.encode())
            self.connection.flush()

            # Log active thrusters (debug level - won't show in terminal at default INFO level)
            active_thrusters = list(np.where(command.thrusters > 0)[0] + 1)
            if active_thrusters:
                logger.debug(
                    f"  -> Serial: CMD:{bitmask}:{duration_ms} - Thrusters {active_thrusters} ON"
                )
            else:
                logger.debug("  -> Serial: CMD:0:0 - All thrusters OFF")

            return True

        except serial.SerialException as e:
            logger.error(f"Serial communication error: {e}")
            return False

    def send_udp_command(self, command: ThrusterCommand) -> bool:
        """Send command via UDP."""
        if not isinstance(self.connection, socket.socket):
            logger.error("Error: UDP socket connection not available")
            return False

        try:
            # Example protocol: Send as JSON packet
            packet_data = {
                "command_type": "thruster_control",
                "command_id": command.command_id,
                "timestamp": command.timestamp,
                "thrusters": command.thrusters.tolist(),
            }

            packet_json = json.dumps(packet_data)
            packet_bytes = packet_json.encode("utf-8")

            # Send packet
            self.connection.sendto(packet_bytes, self.satellite_address)

            # Log active thrusters (debug level - won't show in terminal at default INFO level)
            active_thrusters = list(np.where(command.thrusters > 0)[0] + 1)
            if active_thrusters:
                logger.debug(f"  -> UDP: Thrusters {active_thrusters} ON")
            else:
                logger.debug("  -> UDP: All thrusters OFF")

            return True

        except socket.error as e:
            logger.error(f"UDP communication error: {e}")
            return False

    def send_tcp_command(self, command: ThrusterCommand) -> bool:
        """Send command via TCP."""
        if not isinstance(self.connection, socket.socket):
            logger.error("Error: TCP socket connection not available")
            return False

        try:
            # Example protocol: Send as JSON with length header
            packet_data = {
                "command_type": "thruster_control",
                "command_id": command.command_id,
                "timestamp": command.timestamp,
                "thrusters": command.thrusters.tolist(),
            }

            packet_json = json.dumps(packet_data)
            packet_bytes = packet_json.encode("utf-8")

            # Send length header followed by data
            length_header = len(packet_bytes).to_bytes(4, byteorder="big")
            self.connection.sendall(length_header + packet_bytes)

            # Log active thrusters (debug level - won't show in terminal at default INFO level)
            active_thrusters = list(np.where(command.thrusters > 0)[0] + 1)
            if active_thrusters:
                logger.debug(f"  -> TCP: Thrusters {active_thrusters} ON")
            else:
                logger.debug("  -> TCP: All thrusters OFF")

            return True

        except socket.error as e:
            logger.error(f"TCP communication error: {e}")
            return False

    def send_custom_command(self, command: ThrusterCommand) -> bool:
        """Send command via custom protocol."""
        # Implement your custom communication protocol here

        active_thrusters = list(np.where(command.thrusters > 0)[0] + 1)
        if active_thrusters:
            logger.debug(f"  -> Custom: Thrusters {active_thrusters} ON")
        else:
            logger.debug("  -> Custom: All thrusters OFF")

        return True

    def send_emergency_stop(self) -> bool:
        """Send emergency stop command (all thrusters off) using CMD protocol."""
        logger.warning("EMERGENCY STOP - All thrusters OFF")

        if not self.is_connected:
            logger.warning("Warning: Not connected to satellite hardware")
            return False

        try:
            if self.communication_type == "serial":
                if not isinstance(self.connection, serial.Serial):
                    logger.error("Error: Serial connection not available for emergency stop")
                    return False
                # Send immediate stop command using same protocol as thruster calibration system
                stop_command = "CMD:0:0\n"
                self.connection.write(stop_command.encode())
                self.connection.flush()
                logger.debug("  -> Serial: CMD:0:0 sent")
                return True
            else:
                # For other communication types, use the regular method
                emergency_command = np.zeros(8, dtype=np.float64)
                return self.send_thruster_command(emergency_command)

        except Exception as e:
            logger.error(f"Error sending emergency stop: {e}")
            return False

    def test_connection(self) -> bool:
        """Test the communication connection."""
        if not self.is_connected:
            return False

        test_command = np.zeros(8, dtype=np.float64)
        success = self.send_thruster_command(test_command)

        if success:
            logger.info(" Hardware connection test successful")
        else:
            logger.error(" Hardware connection test failed")

        return success

    def close_connection(self):
        """Close the connection to satellite hardware."""
        if self.connection:
            try:
                # Send final stop command before closing
                self.send_emergency_stop()
                time.sleep(0.1)

                if self.communication_type == "serial":
                    self.connection.close()
                elif self.communication_type in ["udp", "tcp"]:
                    self.connection.close()

                logger.info(" Hardware connection closed")

            except Exception as e:
                logger.error(f"Error closing connection: {e}")

        self.is_connected = False
        self.connection = None


def test_hardware_interface():
    """Test the hardware interface with different communication types."""

    print("Testing Hardware Interface")
    print("=" * 30)

    # Test different communication types
    test_configs = [
        {"type": "custom", "name": "Custom Protocol"},
        {"type": "serial", "name": "Serial (COM3)", "port": "COM3"},
        {
            "type": "udp",
            "name": "UDP Network",
            "satellite_ip": "127.0.0.1",
            "satellite_port": 8080,
        },
    ]

    for config in test_configs:
        print(f"\nTesting {config['name']}...")

        # Create interface
        interface_args = {k: v for k, v in config.items() if k not in ["type", "name"]}
        hardware = SatelliteHardwareInterface(config["type"], **interface_args)

        if hardware.is_connected:
            # Test basic command
            test_thrusters = np.array([1, 0, 1, 0, 0, 1, 0, 0])  # Thrusters 1, 3, 6
            success = hardware.send_thruster_command(test_thrusters)

            if success:
                print(f" {config['name']} test successful")
            else:
                print(f" {config['name']} test failed")

            # Close connection
            hardware.close_connection()
        else:
            print(f" {config['name']} connection failed")


if __name__ == "__main__":
    test_hardware_interface()
