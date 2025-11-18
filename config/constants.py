"""
System Constants for Satellite Control System

Read-only system-wide constants for UI, network, data management, and conversions.
These values remain unchanged during execution.

Constant categories:
- UI/Visualization: Window sizes, subplot configuration, overlay dimensions
- Network: Server URLs, ports, timeout values
- Data Management: File paths, directory structure, logging locations
- Unit Conversions: Degrees/radians, meters/millimeters
- Platform-Specific: OS-dependent paths and settings

Key features:
- Centralized constant definitions
- Platform-specific path handling (macOS/Linux/Windows)
- Consistent UI dimensions across modules
- Standardized network communication parameters
"""

import os
import platform

import numpy as np


class Constants:
    """
    System-wide constants for UI, network, data management, and conversions.

    This class contains read-only constants that don't change during execution.
    """

    # ========================================================================
    # UI/VISUALIZATION CONSTANTS
    # ========================================================================

    WINDOW_WIDTH = 700  # Main window width (pixels)
    WINDOW_HEIGHT = 600  # Main window height (pixels)
    SUBPLOT_CONFIG = 111  # Matplotlib subplot configuration
    OVERLAY_HEIGHT = 32  # Overlay widget height (pixels)

    # Arrow visualization
    ARROW_X_OFFSET = 0.08  # Arrow X position offset (from center)
    ARROW_Y_OFFSET = 0.08  # Arrow Y position offset (from center)
    ARROW_WIDTH = 0.05  # Arrow width for heading display

    SLEEP_TARGET_DT = 0.9  # Sleep multiplier for target dt

    # Headless mode (no GUI windows)
    HEADLESS_MODE = True

    # ========================================================================
    # CONVERSION CONSTANTS
    # ========================================================================

    DEG_PER_CIRCLE = 360  # Degrees in full circle
    RAD_TO_DEG = 180.0 / np.pi  # Radians to degrees conversion
    DEG_TO_RAD = np.pi / 180.0  # Degrees to radians conversion

    # ========================================================================
    # NETWORK/COMMUNICATION CONSTANTS
    # ========================================================================

    SATELLITE_UDP_PORT = 8080  # Default UDP port for satellite communication
    HTTP_SUCCESS_CODE = 200  # HTTP success status code

    # Serial communication
    SERIAL_PORT = "COM20"  # Default port (Windows), set to None for auto-detect
    SERIAL_BAUDRATE = 115200
    SERIAL_TIMEOUT = 1.0
    SERIAL_CONNECTION_TIMEOUT = 5.0  # Timeout for serial connection (seconds)
    SERIAL_RETRY_DELAY_BASE = 0.1  # Base delay for exponential backoff (seconds)

    # Motion capture server
    MOTIVE_SERVER_URL = "http://127.0.0.1:5000/data"
    MOTIVE_TIMEOUT = 0.5  # Timeout for motion capture requests (seconds)
    TELEMETRY_TIMEOUT = 0.1  # Quick timeout for responsive telemetry (seconds)

    # Real-time operation
    HARDWARE_SAFETY_MARGIN = 0.02  # 20ms safety margin for real-time operation

    # Mission precision requirement
    PRECISION_TARGET_THRESHOLD = 0.050  # Final position error threshold for precision (meters)

    # ========================================================================
    # DATA MANAGEMENT
    # ========================================================================

    # Base data directory
    DATA_DIR = "Data"

    # Specific data subdirectories
    LINEARIZED_DATA_DIR = os.path.join(DATA_DIR, "Linearized")
    REAL_TEST_DATA_DIR = os.path.join(DATA_DIR, "Real_Test")
    THRUSTER_DATA_DIR = os.path.join(DATA_DIR, "Thruster_Data")

    # File naming conventions
    CSV_TIMESTAMP_FORMAT = "%d-%m-%Y_%H-%M-%S"

    # ========================================================================
    # FFMPEG PATHS (for video recording)
    # ========================================================================

    FFMPEG_PATH_WINDOWS = r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"  # Update to your FFmpeg installation path
    FFMPEG_PATH_MACOS = "/opt/homebrew/bin/ffmpeg"
    FFMPEG_PATH_LINUX = "ffmpeg"  # Usually in PATH

    # Auto-detect platform and set FFmpeg path
    _platform = platform.system()
    if _platform == "Windows":
        FFMPEG_PATH = FFMPEG_PATH_WINDOWS
    elif _platform == "Darwin":  # macOS
        FFMPEG_PATH = FFMPEG_PATH_MACOS
    else:  # Linux and others
        FFMPEG_PATH = FFMPEG_PATH_LINUX

    # ========================================================================
    # MISSION DEFAULTS
    # ========================================================================

    DEFAULT_START_POS = (-1.0, -1.0)
    DEFAULT_TARGET_POS = (0.0, 0.0)
    DEFAULT_START_ANGLE = np.deg2rad(90)
    DEFAULT_TARGET_ANGLE = np.deg2rad(90)

    # ========================================================================
    # HELPER METHODS
    # ========================================================================

    @classmethod
    def get_hardware_params(cls) -> dict:
        """
        Get hardware interface parameters.

        Returns:
            dict: Hardware parameters for real satellite operation
        """
        return {
            "serial_port": cls.SERIAL_PORT,
            "serial_baudrate": cls.SERIAL_BAUDRATE,
            "serial_timeout": cls.SERIAL_TIMEOUT,
            "data_dir": cls.REAL_TEST_DATA_DIR,
            "safety_margin": cls.HARDWARE_SAFETY_MARGIN,
        }

    @classmethod
    def get_simulation_params(cls) -> dict:
        """
        Get simulation-specific parameters.

        Returns:
            dict: Simulation parameters
        """
        return {
            "data_dir": cls.LINEARIZED_DATA_DIR,
            "timestamp_format": cls.CSV_TIMESTAMP_FORMAT,
        }

    @classmethod
    def print_constants(cls) -> None:
        """Print all system constants for debugging."""
        print("=" * 80)
        print("SYSTEM CONSTANTS")
        print("=" * 80)

        print("\nUI/VISUALIZATION:")
        print(f"   Window size:            {cls.WINDOW_WIDTH}x{cls.WINDOW_HEIGHT}")
        print(f"   Headless mode:          {cls.HEADLESS_MODE}")

        print("\nNETWORK:")
        print(f"   UDP port:               {cls.SATELLITE_UDP_PORT}")
        print(f"   Serial port:            {cls.SERIAL_PORT}")
        print(f"   Serial baudrate:        {cls.SERIAL_BAUDRATE}")
        print(f"   Motive server:          {cls.MOTIVE_SERVER_URL}")

        print("\nDATA MANAGEMENT:")
        print(f"   Data directory:         {cls.DATA_DIR}")
        print(f"   Simulation data:        {cls.LINEARIZED_DATA_DIR}")
        print(f"   Real test data:         {cls.REAL_TEST_DATA_DIR}")
        print(f"   Timestamp format:       {cls.CSV_TIMESTAMP_FORMAT}")

        print("\nFFMPEG:")
        print(f"   Platform:               {cls._platform}")
        print(f"   FFmpeg path:            {cls.FFMPEG_PATH}")

        print("=" * 80)
