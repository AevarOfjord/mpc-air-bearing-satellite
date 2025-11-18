"""
Camera Configuration for Satellite Control System

Camera streaming and recording parameters for multi-camera setups.
Manages HTTP/RTSP camera connections with robust retry policies.

Configuration sections:
- Camera stream URLs and display order
- Video recording settings (codec, resolution, frame rate)
- Connection retry policies with exponential backoff
- Error handling and reconnection timeouts
- Performance monitoring intervals

Key features:
- Multi-camera support with configurable URLs
- Optional video recording to disk
- Automatic reconnection with exponential backoff
- Configurable frame sizes and frame rates
- Statistics tracking for connection health
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class CameraConfig:
    """
    HTTP camera feed and recording parameters.

    Attributes:
        streams_enabled: Enable camera streaming
        recording_enabled: Enable video recording
        recording_codec: Video codec for recording (e.g., 'mp4v', 'MJPG')
        recording_extension: File extension for recordings (e.g., '.mp4', '.avi')
        frame_size: Video frame size (width, height) in pixels
        fps: Target frames per second
        initial_delay: Initial delay before first connection attempt in seconds
        retry_backoff: Initial backoff delay for retries in seconds
        max_backoff: Maximum backoff delay in seconds
        backoff_multiplier: Multiplier for exponential backoff
        max_consecutive_failures: Max failures before giving up
        reconnect_timeout: Timeout for reconnection attempts in seconds
        frame_read_delay: Delay between frame reads in seconds
        error_delay: Delay after errors in seconds
        stats_interval: Interval for printing statistics in seconds
        urls: Dict mapping camera name to HTTP stream URL
        order: List of camera names in display order
    """

    streams_enabled: bool
    recording_enabled: bool
    recording_codec: str
    recording_extension: str
    frame_size: Tuple[int, int]
    fps: int
    initial_delay: float
    retry_backoff: float
    max_backoff: float
    backoff_multiplier: float
    max_consecutive_failures: int
    reconnect_timeout: float
    frame_read_delay: float
    error_delay: float
    stats_interval: int
    urls: Dict[str, str]
    order: List[str]


# DEFAULT CAMERA PARAMETERS
# ============================================================================

CAMERA_STREAMS_ENABLED = True
CAMERA_RECORDING_ENABLED = True
CAMERA_RECORDING_CODEC = "mp4v"
CAMERA_RECORDING_EXTENSION = ".mp4"
CAMERA_FRAME_SIZE = (640, 480)
CAMERA_FPS = 15

# Connection retry policy
CAMERA_INITIAL_DELAY = 0.3
CAMERA_RETRY_BACKOFF = 0.5
CAMERA_MAX_BACKOFF = 5.0
CAMERA_BACKOFF_MULTIPLIER = 1.5
CAMERA_MAX_CONSECUTIVE_FAILURES = 10
CAMERA_RECONNECT_TIMEOUT = 10.0

# Frame reading timing
CAMERA_FRAME_READ_DELAY = 0.05
CAMERA_ERROR_DELAY = 0.2
CAMERA_STATS_INTERVAL = 300  # seconds

# Camera URLs (IP addresses and ports)
CAMERA_URLS = {
    "Front": "http://192.168.137.200:81/stream",
    "Right": "http://192.168.137.100:81/stream",
    "Back": "http://192.168.137.86:81/stream",
    "Left": "http://192.168.137.80:81/stream",
}

CAMERA_ORDER = ["Front", "Right", "Back", "Left"]


def get_camera_params() -> CameraConfig:
    """
    Get default camera configuration.

    Returns:
        CameraConfig with default camera parameters
    """
    return CameraConfig(
        streams_enabled=CAMERA_STREAMS_ENABLED,
        recording_enabled=CAMERA_RECORDING_ENABLED,
        recording_codec=CAMERA_RECORDING_CODEC,
        recording_extension=CAMERA_RECORDING_EXTENSION,
        frame_size=CAMERA_FRAME_SIZE,
        fps=CAMERA_FPS,
        initial_delay=CAMERA_INITIAL_DELAY,
        retry_backoff=CAMERA_RETRY_BACKOFF,
        max_backoff=CAMERA_MAX_BACKOFF,
        backoff_multiplier=CAMERA_BACKOFF_MULTIPLIER,
        max_consecutive_failures=CAMERA_MAX_CONSECUTIVE_FAILURES,
        reconnect_timeout=CAMERA_RECONNECT_TIMEOUT,
        frame_read_delay=CAMERA_FRAME_READ_DELAY,
        error_delay=CAMERA_ERROR_DELAY,
        stats_interval=CAMERA_STATS_INTERVAL,
        urls=CAMERA_URLS.copy(),
        order=CAMERA_ORDER.copy(),
    )


def validate_camera_params(config: CameraConfig) -> bool:
    """
    Validate camera parameters.

    Args:
        config: CameraConfig to validate

    Returns:
        True if valid, False otherwise
    """
    issues = []

    # Frame size validation
    if len(config.frame_size) != 2:
        issues.append(f"Frame size must be (width, height): {config.frame_size}")

    width, height = config.frame_size
    if width <= 0 or height <= 0:
        issues.append(f"Frame dimensions must be positive: {config.frame_size}")

    # FPS validation
    if config.fps <= 0:
        issues.append(f"FPS must be positive: {config.fps}")

    # Timing validation
    if config.initial_delay < 0:
        issues.append(f"Initial delay cannot be negative: {config.initial_delay}")

    if config.retry_backoff < 0:
        issues.append(f"Retry backoff cannot be negative: {config.retry_backoff}")

    if config.max_backoff < config.retry_backoff:
        issues.append(
            f"Max backoff ({config.max_backoff}) < "
            f"retry backoff ({config.retry_backoff})"
        )

    # URL validation
    if not config.urls:
        issues.append("Camera URLs dictionary is empty")

    for name in config.order:
        if name not in config.urls:
            issues.append(f"Camera '{name}' in order but not in URLs")

    # Report validation results
    if issues:
        print("Camera parameter validation failed:")
        for issue in issues:
            print(f"  - {issue}")
        return False

    return True


def print_camera_config(config: CameraConfig) -> None:
    """
    Print camera configuration for debugging.

    Args:
        config: CameraConfig to print
    """
    print("\nCAMERA CONFIGURATION:")
    print(f"   Streaming enabled:      {config.streams_enabled}")
    print(f"   Recording enabled:      {config.recording_enabled}")
    print(f"   Frame size:             {config.frame_size[0]}x{config.frame_size[1]}")
    print(f"   FPS:                    {config.fps}")
    print(f"   Codec:                  {config.recording_codec}")
    print(f"   Extension:              {config.recording_extension}")

    print("\n   Camera URLs:")
    for name in config.order:
        url = config.urls.get(name, "NOT CONFIGURED")
        print(f"      {name}: {url}")
    print()
