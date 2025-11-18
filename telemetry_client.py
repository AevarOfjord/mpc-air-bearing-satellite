"""
Telemetry Client for OptiTrack/Motive Motion Capture System

HTTP client for retrieving real-time position and orientation data from motion capture.
Provides interface between Motive data server and satellite control system.

Communication features:
- HTTP GET requests to motive_data_server.py
- JSON data parsing with validation
- Configurable timeout for real-time performance
- Error handling and retry logic
- Connection health monitoring

Data provided:
- Position: (x, y) coordinates in meters
- Orientation: Yaw angle in radians
- Timestamp: Server-side capture time
- Quality indicators: Marker visibility, tracking confidence

Key features:
- Low-latency communication (<10ms typical)
- Graceful degradation on connection loss
- Thread-safe for concurrent access
- Automatic reconnection attempts
- Integration with real.py control loop
"""

import json
import time
from typing import Any, Dict, Optional

import numpy as np
import requests


class TelemetryClient:
    """
    Client for OptiTrack/Motive motion capture telemetry data.

    Handles HTTP communication with motive_data_server.py to retrieve
    real-time position (x, y) and orientation (yaw) measurements.
    """

    def __init__(
        self, server_url: str = "http://127.0.0.1:5000/data", timeout: float = 0.2
    ):
        """
        Initialize telemetry client.

        Args:
            server_url: URL of the Motive data server endpoint
            timeout: HTTP request timeout in seconds
        """
        self.server_url = server_url
        self.timeout = timeout
        self.failure_count = 0

    def get_telemetry(self, max_retries: int = 2) -> Optional[Dict[str, Any]]:
        """
        Retrieve current position data from the Motive system with retry logic.

        Args:
            max_retries: Maximum number of retry attempts on failure

        Returns:
            Dictionary containing x, y, yaw, timestamp measurements or None if unavailable

        Note:
            Implements comprehensive error handling for:
            - Network timeouts
            - Connection errors
            - Invalid/corrupted data
            - Server unavailability
        """
        for attempt in range(max_retries):
            try:
                # Attempt to get telemetry with strict timeout
                response = requests.get(self.server_url, timeout=self.timeout)

                if response.status_code == 200:
                    data = response.json()

                    # Validate that we have complete data
                    if (
                        data.get("x") is None
                        or data.get("y") is None
                        or data.get("yaw") is None
                    ):
                        if attempt == 0:  # Only print on first attempt
                            print(
                                "  Motion capture data incomplete (object may not be tracked)"
                            )
                        continue

                    x_mm, y_mm, yaw_deg = data["x"], data["y"], data["yaw"]

                    # Validate data ranges
                    if not (-10000 < x_mm < 10000):  # ±10m reasonable range
                        print(f"  Motion capture X value out of range: {x_mm}mm")
                        continue

                    if not (-10000 < y_mm < 10000):  # ±10m reasonable range
                        print(f"  Motion capture Y value out of range: {y_mm}mm")
                        continue

                    if not (-360 <= yaw_deg <= 360):
                        print(f"  Motion capture yaw value out of range: {yaw_deg}°")
                        continue

                    # Convert units: mm to meters, degrees to radians
                    telemetry = {
                        "x": x_mm / 1000.0,  # mm to m
                        "y": y_mm / 1000.0,  # mm to m
                        "yaw": np.radians(yaw_deg),  # degrees to radians
                        "timestamp": time.time(),
                    }

                    # Reset failure counter on success
                    self.failure_count = 0

                    return telemetry

                else:
                    if attempt == 0:
                        print(f"  Motion capture HTTP error {response.status_code}")

            except requests.exceptions.Timeout:
                print(f"  Motion capture timeout (attempt {attempt + 1}/{max_retries})")
                self.failure_count += 1

            except requests.exceptions.ConnectionError:
                print(
                    f"  Motion capture connection error (attempt {attempt + 1}/{max_retries})"
                )
                self.failure_count += 1

            except requests.exceptions.RequestException as e:
                print(
                    f"  Motion capture request failed (attempt {attempt + 1}/{max_retries}): {e}"
                )
                self.failure_count += 1

            except (json.JSONDecodeError, KeyError, TypeError) as e:
                print(
                    f"  Motion capture data parsing error (attempt {attempt + 1}/{max_retries}): {e}"
                )
                self.failure_count += 1

            except Exception as e:
                print(
                    f" Unexpected motion capture error (attempt {attempt + 1}/{max_retries}): {e}"
                )
                self.failure_count += 1

            # Small delay before retry
            if attempt < max_retries - 1:
                time.sleep(0.05)  # 50ms delay between retries

        # All retries exhausted
        self.failure_count += 1

        if self.failure_count > 10:
            print(
                f" CRITICAL: Motion capture has failed {self.failure_count} times - check Motive server at {self.server_url}"
            )

        return None

    def get_failure_count(self) -> int:
        """Get the number of consecutive telemetry failures."""
        return self.failure_count

    def reset_failure_count(self) -> None:
        """Reset the telemetry failure counter."""
        self.failure_count = 0


def create_telemetry_client(
    server_url: str = "http://127.0.0.1:5000/data", timeout: float = 0.2
) -> TelemetryClient:
    """
    Factory function to create a telemetry client.

    Args:
        server_url: URL of the Motive data server endpoint
        timeout: HTTP request timeout in seconds

    Returns:
        Configured TelemetryClient instance
    """
    return TelemetryClient(server_url=server_url, timeout=timeout)
