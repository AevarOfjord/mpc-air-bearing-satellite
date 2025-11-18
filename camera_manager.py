"""
Camera Management System for Multi-Camera Recording and Streaming

Handles concurrent camera streams from multiple sources (HTTP/RTSP/local)
with robust error handling, automatic reconnection, and optional recording.

Supports three operational modes:
1. Dashboard mode: Frame streaming for live display + optional recording
2. Window mode: Qt GUI window with camera feeds and overlays
3. Headless mode: Background recording only (no GUI)
"""

import logging
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import QGridLayout, QLabel, QMainWindow, QVBoxLayout, QWidget

# Import SatelliteConfig at module level for test patching
from config import SatelliteConfig

logger = logging.getLogger(__name__)


class CameraManager:
    """
    Unified camera management for multiple concurrent camera streams.

    Features:
    - Multi-camera streaming with thread-per-camera architecture
    - Automatic reconnection with exponential backoff
    - Optional video recording to disk (per-camera)
    - Frame validation and corruption detection
    - Graceful degradation on failures
    - Health monitoring and statistics
    """

    def __init__(
        self,
        camera_urls: Dict[str, str],
        camera_order: list,
        data_dir: Path,
        frame_size: Tuple[int, int] = (640, 480),
        fps: int = 30,
    ):
        """
        Initialize camera manager.

        Args:
            camera_urls: Dictionary mapping camera names to URLs
            camera_order: List of camera names in display order
            data_dir: Directory for saving recordings
            frame_size: Frame dimensions (width, height)
            fps: Frames per second for recording
        """
        self.camera_urls = camera_urls
        self.camera_order = camera_order
        self.data_dir = Path(data_dir)
        self.frame_size = frame_size
        self.fps = fps

        # Camera state
        self.running = False
        self.cam_threads: Dict[str, threading.Thread] = {}
        self.cam_frames: Dict[str, Optional[np.ndarray]] = {
            k: None for k in camera_order
        }
        self.captures: Dict[str, Optional[cv2.VideoCapture]] = {}
        self.writers: Dict[str, Optional[cv2.VideoWriter]] = {}

    def start(
        self, recording_enabled: bool = False, camera_config: Optional[Any] = None
    ) -> None:
        """
        Start all camera streams with optional recording.

        Args:
            recording_enabled: Whether to record video to disk
            camera_config: Optional config object with camera settings
        """
        # Check if cameras are enabled
        if camera_config and not getattr(camera_config, "CAMERA_STREAMS_ENABLED", True):
            logger.info("Camera streams disabled via Config.CAMERA_STREAMS_ENABLED")
            return

        self.running = True
        self.recording_enabled = recording_enabled
        self.camera_config = camera_config or SatelliteConfig

        logger.info(f"Starting camera manager with {len(self.camera_order)} cameras")

        for cam in self.camera_order:
            url = self.camera_urls.get(cam)
            if url:
                t = threading.Thread(
                    target=self._camera_worker,
                    args=(cam, url),
                    daemon=True,
                    name=f"Camera-{cam}",
                )
                t.start()
                self.cam_threads[cam] = t

    def stop(self) -> None:
        """Stop all camera streams and release resources."""
        if not self.running:
            return

        logger.info("Stopping camera manager...")
        self.running = False

        # Wait for threads to exit
        for _cam, t in list(self.cam_threads.items()):
            try:
                t.join(timeout=2.0)
            except Exception:
                pass

        # Release all captures
        for _cam, cap in list(self.captures.items()):
            try:
                if cap is not None:
                    cap.release()
            except Exception:
                pass

        # Release all writers
        for _cam, writer in list(self.writers.items()):
            try:
                if writer is not None:
                    writer.release()
            except Exception:
                pass

        self.captures.clear()
        self.writers.clear()
        self.cam_threads.clear()
        logger.info("Camera manager stopped")

    def get_frames(self) -> Dict[str, Optional[np.ndarray]]:
        """Get current frames from all cameras."""
        return self.cam_frames.copy()

    def get_frame(self, camera_name: str) -> Optional[np.ndarray]:
        """Get current frame from specific camera."""
        return self.cam_frames.get(camera_name)

    def _camera_worker(self, cam: str, url: str) -> None:
        """
        Camera worker thread with robust error handling and automatic recovery.

        Features:
        - Automatic reconnection with exponential backoff
        - Graceful degradation on recording failures
        - Frame validation and corruption detection
        - Connection health monitoring

        Args:
            cam: Camera name
            url: Camera stream URL
        """
        cfg = self.camera_config

        # Track camera health
        consecutive_failures = 0
        max_consecutive_failures = getattr(cfg, "CAMERA_MAX_CONSECUTIVE_FAILURES", 10)
        total_frames = 0
        failed_frames = 0
        last_success_time = time.time()

        def open_capture(u: str, max_attempts: int = 3) -> Optional[cv2.VideoCapture]:
            """Open capture with retries and validation."""
            for attempt in range(max_attempts):
                try:
                    cap_local = cv2.VideoCapture(u)
                    # Give HTTP streams time to initialize
                    time.sleep(getattr(cfg, "CAMERA_INITIAL_DELAY", 0.2))

                    if not cap_local.isOpened():
                        logger.debug(
                            f"{cam}: Camera not opened (attempt {attempt + 1}/{max_attempts})"
                        )
                        if attempt < max_attempts - 1:
                            time.sleep(
                                getattr(cfg, "CAMERA_RETRY_BACKOFF", 0.5)
                                * (attempt + 1)
                            )
                        continue

                    # Try to read a test frame to verify stream works
                    ret, test_frame = cap_local.read()
                    if not ret or test_frame is None:
                        logger.debug(
                            f"{cam}: Cannot read test frame (attempt {attempt + 1}/{max_attempts})"
                        )
                        cap_local.release()
                        if attempt < max_attempts - 1:
                            time.sleep(
                                getattr(cfg, "CAMERA_RETRY_BACKOFF", 0.5)
                                * (attempt + 1)
                            )
                        continue

                    logger.info(f"{cam}: Camera connected successfully")
                    return cap_local

                except Exception as e:
                    logger.warning(
                        f"{cam}: Connection error (attempt {attempt + 1}/{max_attempts}): {e}"
                    )
                    if attempt < max_attempts - 1:
                        time.sleep(
                            getattr(cfg, "CAMERA_RETRY_BACKOFF", 0.5) * (attempt + 1)
                        )

            logger.error(f"{cam}: Failed to connect after {max_attempts} attempts")
            return None

        # Setup recording if enabled
        writer = None
        if self.recording_enabled:
            codec = getattr(cfg, "CAMERA_RECORDING_CODEC", "MJPG")
            ext = getattr(cfg, "CAMERA_RECORDING_EXTENSION", ".avi")
            fourcc = cv2.VideoWriter_fourcc(*codec)  # type: ignore[attr-defined]
            out_path = str(self.data_dir / f"{cam}_video{ext}")

            try:
                tmp_writer = cv2.VideoWriter(
                    out_path, fourcc, self.fps, self.frame_size
                )
                if tmp_writer is not None and tmp_writer.isOpened():
                    writer = tmp_writer
                    logger.info(f"{cam}: Recording to {out_path}")
                else:
                    logger.warning(
                        f"{cam}: Failed to open video writer. Recording disabled."
                    )
                    writer = None
            except Exception as e:
                logger.warning(f"{cam}: VideoWriter error: {e}. Recording disabled.")
                writer = None

        # Initial connection
        cap = open_capture(url)
        self.captures[cam] = cap

        if writer is not None:
            self.writers[cam] = writer

        backoff = getattr(cfg, "CAMERA_RETRY_BACKOFF", 0.5)
        max_backoff = getattr(cfg, "CAMERA_MAX_BACKOFF", 5.0)

        # Main camera loop
        while self.running:
            try:
                # Handle reconnection if needed
                if cap is None or not cap.isOpened():
                    if self.running:
                        logger.debug(f"{cam}: Reconnecting...")
                        consecutive_failures += 1

                        if consecutive_failures > max_consecutive_failures:
                            logger.warning(
                                f"{cam}: Too many failures ({consecutive_failures}), "
                                f"backing off {max_backoff}s"
                            )
                            time.sleep(max_backoff)
                            consecutive_failures = 0  # Reset to try again

                    cap = open_capture(url)
                    self.captures[cam] = cap

                    if cap is None:
                        time.sleep(backoff)
                        backoff = min(
                            max_backoff,
                            backoff * getattr(cfg, "CAMERA_BACKOFF_MULTIPLIER", 1.5),
                        )
                        continue

                # Attempt to read frame
                ret, frame = cap.read()
                total_frames += 1

                if not ret or frame is None:
                    failed_frames += 1
                    consecutive_failures += 1

                    reconnect_timeout = getattr(cfg, "CAMERA_RECONNECT_TIMEOUT", 30)
                    if time.time() - last_success_time > reconnect_timeout:
                        logger.warning(
                            f"{cam}: No frames for {reconnect_timeout}s, forcing reconnect"
                        )
                        if cap is not None:
                            cap.release()
                        cap = None
                        self.captures[cam] = None
                        continue

                    time.sleep(getattr(cfg, "CAMERA_FRAME_READ_DELAY", 0.05))
                    continue

                # Validate frame
                if frame.shape[0] == 0 or frame.shape[1] == 0:
                    logger.warning(f"{cam}: Invalid frame dimensions {frame.shape}")
                    failed_frames += 1
                    consecutive_failures += 1
                    time.sleep(getattr(cfg, "CAMERA_FRAME_READ_DELAY", 0.05))
                    continue

                # Success - reset counters
                consecutive_failures = 0
                backoff = getattr(cfg, "CAMERA_RETRY_BACKOFF", 0.5)
                last_success_time = time.time()

                # Resize and store frame
                frame = cv2.resize(frame, self.frame_size)
                self.cam_frames[cam] = frame.copy()

                # Write to recording if enabled
                if writer is not None:
                    try:
                        writer.write(frame)
                    except Exception as e:
                        logger.warning(
                            f"{cam}: Recording error: {e}. Disabling recording."
                        )
                        try:
                            writer.release()
                        except Exception:
                            pass
                        writer = None
                        if cam in self.writers:
                            del self.writers[cam]

                # Report statistics periodically
                stats_interval = getattr(cfg, "CAMERA_STATS_INTERVAL", 1000)
                if total_frames % stats_interval == 0 and total_frames > 0:
                    success_rate = 100 * (1 - failed_frames / total_frames)
                    logger.info(
                        f"{cam}: {total_frames} frames, {success_rate:.1f}% success rate"
                    )

            except cv2.error as e:
                logger.error(f"{cam}: OpenCV error: {e}")
                consecutive_failures += 1
                time.sleep(getattr(cfg, "CAMERA_ERROR_DELAY", 1.0))

            except Exception as e:
                logger.error(f"{cam}: Unexpected error: {e}")
                consecutive_failures += 1
                time.sleep(getattr(cfg, "CAMERA_ERROR_DELAY", 1.0))

        # Cleanup on exit
        try:
            if cap is not None:
                cap.release()
        except Exception:
            pass

        try:
            if writer is not None:
                writer.release()
        except Exception:
            pass


class SatelliteCameraWindow(QMainWindow):
    """Qt window that shows live camera feeds with distance overlays."""

    def __init__(
        self,
        camera_manager: CameraManager,
        get_distances_func: Callable[[], Dict[str, str]],
        overlay_height: int = 40,
    ):
        """
        Initialize camera window.

        Args:
            camera_manager: CameraManager instance managing the camera streams
            get_distances_func: Function that returns distance info for overlays
            overlay_height: Height of overlay labels in pixels
        """
        super().__init__()
        self.camera_manager = camera_manager
        self.get_distances = get_distances_func
        self.overlay_height = overlay_height

        self.setWindowTitle("Satellite Cameras")
        self.setGeometry(120, 120, 1320, 680)

        # Build camera grid layout
        central = QWidget()
        grid = QGridLayout(central)

        self.cam_labels = {}
        self.overlay_labels = {}

        for i, cam in enumerate(camera_manager.camera_order):
            vbox = QVBoxLayout()

            # Camera frame display
            cam_label = QLabel()
            cam_label.setFixedSize(*camera_manager.frame_size)
            cam_label.setStyleSheet("background: #222;")

            # Distance overlay
            overlay = QLabel("-- cm")
            overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
            overlay.setStyleSheet(
                "background: rgba(0,0,0,180); color: #fff; font-size: 22px;"
            )
            overlay.setFixedHeight(overlay_height)
            overlay.setFont(QFont("Arial", 16, QFont.Weight.Bold))

            vbox.addWidget(cam_label)
            vbox.addWidget(overlay)
            grid.addLayout(vbox, i // 2, i % 2)

            self.cam_labels[cam] = cam_label
            self.overlay_labels[cam] = overlay

        self.setCentralWidget(central)

        # UI updater timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_views)
        self.timer.start(50)  # 50ms update interval for UI responsiveness

    def update_views(self):
        """Update camera displays and overlays."""
        distances = self.get_distances()
        frames = self.camera_manager.get_frames()

        for cam in self.camera_manager.camera_order:
            frame = frames.get(cam)
            if frame is not None:
                # Convert BGR to RGB for Qt display
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.tobytes(), w, h, ch * w, QImage.Format.Format_RGB888)
                self.cam_labels[cam].setPixmap(QPixmap.fromImage(qimg))

            # Update distance overlay
            self.overlay_labels[cam].setText(f"{distances.get(cam, '--')} cm")

    def closeEvent(self, event):  # type: ignore[override]
        """Handle window close event."""
        self.timer.stop()
        event.accept()


def create_camera_manager(
    camera_urls: Dict[str, str],
    camera_order: list,
    data_dir: Path,
    frame_size: Tuple[int, int] = (640, 480),
    fps: int = 30,
) -> CameraManager:
    """
    Factory function to create a camera manager.

    Args:
        camera_urls: Dictionary mapping camera names to URLs
        camera_order: List of camera names in display order
        data_dir: Directory for saving recordings
        frame_size: Frame dimensions (width, height)
        fps: Frames per second for recording

    Returns:
        Configured CameraManager instance
    """
    return CameraManager(
        camera_urls=camera_urls,
        camera_order=camera_order,
        data_dir=data_dir,
        frame_size=frame_size,
        fps=fps,
    )
