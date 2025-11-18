"""
Unit tests for camera_manager.py module.

Tests the CameraManager class which handles multi-camera streaming
and recording for satellite control systems.
"""

import tempfile
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from camera_manager import CameraManager, create_camera_manager


class TestCameraManagerInitialization:
    """Test CameraManager initialization."""

    def test_basic_initialization(self):
        """Test basic CameraManager initialization."""
        camera_urls = {"Front": "http://192.168.1.100", "Back": "http://192.168.1.101"}
        camera_order = ["Front", "Back"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            assert manager.camera_urls == camera_urls
            assert manager.camera_order == camera_order
            assert manager.data_dir == Path(tmpdir)
            assert manager.running is False

    def test_initialization_with_custom_settings(self):
        """Test initialization with custom frame size and FPS."""
        camera_urls = {"Cam1": "http://example.com"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
                frame_size=(1280, 720),
                fps=60,
            )

            assert manager.frame_size == (1280, 720)
            assert manager.fps == 60

    def test_initialization_creates_frame_dict(self):
        """Test that initialization creates frame dictionary for all cameras."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2", "Cam3": "url3"}
        camera_order = ["Cam1", "Cam2", "Cam3"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            assert len(manager.cam_frames) == 3
            assert "Cam1" in manager.cam_frames
            assert "Cam2" in manager.cam_frames
            assert "Cam3" in manager.cam_frames
            # All should be None initially
            assert all(frame is None for frame in manager.cam_frames.values())


class TestFactoryFunction:
    """Test the create_camera_manager factory function."""

    def test_create_camera_manager(self):
        """Test creating camera manager via factory."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = create_camera_manager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            assert isinstance(manager, CameraManager)
            assert manager.camera_urls == camera_urls


class TestCameraManagerStart:
    """Test camera manager start functionality."""

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_start_creates_threads(self, mock_config, mock_thread):
        """Test that start() creates worker threads for each camera."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2"}
        camera_order = ["Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Mock the thread
            mock_thread_instance = MagicMock()
            mock_thread.return_value = mock_thread_instance

            manager.start(recording_enabled=False)

            # Should create 2 threads (one per camera)
            assert mock_thread.call_count == 2
            assert manager.running is True

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_start_with_recording_enabled(self, mock_config, mock_thread):
        """Test starting with recording enabled."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            mock_thread_instance = MagicMock()
            mock_thread.return_value = mock_thread_instance

            manager.start(recording_enabled=True)

            assert manager.recording_enabled is True

    @patch("camera_manager.logger")
    @patch("camera_manager.SatelliteConfig")
    def test_start_with_cameras_disabled(self, mock_config, mock_logger):
        """Test that start respects camera disable setting."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        # Create mock config with cameras disabled
        mock_cfg = MagicMock()
        mock_cfg.CAMERA_STREAMS_ENABLED = False

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            manager.start(recording_enabled=False, camera_config=mock_cfg)

            # Should not start if cameras disabled
            assert manager.running is False


class TestCameraManagerStop:
    """Test camera manager stop functionality."""

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_stop_when_not_running(self, mock_config, mock_thread):
        """Test that stop() handles case when not running."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Should not crash when stopping without starting
            manager.stop()
            assert manager.running is False

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_stop_sets_running_false(self, mock_config, mock_thread):
        """Test that stop() sets running flag to False."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            mock_thread_instance = MagicMock()
            mock_thread.return_value = mock_thread_instance

            manager.start()
            assert manager.running is True

            manager.stop()
            assert manager.running is False

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_stop_clears_resources(self, mock_config, mock_thread):
        """Test that stop() clears all resource dictionaries."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2"}
        camera_order = ["Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            mock_thread_instance = MagicMock()
            mock_thread.return_value = mock_thread_instance

            manager.start()
            manager.stop()

            assert len(manager.captures) == 0
            assert len(manager.writers) == 0
            assert len(manager.cam_threads) == 0


class TestGetFrames:
    """Test frame retrieval functionality."""

    def test_get_frames_returns_copy(self):
        """Test that get_frames() returns a copy of frames dict."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            frames1 = manager.get_frames()
            frames2 = manager.get_frames()

            # Should be equal but not the same object
            assert frames1 == frames2
            assert frames1 is not frames2

    def test_get_frame_returns_specific_camera(self):
        """Test getting frame from specific camera."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2"}
        camera_order = ["Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Set a mock frame for Cam1
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            manager.cam_frames["Cam1"] = test_frame

            retrieved = manager.get_frame("Cam1")
            assert retrieved is test_frame

    def test_get_frame_nonexistent_camera(self):
        """Test getting frame from camera that doesn't exist."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            frame = manager.get_frame("NonexistentCam")
            assert frame is None

    def test_get_frames_initially_none(self):
        """Test that all frames are None initially."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2"}
        camera_order = ["Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            frames = manager.get_frames()
            assert all(frame is None for frame in frames.values())


class TestFrameHandling:
    """Test frame data handling."""

    def test_frame_storage(self):
        """Test that frames can be stored and retrieved."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Create a test frame
            test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            manager.cam_frames["Cam1"] = test_frame

            # Retrieve and verify
            retrieved = manager.get_frame("Cam1")
            assert retrieved is not None
            assert np.array_equal(retrieved, test_frame)

    def test_multiple_camera_frames(self):
        """Test storing frames for multiple cameras."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2", "Cam3": "url3"}
        camera_order = ["Cam1", "Cam2", "Cam3"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Set different frames for each camera
            frame1 = np.ones((480, 640, 3), dtype=np.uint8) * 50
            frame2 = np.ones((480, 640, 3), dtype=np.uint8) * 100
            frame3 = np.ones((480, 640, 3), dtype=np.uint8) * 150

            manager.cam_frames["Cam1"] = frame1
            manager.cam_frames["Cam2"] = frame2
            manager.cam_frames["Cam3"] = frame3

            # Verify each frame
            retrieved_frame1 = manager.get_frame("Cam1")
            assert retrieved_frame1 is not None
            assert np.array_equal(retrieved_frame1, frame1)

            retrieved_frame2 = manager.get_frame("Cam2")
            assert retrieved_frame2 is not None
            assert np.array_equal(retrieved_frame2, frame2)

            retrieved_frame3 = manager.get_frame("Cam3")
            assert retrieved_frame3 is not None
            assert np.array_equal(retrieved_frame3, frame3)


class TestCameraConfiguration:
    """Test camera configuration handling."""

    def test_custom_frame_size(self):
        """Test using custom frame size."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
                frame_size=(1920, 1080),
            )

            assert manager.frame_size == (1920, 1080)

    def test_custom_fps(self):
        """Test using custom FPS."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
                fps=24,
            )

            assert manager.fps == 24

    def test_data_directory_creation(self):
        """Test that data directory path is stored correctly."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            data_path = Path(tmpdir) / "test_data"
            manager = CameraManager(
                camera_urls=camera_urls, camera_order=camera_order, data_dir=data_path
            )

            assert manager.data_dir == data_path


class TestCameraOrder:
    """Test camera ordering functionality."""

    def test_camera_order_preserved(self):
        """Test that camera order is preserved."""
        camera_urls = {"Back": "url1", "Front": "url2", "Side": "url3"}
        camera_order = ["Front", "Side", "Back"]  # Different from dict order

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            assert manager.camera_order == ["Front", "Side", "Back"]

    def test_frames_dict_matches_order(self):
        """Test that frames dict contains all cameras in order."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2", "Cam3": "url3"}
        camera_order = ["Cam3", "Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # All cameras from order should be in frames dict
            for cam in camera_order:
                assert cam in manager.cam_frames


class TestThreadSafety:
    """Test thread safety considerations."""

    def test_concurrent_frame_access(self):
        """Test that concurrent frame access doesn't crash."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Set a frame
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            manager.cam_frames["Cam1"] = test_frame

            # Multiple get_frames calls should not crash
            frames1 = manager.get_frames()
            frames2 = manager.get_frames()
            frames3 = manager.get_frames()

            assert frames1 is not None
            assert frames2 is not None
            assert frames3 is not None


class TestEdgeCases:
    """Test edge cases and error conditions."""

    def test_empty_camera_list(self):
        """Test initialization with empty camera list."""
        camera_urls = {}
        camera_order = []

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            assert len(manager.cam_frames) == 0

    def test_camera_in_order_but_no_url(self):
        """Test camera in order but missing from URLs."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1", "Cam2"]  # Cam2 has no URL

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Should still have frame slot for Cam2
            assert "Cam2" in manager.cam_frames

    def test_stop_before_start(self):
        """Test calling stop before start."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Should not crash
            manager.stop()
            assert manager.running is False

    def test_multiple_stops(self):
        """Test calling stop multiple times."""
        camera_urls = {"Cam1": "url1"}
        camera_order = ["Cam1"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = CameraManager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            # Multiple stops should not crash
            manager.stop()
            manager.stop()
            manager.stop()
            assert manager.running is False


class TestIntegration:
    """Integration tests for complete workflows."""

    @patch("camera_manager.threading.Thread")
    @patch("camera_manager.SatelliteConfig")
    def test_complete_start_stop_cycle(self, mock_config, mock_thread):
        """Test complete start-stop cycle."""
        camera_urls = {"Cam1": "url1", "Cam2": "url2"}
        camera_order = ["Cam1", "Cam2"]

        with tempfile.TemporaryDirectory() as tmpdir:
            manager = create_camera_manager(
                camera_urls=camera_urls,
                camera_order=camera_order,
                data_dir=Path(tmpdir),
            )

            mock_thread_instance = MagicMock()
            mock_thread.return_value = mock_thread_instance

            # Start
            manager.start(recording_enabled=True)
            assert manager.running is True
            assert manager.recording_enabled is True

            # Stop
            manager.stop()
            assert manager.running is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
