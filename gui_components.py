"""
GUI Components for Satellite Control System

Reusable PyQt5 GUI components for real-time monitoring and control.
Extracted from real.py to improve code organization and reusability.

Components:
- CameraWidget: Display camera streams
- LivePlotWidget: Real-time matplotlib plotting
- StatusDisplayWidget: System status and telemetry
- ThrusterDisplayWidget: Thruster activation visualization
"""

import cv2
import numpy as np

try:
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
except ImportError:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas  # type: ignore[import]

from typing import Dict, Optional

import matplotlib.patches as patches
from matplotlib.figure import Figure
from PyQt5.QtCore import Qt  # For alignment constants
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from config import SatelliteConfig


class CameraWidget(QWidget):
    """
    Widget for displaying a single camera stream.

    Features:
    - Live video display
    - Connection status indicator
    - Frame rate display
    - Distance overlay
    """

    def __init__(self, camera_name: str, parent=None):
        """
        Initialize camera widget.

        Args:
            camera_name: Name of camera (e.g., "Front", "Right")
            parent: Parent widget
        """
        super().__init__(parent)
        self.camera_name = camera_name

        # Setup UI
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        self.camera_label = QLabel()
        frame_size = SatelliteConfig.CAMERA_FRAME_SIZE
        self.camera_label.setFixedSize(*frame_size)
        self.camera_label.setStyleSheet("background: #222; border: 1px solid #555;")
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # type: ignore[attr-defined]

        self.overlay_label = QLabel("-- cm")
        self.overlay_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # type: ignore[attr-defined]
        self.overlay_label.setStyleSheet(
            "background: rgba(0,0,0,180); "
            "color: #fff; "
            "font-size: 18px; "
            "font-weight: bold; "
            "padding: 4px;"
        )
        self.overlay_label.setFixedHeight(32)

        # Title label
        title_label = QLabel(camera_name)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)  # type: ignore[attr-defined]
        title_label.setStyleSheet("color: #fff; font-size: 12px; font-weight: bold;")

        layout.addWidget(title_label)
        layout.addWidget(self.camera_label)
        layout.addWidget(self.overlay_label)

        # Frame tracking
        self.frame_count = 0
        self.last_frame_time = 0

    def update_frame(self, frame: Optional[np.ndarray]):
        """
        Update the displayed frame.

        Args:
            frame: OpenCV frame (BGR format) or None if unavailable
        """
        if frame is None:
            self.camera_label.setText(f"{self.camera_name}\nDisconnected")
            self.camera_label.setStyleSheet("background: #222; color: #888;")
            return

        try:
            # Convert BGR to RGB
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Convert to QImage
            height, width, channel = frame_rgb.shape
            bytes_per_line = 3 * width
            q_image = QImage(
                frame_rgb.tobytes(), width, height, bytes_per_line, QImage.Format_RGB888
            )

            # Display
            self.camera_label.setPixmap(QPixmap.fromImage(q_image))
            self.camera_label.setStyleSheet("background: #000; border: 1px solid #555;")

            self.frame_count += 1

        except Exception as e:
            print(f"  CameraWidget error ({self.camera_name}): {e}")

    def update_overlay(self, text: str):
        """
        Update overlay text.

        Args:
            text: Text to display (e.g., "123 cm")
        """
        self.overlay_label.setText(text)


class LivePlotWidget(QWidget):
    """
    Widget for real-time 2D plotting using matplotlib.

    Features:
    - Live trajectory plotting
    - Target visualization
    - Obstacle display
    - Satellite orientation indicator
    """

    def __init__(self, parent=None):
        """Initialize live plot widget."""
        super().__init__(parent)

        # Create matplotlib figure
        self.figure = Figure(figsize=(6, 6), facecolor="#2b2b2b")
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)

        # Configure axes
        self.ax.set_facecolor("#1e1e1e")
        self.ax.set_xlabel("X Position (m)", color="white")
        self.ax.set_ylabel("Y Position (m)", color="white")
        self.ax.set_title(
            "Satellite Position", color="white", fontsize=14, fontweight="bold"
        )
        self.ax.tick_params(colors="white")
        self.ax.spines["bottom"].set_color("white")
        self.ax.spines["top"].set_color("white")
        self.ax.spines["right"].set_color("white")
        self.ax.spines["left"].set_color("white")
        self.ax.grid(True, alpha=0.3, color="white")
        self.ax.set_aspect("equal")

        # Layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)

        # Data storage
        self.trajectory_x = []
        self.trajectory_y = []
        self.max_trajectory_points = 500

        self.trajectory_line = None
        self.satellite_marker = None
        self.satellite_orientation = None
        self.target_marker = None

    def update_plot(
        self,
        position: np.ndarray,
        angle: float,
        target_position: Optional[np.ndarray] = None,
        obstacles: Optional[list] = None,
    ):
        """
        Update the plot with new data.

        Args:
            position: [x, y] current position
            angle: Current orientation (radians)
            target_position: Optional [x, y] target position
            obstacles: Optional list of (x, y, radius) obstacles
        """
        try:
            # Add to trajectory
            self.trajectory_x.append(position[0])
            self.trajectory_y.append(position[1])

            # Limit trajectory length
            if len(self.trajectory_x) > self.max_trajectory_points:
                self.trajectory_x.pop(0)
                self.trajectory_y.pop(0)

            # Clear and redraw
            self.ax.clear()

            self.ax.set_facecolor("#1e1e1e")
            self.ax.set_xlabel("X Position (m)", color="white")
            self.ax.set_ylabel("Y Position (m)", color="white")
            self.ax.set_title(
                "Satellite Position", color="white", fontsize=14, fontweight="bold"
            )
            self.ax.tick_params(colors="white")
            self.ax.spines["bottom"].set_color("white")
            self.ax.spines["top"].set_color("white")
            self.ax.spines["right"].set_color("white")
            self.ax.spines["left"].set_color("white")
            self.ax.grid(True, alpha=0.3, color="white")
            self.ax.set_aspect("equal")

            # Plot trajectory
            if len(self.trajectory_x) > 1:
                self.ax.plot(
                    self.trajectory_x,
                    self.trajectory_y,
                    "c-",
                    alpha=0.5,
                    linewidth=1,
                    label="Trajectory",
                )

            # Plot target
            if target_position is not None:
                self.ax.plot(
                    target_position[0],
                    target_position[1],
                    "g*",
                    markersize=20,
                    label="Target",
                    markeredgecolor="white",
                )

            # Plot obstacles
            if obstacles:
                for obs_x, obs_y, obs_r in obstacles:
                    circle = patches.Circle(
                        (obs_x, obs_y), obs_r, color="red", alpha=0.3, label="Obstacle"
                    )
                    self.ax.add_patch(circle)

            # Plot satellite
            satellite_size = 0.1
            satellite_rect = patches.Rectangle(
                (position[0] - satellite_size / 2, position[1] - satellite_size / 2),
                satellite_size,
                satellite_size,
                angle=np.degrees(angle),
                rotation_point="center",
                facecolor="yellow",
                edgecolor="white",
                linewidth=2,
                label="Satellite",
            )
            self.ax.add_patch(satellite_rect)

            # Orientation arrow
            arrow_length = 0.15
            dx = arrow_length * np.cos(angle)
            dy = arrow_length * np.sin(angle)
            self.ax.arrow(
                float(position[0]),
                float(position[1]),
                float(dx),
                float(dy),
                head_width=0.05,
                head_length=0.05,
                fc="red",
                ec="red",
            )

            # Auto-scale with margin
            if len(self.trajectory_x) > 0:
                x_min, x_max = min(self.trajectory_x), max(self.trajectory_x)
                y_min, y_max = min(self.trajectory_y), max(self.trajectory_y)
                margin = 0.5
                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)

            self.ax.legend(
                loc="upper right",
                facecolor="#2b2b2b",
                edgecolor="white",
                labelcolor="white",
            )

            # Redraw
            self.canvas.draw_idle()

        except Exception as e:
            print(f"  LivePlotWidget error: {e}")

    def clear_trajectory(self):
        """Clear the trajectory history."""
        self.trajectory_x.clear()
        self.trajectory_y.clear()


class StatusDisplayWidget(QWidget):
    """
    Widget for displaying system status and telemetry.

    Features:
    - Position and velocity display
    - Error metrics
    - MPC status
    - Connection health
    """

    def __init__(self, parent=None):
        """Initialize status display widget."""
        super().__init__(parent)

        # Setup UI
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)

        # Title
        title = QLabel("System Status")
        title.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)  # type: ignore[attr-defined]
        layout.addWidget(title)

        # Status labels
        self.labels = {}
        status_items = [
            ("position", "Position"),
            ("velocity", "Velocity"),
            ("angle", "Angle"),
            ("pos_error", "Position Error"),
            ("ang_error", "Angle Error"),
            ("mpc_status", "MPC Status"),
            ("solve_time", "Solve Time"),
            ("iteration", "Iteration"),
        ]

        for key, display_name in status_items:
            label = QLabel(f"{display_name}: --")
            label.setStyleSheet("color: white; font-size: 12px; padding: 2px;")
            self.labels[key] = label
            layout.addWidget(label)

        layout.addStretch()

    def update_status(self, status_dict: Dict):
        """
        Update displayed status.

        Args:
            status_dict: Dictionary with status information
        """
        try:
            if "position" in status_dict:
                pos = status_dict["position"]
                self.labels["position"].setText(
                    f"Position: ({pos[0]:.3f}, {pos[1]:.3f}) m"
                )

            if "velocity" in status_dict:
                vel = status_dict["velocity"]
                vel_mag = np.linalg.norm(vel)
                self.labels["velocity"].setText(f"Velocity: {vel_mag:.3f} m/s")

            if "angle" in status_dict:
                angle_deg = np.degrees(status_dict["angle"])
                self.labels["angle"].setText(f"Angle: {angle_deg:.1f}°")

            if "position_error" in status_dict:
                err = status_dict["position_error"]
                self.labels["pos_error"].setText(f"Position Error: {err:.3f} m")

            if "angle_error" in status_dict:
                err_deg = np.degrees(status_dict["angle_error"])
                self.labels["ang_error"].setText(f"Angle Error: {err_deg:.1f}°")

            if "mpc_status" in status_dict:
                self.labels["mpc_status"].setText(
                    f"MPC Status: {status_dict['mpc_status']}"
                )

            if "solve_time" in status_dict:
                solve_ms = status_dict["solve_time"] * 1000
                self.labels["solve_time"].setText(f"Solve Time: {solve_ms:.1f} ms")

            if "iteration" in status_dict:
                self.labels["iteration"].setText(
                    f"Iteration: {status_dict['iteration']}"
                )

        except Exception as e:
            print(f"  StatusDisplayWidget error: {e}")


# Test function
def test_gui_components():
    """Test GUI components in standalone window."""
    import sys

    from PyQt5.QtWidgets import QApplication, QMainWindow

    app = QApplication(sys.argv)

    # Create test window
    window = QMainWindow()
    window.setWindowTitle("GUI Components Test")
    window.setGeometry(100, 100, 1200, 600)

    # Central widget
    central = QWidget()
    layout = QHBoxLayout(central)

    # Add camera widget
    camera = CameraWidget("Test Camera")
    layout.addWidget(camera)

    # Add plot widget
    plot = LivePlotWidget()
    layout.addWidget(plot)

    # Add status widget
    status = StatusDisplayWidget()
    layout.addWidget(status)

    window.setCentralWidget(central)

    # Update timer
    def update_test_data():
        import time

        t = time.time()
        pos = np.array([np.sin(t), np.cos(t)])
        angle = t

        plot.update_plot(pos, angle, target_position=np.array([0, 0]))
        status.update_status(
            {
                "position": pos,
                "velocity": np.array([0.1, 0.1]),
                "angle": angle,
                "position_error": 0.5,
                "angle_error": 0.1,
                "mpc_status": "OPTIMAL",
                "solve_time": 0.15,
                "iteration": int(t),
            }
        )

    timer = QTimer()
    timer.timeout.connect(update_test_data)
    timer.start(50)

    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    test_gui_components()
