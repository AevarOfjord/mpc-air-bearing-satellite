"""
Real-time UI Components for Satellite Hardware Testing

PyQt5-based visualization windows for live monitoring during hardware tests.
Provides multi-camera views, 2D navigation maps, and real-time telemetry display.

UI components:
- SatelliteLiveDashboard: Integrated multi-camera view with 2D map and telemetry
- LiveAnimationWindow: Standalone matplotlib-based animation window
- Camera integration with distance overlay display
- Real-time state plotting and trajectory visualization

Key features:
- Live camera feeds from multiple sources
- Real-time 2D position and trajectory plotting
- Thruster activation visualization
- Distance sensor overlays
- Target and obstacle visualization
- Configurable update rates for performance
- Integration with camera_manager for multi-camera support
"""

from pathlib import Path
from typing import Any, Callable, Dict, Optional, Tuple, Union

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.patches import Circle
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

from camera_manager import create_camera_manager
from config import SatelliteConfig

# Camera configuration from SatelliteConfig
CAMERA_URLS = SatelliteConfig.CAMERA_URLS
CAMERA_ORDER = SatelliteConfig.CAMERA_ORDER


class SatelliteLiveDashboard(QMainWindow):
    """
    Live dashboard window for real-time satellite monitoring.

    Displays:
    - Multi-camera video feeds with distance overlays
    - Live 2D map showing satellite trajectory, target, and obstacles
    - Thruster status and orientation
    - DXF shape overlays (if applicable)

    The dashboard updates at 50ms intervals for smooth visualization.
    """

    def __init__(
        self,
        data_dir: Union[str, Path],
        get_position_angle_func: Callable[[], Tuple[np.ndarray, float]],
        get_distances_func: Callable[[], Dict[str, float]],
        get_map_state_func: Optional[Callable[[], Dict[str, Any]]] = None,
    ) -> None:
        """
        Initialize the live dashboard.

        Args:
            data_dir: Directory for saving camera recordings
            get_position_angle_func: Callback to get (position, angle) from controller
            get_distances_func: Callback to get distance measurements
            get_map_state_func: Optional callback to get comprehensive map state
        """
        super().__init__()
        self.setWindowTitle("Satellite Live Dashboard")
        self.setGeometry(
            100, 100, SatelliteConfig.WINDOW_WIDTH, SatelliteConfig.WINDOW_HEIGHT
        )
        self.data_dir = Path(data_dir)
        self.get_position_angle = get_position_angle_func
        self.get_distances = get_distances_func
        self.get_map_state = get_map_state_func

        # Initialize camera manager
        self.camera_manager = create_camera_manager(
            camera_urls=CAMERA_URLS,
            camera_order=CAMERA_ORDER,
            data_dir=self.data_dir,
            frame_size=SatelliteConfig.CAMERA_FRAME_SIZE,
            fps=SatelliteConfig.CAMERA_FPS,
        )

        self.frame_size = SatelliteConfig.CAMERA_FRAME_SIZE
        self.fps = SatelliteConfig.CAMERA_FPS
        self.cam_frames = {}  # Will be populated from camera_manager

        self.initUI()
        # Start camera streams (with optional recording)
        self.camera_manager.start(
            recording_enabled=getattr(
                SatelliteConfig, "CAMERA_RECORDING_ENABLED", False
            )
        )

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_dashboard)
        self.timer.start(50)  # 50ms update interval for UI responsiveness

    def initUI(self) -> None:
        """Initialize the UI layout with camera grid and map canvas."""
        central = QWidget()
        main_layout = QHBoxLayout(central)
        # Camera grid
        cam_grid = QGridLayout()
        self.cam_labels = {}
        self.overlay_labels = {}
        for i, cam in enumerate(CAMERA_ORDER):
            vbox = QVBoxLayout()
            cam_label = QLabel()
            cam_label.setFixedSize(*self.frame_size)
            cam_label.setStyleSheet("background: #222;")
            overlay = QLabel("-- cm")
            overlay.setAlignment(Qt.AlignmentFlag.AlignCenter)
            overlay.setStyleSheet(
                "background: rgba(0,0,0,180); color: #fff; font-size: 22px;"
            )
            overlay.setFixedHeight(SatelliteConfig.OVERLAY_HEIGHT)
            overlay.setFont(QFont("Arial", 16, QFont.Bold))
            vbox.addWidget(cam_label)
            vbox.addWidget(overlay)
            cam_grid.addLayout(vbox, i // 2, i % 2)
            self.cam_labels[cam] = cam_label
            self.overlay_labels[cam] = overlay
        main_layout.addLayout(cam_grid, 2)
        self.map_canvas = FigureCanvas(plt.figure(figsize=(5, 5)))
        main_layout.addWidget(self.map_canvas, 1)
        self.setCentralWidget(central)

    def update_dashboard(self) -> None:
        """Update camera views and 2D map (called every 50ms)."""
        # Update camera views and overlays from camera manager
        distances = self.get_distances()
        self.cam_frames = self.camera_manager.get_frames()  # Update local cache
        for cam in CAMERA_ORDER:
            frame = self.cam_frames.get(cam)
            if frame is not None:
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qimg = QImage(rgb.tobytes(), w, h, ch * w, QImage.Format_RGB888)
                self.cam_labels[cam].setPixmap(QPixmap.fromImage(qimg))
            # Overlay distance
            val = distances.get(cam, "--")
            self.overlay_labels[cam].setText(f"{val} cm")
        # Update 2D map
        self.update_map()

    def update_map(self) -> None:
        """Render the live 2D map to match the post-mission animation styling."""
        fig = self.map_canvas.figure
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")

        try:
            if callable(self.get_map_state):
                state: Dict[str, Any] = self.get_map_state()  # type: ignore
                # Trajectory
                traj = state.get("trajectory") or []
                if len(traj) > 1:
                    traj_arr = np.array(traj)
                    ax.plot(
                        traj_arr[:, 0],
                        traj_arr[:, 1],
                        color="cyan",
                        linewidth=2,
                        alpha=0.8,
                        linestyle="-",
                        label="Trajectory",
                    )

                # Target
                tx, ty = state.get("target_x"), state.get("target_y")
                tyaw = state.get("target_yaw")
                if tx is not None and ty is not None and tyaw is not None:
                    ax.scatter(
                        tx, ty, c="red", s=200, marker="x", linewidth=4, label="Target"
                    )
                    tgt_circle = Circle(
                        (tx, ty), 0.05, color="red", fill=False, linewidth=2, alpha=0.7
                    )
                    ax.add_patch(tgt_circle)
                    # Target orientation arrow
                    s_size = float(state.get("satellite_size", 0.2))
                    arr_len = s_size * 0.6
                    ax.arrow(
                        tx,
                        ty,
                        arr_len * np.cos(tyaw) - 0,
                        arr_len * np.sin(tyaw) - 0,
                        head_width=0.06,
                        head_length=0.06,
                        fc="red",
                        ec="red",
                        alpha=0.8,
                        linewidth=2,
                    )

                # DXF overlays
                base_shape = state.get("dxf_base_shape") or []
                offset_path = state.get("dxf_offset_path") or []
                dxf_center = state.get("dxf_center")
                if isinstance(base_shape, (list, tuple)) and len(base_shape) >= 3:
                    bx = [p[0] for p in base_shape] + [base_shape[0][0]]
                    by = [p[1] for p in base_shape] + [base_shape[0][1]]
                    ax.plot(
                        bx,
                        by,
                        color="#9b59b6",
                        linewidth=2.5,
                        alpha=0.7,
                        label="Object Shape",
                    )
                if isinstance(offset_path, (list, tuple)) and len(offset_path) >= 2:
                    px = [p[0] for p in offset_path]
                    py = [p[1] for p in offset_path]
                    ax.plot(
                        px,
                        py,
                        color="#e67e22",
                        linewidth=2,
                        alpha=0.8,
                        linestyle="--",
                        label="Shape Path",
                    )
                if isinstance(dxf_center, (list, tuple)) and len(dxf_center) == 2:
                    ax.scatter(
                        dxf_center[0],
                        dxf_center[1],
                        c="#2ecc71",
                        s=60,
                        marker="+",
                        linewidth=2,
                        label="Shape Center",
                    )

                # Obstacles
                if state.get("obstacles_enabled"):
                    for i, (ox, oy, orad) in enumerate(state.get("obstacles", []), 1):
                        oc = Circle(
                            (ox, oy),
                            orad,
                            fill=True,
                            color="red",
                            alpha=0.6,
                            edgecolor="darkred",
                            linewidth=2,
                            zorder=15,
                        )
                        ax.add_patch(oc)
                        ax.text(
                            ox,
                            oy,
                            f"O{i}",
                            fontsize=8,
                            color="white",
                            ha="center",
                            va="center",
                            fontweight="bold",
                            zorder=16,
                        )

                # Satellite body and thrusters
                x, y = state.get("x", 0.0), state.get("y", 0.0)
                yaw = state.get("yaw", 0.0)
                s_size = float(state.get("satellite_size", 0.2))
                c, s = np.cos(yaw), np.sin(yaw)
                R = np.array([[c, -s], [s, c]])
                body = np.array(
                    [
                        [-s_size / 2, -s_size / 2],
                        [s_size / 2, -s_size / 2],
                        [s_size / 2, s_size / 2],
                        [-s_size / 2, s_size / 2],
                        [-s_size / 2, -s_size / 2],
                    ]
                )
                body_w = body @ R.T + np.array([x, y])
                ax.plot(
                    body_w[:, 0],
                    body_w[:, 1],
                    color="blue",
                    linewidth=3,
                    label="Satellite",
                )
                ax.fill(body_w[:, 0], body_w[:, 1], color="blue", alpha=0.3)

                # Thrusters
                thrusters = state.get("thrusters") or {}
                active = set(state.get("active_thrusters") or [])
                for tid, (tx, ty) in thrusters.items():
                    pos_w = np.array([tx, ty]) @ R.T
                    txw, tyw = x + pos_w[0], y + pos_w[1]
                    if tid in active:
                        color, size, marker, alpha = "red", 80, "o", 1.0
                    else:
                        color, size, marker, alpha = "gray", 40, "s", 0.5
                    ax.scatter(
                        txw,
                        tyw,
                        c=color,
                        s=size,
                        marker=marker,
                        alpha=alpha,
                        edgecolors="black",
                        linewidth=1,
                    )

                # Orientation arrow
                arr_len = s_size * 0.8
                ax.arrow(
                    x,
                    y,
                    arr_len * c,
                    arr_len * s,
                    head_width=0.08,
                    head_length=0.08,
                    fc="green",
                    ec="green",
                    linewidth=2,
                    alpha=0.8,
                )

                # Legend
                ax.legend(loc="upper right", fontsize=9)
            else:
                pos, angle = self.get_position_angle()
                x, y = pos
                ax.plot(x, y, "bo", markersize=12)
                ax.arrow(
                    x,
                    y,
                    0.2 * np.cos(angle),
                    0.2 * np.sin(angle),
                    head_width=0.07,
                    head_length=0.1,
                    fc="b",
                    ec="b",
                )

        except Exception as e:
            # Ensure UI never crashes due to plotting
            ax.text(
                0.5,
                0.5,
                f"Map render error: {e}",
                ha="center",
                va="center",
                transform=ax.transAxes,
                color="red",
            )

        fig.tight_layout()
        self.map_canvas.draw()

    def closeEvent(self, a0):  # type: ignore
        """Handle window close event."""
        self.camera_manager.stop()
        a0.accept()  # type: ignore


class LiveAnimationWindow(QMainWindow):
    """
    A standalone window that renders a live 2D animation similar to the post-mission one.

    This provides a simplified view focusing on trajectory, target, and satellite position
    without the camera feeds. Updates at 100ms intervals.
    """

    def __init__(self, get_map_state_func):
        """
        Initialize the live animation window.

        Args:
            get_map_state_func: Callback to get comprehensive map state dict
        """
        super().__init__()
        self.setWindowTitle("Satellite Live Animation")
        self.setGeometry(180, 160, 700, 700)
        self.get_map_state = get_map_state_func
        central = QWidget()
        layout = QVBoxLayout(central)
        self.canvas = FigureCanvas(plt.figure(figsize=(5, 5)))
        layout.addWidget(self.canvas)
        self.setCentralWidget(central)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)

    def update_map(self):
        """Update the 2D animation (called every 100ms)."""
        fig = self.canvas.figure
        fig.clear()
        ax = fig.add_subplot(111)
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X Position (m)")
        ax.set_ylabel("Y Position (m)")
        try:
            if callable(self.get_map_state):
                state: Dict[str, Any] = self.get_map_state()  # type: ignore
                traj = state.get("trajectory") or []
                if len(traj) > 1:
                    traj_arr = np.array(traj)
                    ax.plot(
                        traj_arr[:, 0],
                        traj_arr[:, 1],
                        color="cyan",
                        linewidth=2,
                        alpha=0.8,
                        linestyle="-",
                        label="Trajectory",
                    )
                tx, ty = state.get("target_x"), state.get("target_y")
                tyaw = state.get("target_yaw")
                if tx is not None and ty is not None and tyaw is not None:
                    ax.scatter(
                        tx, ty, c="red", s=200, marker="x", linewidth=4, label="Target"
                    )
                    tgt_circle = Circle(
                        (tx, ty), 0.05, color="red", fill=False, linewidth=2, alpha=0.7
                    )
                    ax.add_patch(tgt_circle)
                    s_size = float(state.get("satellite_size", 0.2))
                    arr_len = s_size * 0.6
                    ax.arrow(
                        tx,
                        ty,
                        arr_len * np.cos(tyaw),
                        arr_len * np.sin(tyaw),
                        head_width=0.06,
                        head_length=0.06,
                        fc="red",
                        ec="red",
                        alpha=0.8,
                        linewidth=2,
                    )
                base_shape = state.get("dxf_base_shape") or []
                offset_path = state.get("dxf_offset_path") or []
                dxf_center = state.get("dxf_center")
                if isinstance(base_shape, (list, tuple)) and len(base_shape) >= 3:
                    bx = [p[0] for p in base_shape] + [base_shape[0][0]]
                    by = [p[1] for p in base_shape] + [base_shape[0][1]]
                    ax.plot(
                        bx,
                        by,
                        color="#9b59b6",
                        linewidth=2.5,
                        alpha=0.7,
                        label="Object Shape",
                    )
                if isinstance(offset_path, (list, tuple)) and len(offset_path) >= 2:
                    px = [p[0] for p in offset_path]
                    py = [p[1] for p in offset_path]
                    ax.plot(
                        px,
                        py,
                        color="#e67e22",
                        linewidth=2,
                        alpha=0.8,
                        linestyle="--",
                        label="Shape Path",
                    )
                if isinstance(dxf_center, (list, tuple)) and len(dxf_center) == 2:
                    ax.scatter(
                        dxf_center[0],
                        dxf_center[1],
                        c="#2ecc71",
                        s=60,
                        marker="+",
                        linewidth=2,
                        label="Shape Center",
                    )
                if state.get("obstacles_enabled"):
                    for i, (ox, oy, orad) in enumerate(state.get("obstacles", []), 1):
                        oc = Circle(
                            (ox, oy),
                            orad,
                            fill=True,
                            color="red",
                            alpha=0.6,
                            edgecolor="darkred",
                            linewidth=2,
                            zorder=15,
                        )
                        ax.add_patch(oc)
                        ax.text(
                            ox,
                            oy,
                            f"O{i}",
                            fontsize=8,
                            color="white",
                            ha="center",
                            va="center",
                            fontweight="bold",
                            zorder=16,
                        )
                x, y = state.get("x", 0.0), state.get("y", 0.0)
                yaw = state.get("yaw", 0.0)
                s_size = float(state.get("satellite_size", 0.2))
                c, s = np.cos(yaw), np.sin(yaw)
                R = np.array([[c, -s], [s, c]])
                body = np.array(
                    [
                        [-s_size / 2, -s_size / 2],
                        [s_size / 2, -s_size / 2],
                        [s_size / 2, s_size / 2],
                        [-s_size / 2, s_size / 2],
                        [-s_size / 2, -s_size / 2],
                    ]
                )
                body_w = body @ R.T + np.array([x, y])
                ax.plot(
                    body_w[:, 0],
                    body_w[:, 1],
                    color="blue",
                    linewidth=3,
                    label="Satellite",
                )
                ax.fill(body_w[:, 0], body_w[:, 1], color="blue", alpha=0.3)
                ax.arrow(
                    x,
                    y,
                    s_size * 0.8 * c,
                    s_size * 0.8 * s,
                    head_width=0.08,
                    head_length=0.08,
                    fc="green",
                    ec="green",
                    linewidth=2,
                    alpha=0.8,
                )
                ax.legend(loc="upper right", fontsize=9)
        except Exception as e:
            ax.text(
                0.5,
                0.5,
                f"Map render error: {e}",
                ha="center",
                va="center",
                transform=ax.transAxes,
                color="red",
            )
        fig.tight_layout()
        self.canvas.draw()
