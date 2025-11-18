#!/usr/bin/env python3
"""
Real Test vs Simulation Comparison Visualization

Side-by-side comparison visualization between real test data and simulation data,
using the same animation style as visualize.py.

Features:
- Interactive file selection for real test and simulation data
- Side-by-side animation with info panel
- Unified drawing methods matching visualize.py style
- High-quality MP4 animation generation
- Comprehensive comparison metrics display
- Real-time playback with proper timing
"""

import os
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")  # Use non-interactive backend
import csv
import platform
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, List, Optional

import matplotlib.pyplot as plt
from cycler import cycler
from matplotlib.animation import FuncAnimation, writers
from matplotlib.patches import Circle

from config import SatelliteConfig

# Configure ffmpeg path with Config-first logic, then safe fallbacks
try:
    if getattr(SatelliteConfig, "FFMPEG_PATH", None):
        plt.rcParams["animation.ffmpeg_path"] = SatelliteConfig.FFMPEG_PATH
    else:
        # If ffmpeg already in PATH, do nothing
        if shutil.which("ffmpeg") is None:
            if platform.system() == "Darwin":
                brew_ffmpeg = "/opt/homebrew/bin/ffmpeg"
                if os.path.exists(brew_ffmpeg):
                    plt.rcParams["animation.ffmpeg_path"] = brew_ffmpeg
except Exception:
    pass

import warnings

warnings.filterwarnings("ignore")

# Set matplotlib color scheme
plt.rcParams["axes.prop_cycle"] = cycler(
    color=[
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
    ]
)

# Consistent plot styling
plt.rcParams.update(
    {
        "axes.titlesize": 14,
        "axes.titleweight": "normal",
        "axes.labelsize": 12,
        "legend.fontsize": 10,
        "font.family": "DejaVu Sans",
    }
)


class ComparisonVisualizationGenerator:
    """Comparison visualization generator for real test vs simulation data."""

    def __init__(self, real_csv_path: str, sim_csv_path: str) -> None:
        """Initialize the comparison visualization generator.

        Args:
            real_csv_path: Path to real test CSV file
            sim_csv_path: Path to simulation CSV file
        """
        self.real_csv_path = Path(real_csv_path)
        self.sim_csv_path = Path(sim_csv_path)

        # Create output directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_dir = Path("Data/Comparison") / f"comparison_{timestamp}"
        self.output_dir.mkdir(parents=True, exist_ok=True)

        print(f"Output directory: {self.output_dir}")

        # Animation parameters (set defaults first)
        self.fps = 30
        self.speedup_factor = 1
        self.dt = 0.1

        # Load data (this will update fps and dt based on actual data)
        self.real_data = None
        self.sim_data = None
        self._real_rows = []
        self._sim_rows = []

        self.load_data()

        # Satellite configuration
        self.satellite_size = SatelliteConfig.SATELLITE_SIZE
        self.thrusters = {}
        for thruster_id, pos in SatelliteConfig.THRUSTER_POSITIONS.items():
            self.thrusters[thruster_id] = pos

        # Figure references
        self.fig = None
        self.ax_real = None
        self.ax_sim = None
        self.ax_info = None

        print(f" Loaded real test data: {len(self._real_rows)} rows")
        print(f" Loaded simulation data: {len(self._sim_rows)} rows")

    def load_data(self) -> None:
        """Load both real test and simulation CSV data."""
        print(f"\nLoading real test data from: {self.real_csv_path}")
        self._real_rows = []
        with open(self.real_csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self._real_rows.append(row)

        print(f"Loading simulation data from: {self.sim_csv_path}")
        self._sim_rows = []
        with open(self.sim_csv_path, "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self._sim_rows.append(row)

        # Detect timestep from first few rows
        if len(self._real_rows) > 1:
            try:
                t1 = float(
                    self._real_rows[0].get(
                        "Control_Time", self._real_rows[0].get("time", 0)
                    )
                )
                t2 = float(
                    self._real_rows[1].get(
                        "Control_Time", self._real_rows[1].get("time", 0)
                    )
                )
                self.dt = abs(t2 - t1)
                if self.dt > 0:
                    self.fps = min(
                        max(int(1.0 / self.dt), 10), 60
                    )  # Clamp between 10-60
                else:
                    self.dt = 0.1
                    self.fps = 30
                print(f"Detected timestep: {self.dt:.3f}s, FPS: {self.fps}")
            except Exception:
                self.dt = 0.1
                self.fps = 30
                print(f"Using default timestep: {self.dt}s, FPS: {self.fps}")
        else:
            self.dt = 0.1
            self.fps = 30
            print(f"Using default timestep: {self.dt}s, FPS: {self.fps}")

    def parse_command_vector(self, command_str: str) -> np.ndarray:
        """Parse command vector string to numpy array.

        Args:
            command_str: String representation of command vector

        Returns:
            Numpy array of thruster commands
        """
        if not command_str or command_str == "None":
            return np.zeros(8)

        try:
            # Remove brackets and quotes, split by comma
            clean_str = command_str.strip("[]\"'")
            values = [float(x.strip()) for x in clean_str.split(",")]
            if len(values) == 8:
                return np.array(values)
            else:
                return np.zeros(8)
        except Exception as e:
            # Print parsing error for debugging
            print(f"Warning: Could not parse command vector: '{command_str}' - {e}")
            return np.zeros(8)

    def get_active_thrusters(self, command_vector: np.ndarray) -> list:
        """Get list of active thruster IDs from command vector.

        Args:
            command_vector: Array of thruster commands

        Returns:
            List of active thruster IDs (1-8)
        """
        return [i + 1 for i, cmd in enumerate(command_vector) if cmd > 0.5]

    def setup_plot(self) -> None:
        """Initialize matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(20, 9))
        self.fig.suptitle(
            "Real Test vs Simulation Comparison", fontsize=16, fontweight="bold"
        )

        # Create subplots: Real Test | Simulation | Info Panel
        self.ax_real = plt.subplot2grid((1, 5), (0, 0), colspan=2)
        self.ax_sim = plt.subplot2grid((1, 5), (0, 2), colspan=2)
        self.ax_info = plt.subplot2grid((1, 5), (0, 4))

        # Configure info panel
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")

        plt.tight_layout()

    def draw_satellite(
        self,
        ax: Any,
        x: float,
        y: float,
        yaw: float,
        active_thrusters: list,
        color: str = "blue",
    ) -> None:
        """Draw satellite at given position and orientation (matching Visualize.py).

        Args:
            ax: Matplotlib axis to draw on
            x, y: Satellite position
            yaw: Satellite orientation (radians)
            active_thrusters: List of active thruster IDs
            color: Satellite body color
        """
        # Rotation matrix
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        body_corners = np.array(
            [
                [-self.satellite_size / 2, -self.satellite_size / 2],
                [self.satellite_size / 2, -self.satellite_size / 2],
                [self.satellite_size / 2, self.satellite_size / 2],
                [-self.satellite_size / 2, self.satellite_size / 2],
                [-self.satellite_size / 2, -self.satellite_size / 2],
            ]
        )

        # Rotate and translate body
        rotated_body = body_corners @ rotation_matrix.T + np.array([x, y])
        ax.plot(
            rotated_body[:, 0],
            rotated_body[:, 1],
            color=color,
            linewidth=3,
            label="Satellite",
            zorder=15,
        )

        # Fill the satellite body
        ax.fill(
            rotated_body[:, 0], rotated_body[:, 1], color=color, alpha=0.3, zorder=14
        )

        # Draw thrusters
        for thruster_id, (tx, ty) in self.thrusters.items():
            # Rotate thruster position
            thruster_pos = np.array([tx, ty]) @ rotation_matrix.T
            thruster_x = x + thruster_pos[0]
            thruster_y = y + thruster_pos[1]

            # Color and size based on activity
            if thruster_id in active_thrusters:
                t_color = "red"
                size = 80
                marker = "o"
                alpha = 1.0
            else:
                t_color = "gray"
                size = 40
                marker = "s"
                alpha = 0.5

            ax.scatter(
                thruster_x,
                thruster_y,
                c=t_color,
                s=size,
                marker=marker,
                alpha=alpha,
                edgecolors="black",
                linewidth=1,
                zorder=16,
            )

        # Draw orientation arrow
        arrow_length = self.satellite_size * 0.8
        arrow_end_x = x + arrow_length * cos_yaw
        arrow_end_y = y + arrow_length * sin_yaw
        ax.arrow(
            x,
            y,
            arrow_end_x - x,
            arrow_end_y - y,
            head_width=0.08,
            head_length=0.08,
            fc="green",
            ec="green",
            linewidth=2,
            alpha=0.8,
            zorder=17,
        )

    def draw_target(
        self, ax: Any, target_x: float, target_y: float, target_yaw: float
    ) -> None:
        """Draw target position and orientation (matching Visualize.py).

        Args:
            ax: Matplotlib axis to draw on
            target_x, target_y: Target position
            target_yaw: Target orientation (radians)
        """
        ax.scatter(
            target_x,
            target_y,
            c="red",
            s=200,
            marker="x",
            linewidth=4,
            label="Target",
            zorder=10,
        )

        # Target circle
        circle = Circle(
            (target_x, target_y), 0.05, color="red", fill=False, linewidth=2, alpha=0.7
        )
        ax.add_patch(circle)

        # Target orientation arrow
        arrow_length = self.satellite_size * 0.6
        arrow_end_x = target_x + arrow_length * np.cos(target_yaw)
        arrow_end_y = target_y + arrow_length * np.sin(target_yaw)
        ax.arrow(
            target_x,
            target_y,
            arrow_end_x - target_x,
            arrow_end_y - target_y,
            head_width=0.06,
            head_length=0.06,
            fc="red",
            ec="red",
            alpha=0.8,
            linewidth=2,
            zorder=9,
        )

    def draw_trajectory(
        self, ax: Any, trajectory_x: list, trajectory_y: list, color: str
    ) -> None:
        """Draw satellite trajectory (matching Visualize.py).

        Args:
            ax: Matplotlib axis to draw on
            trajectory_x, trajectory_y: Lists of trajectory points
            color: Trajectory color
        """
        if len(trajectory_x) > 1:
            ax.plot(
                trajectory_x,
                trajectory_y,
                color=color,
                linewidth=2,
                alpha=0.8,
                linestyle="-",
                label="Trajectory",
                zorder=5,
            )

    def update_info_panel(self, frame: int) -> None:
        """Update information panel with current data (matching Visualize.py style).

        Args:
            frame: Current frame number
        """
        # If axes haven't been created yet, skip updating info panel
        if self.ax_info is None:
            return

        self.ax_info.clear()
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")
        self.ax_info.set_title("Comparison Info", fontsize=12, weight="bold")

        # Bound frame to valid range
        frame = min(frame, len(self._real_rows) - 1, len(self._sim_rows) - 1)

        # Get current data
        real_row = self._real_rows[frame]
        sim_row = self._sim_rows[frame]

        # Parse real test data
        real_x = float(real_row["Current_X"])
        real_y = float(real_row["Current_Y"])
        real_yaw = float(real_row["Current_Yaw"])
        real_err_x = float(real_row["Error_X"])
        real_err_y = float(real_row["Error_Y"])
        real_err_yaw = float(real_row["Error_Yaw"])
        real_pos_err = np.sqrt(real_err_x**2 + real_err_y**2)
        real_angle_err = abs(np.degrees(real_err_yaw))

        real_cmd = self.parse_command_vector(real_row["Command_Vector"])
        real_active = self.get_active_thrusters(real_cmd)

        # Parse simulation data
        sim_x = float(sim_row["Current_X"])
        sim_y = float(sim_row["Current_Y"])
        sim_yaw = float(sim_row["Current_Yaw"])
        sim_err_x = float(sim_row["Error_X"])
        sim_err_y = float(sim_row["Error_Y"])
        sim_err_yaw = float(sim_row["Error_Yaw"])
        sim_pos_err = np.sqrt(sim_err_x**2 + sim_err_y**2)
        sim_angle_err = abs(np.degrees(sim_err_yaw))

        sim_cmd = self.parse_command_vector(sim_row["Command_Vector"])
        sim_active = self.get_active_thrusters(sim_cmd)

        # Get time
        real_time = float(real_row.get("Control_Time", frame * self.dt))

        # Information text - matching Visualize.py style
        info_text = [
            "COMPARISON",
            "=" * 25,
            f"Frame: {frame}/{min(len(self._real_rows), len(self._sim_rows)) - 1}",
            f"Time: {real_time:.2f}s",
            "",
            "REAL TEST:",
            f"Pos: ({real_x:.3f}, {real_y:.3f})",
            f"Yaw: {np.degrees(real_yaw):.1f}°",
            f"Pos Error: {real_pos_err:.3f} m",
            f"Angle Error: {real_angle_err:.1f}°",
            f"Thrusters: {real_active}",
            f"Count: {len(real_active)}/8",
            "",
            "SIMULATION:",
            f"Pos: ({sim_x:.3f}, {sim_y:.3f})",
            f"Yaw: {np.degrees(sim_yaw):.1f}°",
            f"Pos Error: {sim_pos_err:.3f} m",
            f"Angle Error: {sim_angle_err:.1f}°",
            f"Thrusters: {sim_active}",
            f"Count: {len(sim_active)}/8",
            "",
            "DIFFERENCE:",
            f"ΔPos Error: {abs(real_pos_err - sim_pos_err):.3f} m",
            f"ΔAngle Error: {abs(real_angle_err - sim_angle_err):.1f}°",
            f"ΔThrusters: {abs(len(real_active) - len(sim_active))}",
        ]

        # Display text - matching Visualize.py style
        y_pos = 0.95
        for line in info_text:
            if line.startswith("="):
                weight, size = "normal", 9
            elif line.isupper() and line.endswith(":"):
                weight, size = "bold", 10
            elif line.startswith(("Frame:", "Time:", "COMPARISON")):
                weight, size = "bold", 11
            else:
                weight, size = "normal", 9

            if not line:
                y_pos -= 0.02
                continue

            self.ax_info.text(
                0.05,
                y_pos,
                line,
                fontsize=size,
                weight=weight,
                verticalalignment="top",
                fontfamily="monospace",
            )
            y_pos -= 0.035

    def animate_frame(self, frame: int) -> List[Any]:
        """Animation update function for each frame (matching Visualize.py).

        Args:
            frame: Frame number
        """
        # Ensure axes exist (setup if called directly)
        if self.ax_real is None or self.ax_sim is None or self.ax_info is None:
            # Lazily initialize the plot if missing
            self.setup_plot()

        # Bound frame to valid range first
        frame = min(frame, len(self._real_rows) - 1, len(self._sim_rows) - 1)

        # Clear both plot axes
        if self.ax_real is not None:
            self.ax_real.clear()
        if self.ax_sim is not None:
            self.ax_sim.clear()

        # Calculate axis limits from all data (cache this if needed for performance)
        try:
            real_x_all = [float(row["Current_X"]) for row in self._real_rows]
            real_y_all = [float(row["Current_Y"]) for row in self._real_rows]
            sim_x_all = [float(row["Current_X"]) for row in self._sim_rows]
            sim_y_all = [float(row["Current_Y"]) for row in self._sim_rows]

            all_x = real_x_all + sim_x_all
            all_y = real_y_all + sim_y_all

            margin = 0.5
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
        except Exception:
            # Fallback to default limits
            x_min, x_max = -3, 3
            y_min, y_max = -3, 3

        # Set up both axes (must do each frame after clear)
        for ax, title in [(self.ax_real, "Real Test"), (self.ax_sim, "Simulation")]:
            if ax is None:
                continue
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("X Position (m)", fontsize=12)
            ax.set_ylabel("Y Position (m)", fontsize=12)
            ax.set_title(title, fontsize=14)

        # Get current data (frame already bounded above)
        real_row = self._real_rows[frame]
        sim_row = self._sim_rows[frame]

        # Parse positions and orientations
        real_x = float(real_row["Current_X"])
        real_y = float(real_row["Current_Y"])
        real_yaw = float(real_row["Current_Yaw"])

        sim_x = float(sim_row["Current_X"])
        sim_y = float(sim_row["Current_Y"])
        sim_yaw = float(sim_row["Current_Yaw"])

        # Parse target (same for both)
        target_x = float(real_row["Target_X"])
        target_y = float(real_row["Target_Y"])
        target_yaw = float(real_row["Target_Yaw"])

        # Parse commands
        real_cmd = self.parse_command_vector(real_row["Command_Vector"])
        real_active = self.get_active_thrusters(real_cmd)

        sim_cmd = self.parse_command_vector(sim_row["Command_Vector"])
        sim_active = self.get_active_thrusters(sim_cmd)

        # Draw targets on both axes
        self.draw_target(self.ax_real, target_x, target_y, target_yaw)
        self.draw_target(self.ax_sim, target_x, target_y, target_yaw)

        # Draw trajectories up to current frame
        real_x_traj = [float(self._real_rows[i]["Current_X"]) for i in range(frame + 1)]
        real_y_traj = [float(self._real_rows[i]["Current_Y"]) for i in range(frame + 1)]
        self.draw_trajectory(self.ax_real, real_x_traj, real_y_traj, "blue")

        sim_x_traj = [float(self._sim_rows[i]["Current_X"]) for i in range(frame + 1)]
        sim_y_traj = [float(self._sim_rows[i]["Current_Y"]) for i in range(frame + 1)]
        self.draw_trajectory(self.ax_sim, sim_x_traj, sim_y_traj, "red")

        # Draw satellites
        self.draw_satellite(self.ax_real, real_x, real_y, real_yaw, real_active, "blue")
        self.draw_satellite(self.ax_sim, sim_x, sim_y, sim_yaw, sim_active, "red")

        # Add legends
        if self.ax_real is not None:
            self.ax_real.legend(loc="upper right", fontsize=9)
        if self.ax_sim is not None:
            self.ax_sim.legend(loc="upper right", fontsize=9)

        # Update info panel
        self.update_info_panel(frame)

        return []

    def generate_comparison_plots(self) -> None:
        """Generate comprehensive comparison plots."""
        print(f"\n{'=' * 70}")
        print("GENERATING COMPARISON PLOTS")
        print(f"{'=' * 70}")

        plot_dir = self.output_dir / "plots"
        plot_dir.mkdir(exist_ok=True)

        print("Creating plots...")
        self._generate_trajectory_plot(plot_dir)
        self._generate_position_error_plot(plot_dir)
        self._generate_angular_error_plot(plot_dir)
        self._generate_velocity_plot(plot_dir)
        # Overlay plot showing real and sim thruster usage on the same axes
        self._generate_thruster_usage_overlay_plot(plot_dir)
        self._generate_mpc_performance_plot(plot_dir)
        self._generate_statistics_table(plot_dir)

        print(f" Plots saved to: {plot_dir}")

    def _generate_trajectory_plot(self, plot_dir: Path) -> None:
        """Generate trajectory comparison plot."""
        print("  - Trajectory comparison...")

        fig, ax = plt.subplots(1, 1, figsize=(12, 10))

        # Extract trajectory data
        real_x = [float(row["Current_X"]) for row in self._real_rows]
        real_y = [float(row["Current_Y"]) for row in self._real_rows]
        sim_x = [float(row["Current_X"]) for row in self._sim_rows]
        sim_y = [float(row["Current_Y"]) for row in self._sim_rows]

        # Plot trajectories
        ax.plot(real_x, real_y, "b-", linewidth=2, label="Real Test", alpha=0.8)
        ax.plot(sim_x, sim_y, "r--", linewidth=2, label="Simulation", alpha=0.8)

        # Plot start and end points
        ax.scatter(
            real_x[0],
            real_y[0],
            c="blue",
            s=200,
            marker="o",
            edgecolors="black",
            linewidth=2,
            label="Real Start",
            zorder=10,
        )
        ax.scatter(
            real_x[-1],
            real_y[-1],
            c="blue",
            s=200,
            marker="s",
            edgecolors="black",
            linewidth=2,
            label="Real End",
            zorder=10,
        )
        ax.scatter(
            sim_x[0],
            sim_y[0],
            c="red",
            s=200,
            marker="o",
            edgecolors="black",
            linewidth=2,
            label="Sim Start",
            zorder=10,
        )
        ax.scatter(
            sim_x[-1],
            sim_y[-1],
            c="red",
            s=200,
            marker="s",
            edgecolors="black",
            linewidth=2,
            label="Sim End",
            zorder=10,
        )

        # Plot target
        target_x = float(self._real_rows[-1]["Target_X"])
        target_y = float(self._real_rows[-1]["Target_Y"])
        ax.scatter(
            target_x,
            target_y,
            c="green",
            s=300,
            marker="*",
            edgecolors="black",
            linewidth=2,
            label="Target",
            zorder=15,
        )

        ax.set_xlabel("X Position (m)", fontsize=14)
        ax.set_ylabel("Y Position (m)", fontsize=14)
        ax.set_title(
            "Trajectory Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=11, loc="best")

        plt.tight_layout()
        plt.savefig(
            plot_dir / "trajectory_comparison.png", dpi=300, bbox_inches="tight"
        )
        plt.close()

    def _generate_position_error_plot(self, plot_dir: Path) -> None:
        """Generate position error comparison plot."""
        print("  - Position error comparison...")

        fig, ax = plt.subplots(1, 1, figsize=(14, 6))

        # Extract error data
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_err_x = [float(row["Error_X"]) for row in self._real_rows]
        real_err_y = [float(row["Error_Y"]) for row in self._real_rows]
        real_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(real_err_x, real_err_y)
        ]

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_err_x = [float(row["Error_X"]) for row in self._sim_rows]
        sim_err_y = [float(row["Error_Y"]) for row in self._sim_rows]
        sim_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(sim_err_x, sim_err_y)
        ]

        # Plot position error magnitude
        ax.plot(
            real_time, real_pos_err, "b-", linewidth=2, label="Real Test", alpha=0.8
        )
        ax.plot(
            sim_time, sim_pos_err, "r--", linewidth=2, label="Simulation", alpha=0.8
        )
        ax.axhline(
            y=0.05,
            color="green",
            linestyle="--",
            linewidth=2,
            alpha=0.6,
            label="Target Threshold (5cm)",
        )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Position Error (m)", fontsize=14)
        ax.set_title(
            "Position Error Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

        # Add final errors as text
        ax.text(
            0.02,
            0.98,
            f"Final Error:\nReal: {real_pos_err[-1]:.4f}m\nSim: {sim_pos_err[-1]:.4f}m",
            transform=ax.transAxes,
            fontsize=11,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "position_error.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate angular error comparison plot."""
        print("  - Angular error comparison...")

        fig, ax = plt.subplots(1, 1, figsize=(14, 6))

        # Extract angular error data
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_angle_err = [
            abs(np.degrees(float(row["Error_Yaw"]))) for row in self._real_rows
        ]

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_angle_err = [
            abs(np.degrees(float(row["Error_Yaw"]))) for row in self._sim_rows
        ]

        ax.plot(
            real_time, real_angle_err, "b-", linewidth=2, label="Real Test", alpha=0.8
        )
        ax.plot(
            sim_time, sim_angle_err, "r--", linewidth=2, label="Simulation", alpha=0.8
        )
        ax.axhline(
            y=5.0,
            color="green",
            linestyle="--",
            linewidth=2,
            alpha=0.6,
            label="Target Threshold (5°)",
        )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Angular Error (degrees)", fontsize=14)
        ax.set_title(
            "Angular Error Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

        # Add final errors as text
        ax.text(
            0.02,
            0.98,
            f"Final Error:\nReal: {real_angle_err[-1]:.2f}°\nSim: {sim_angle_err[-1]:.2f}°",
            transform=ax.transAxes,
            fontsize=11,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "angular_error.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_velocity_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude comparison."""
        print("  - Velocity comparison...")

        # Check if velocity columns exist
        has_real_vel = "Current_VX" in self._real_rows[0]
        has_sim_vel = "Current_VX" in self._sim_rows[0]

        if not has_real_vel and not has_sim_vel:
            print("      Skipping: velocity data not available")
            return

        fig, ax = plt.subplots(1, 1, figsize=(14, 6))

        # Extract velocity data
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]

        if has_real_vel:
            real_vx = [float(row["Current_VX"]) for row in self._real_rows]
            real_vy = [float(row["Current_VY"]) for row in self._real_rows]
            real_vel = [np.sqrt(vx**2 + vy**2) for vx, vy in zip(real_vx, real_vy)]
            ax.plot(
                real_time, real_vel, "b-", linewidth=2, label="Real Test", alpha=0.8
            )

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]

        if has_sim_vel:
            sim_vx = [float(row["Current_VX"]) for row in self._sim_rows]
            sim_vy = [float(row["Current_VY"]) for row in self._sim_rows]
            sim_vel = [np.sqrt(vx**2 + vy**2) for vx, vy in zip(sim_vx, sim_vy)]
            ax.plot(
                sim_time, sim_vel, "r--", linewidth=2, label="Simulation", alpha=0.8
            )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Velocity Magnitude (m/s)", fontsize=14)
        ax.set_title(
            "Velocity Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

        plt.tight_layout()
        plt.savefig(plot_dir / "velocity_magnitude.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_thruster_usage_plot(self, plot_dir: Path) -> None:
        """Generate thruster usage comparison plot."""
        print("  - Thruster usage comparison...")

        fig, axes = plt.subplots(2, 1, figsize=(16, 10))

        # Extract thruster data
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_thrusters = []
        for row in self._real_rows:
            cmd = self.parse_command_vector(row["Command_Vector"])
            real_thrusters.append(cmd)
        real_thrusters = np.array(real_thrusters)

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_thrusters = []
        for row in self._sim_rows:
            cmd = self.parse_command_vector(row["Command_Vector"])
            sim_thrusters.append(cmd)
        sim_thrusters = np.array(sim_thrusters)

        # Use get_cmap to obtain tab10 colors (avoid direct attribute access on cm)
        cmap = plt.get_cmap("tab10")
        colors = cmap(np.linspace(0, 1, 8))

        # Real test thruster usage
        ax = axes[0]
        for i in range(8):
            ax.plot(
                real_time,
                real_thrusters[:, i] * (i + 1) * 0.5,
                color=colors[i],
                linewidth=1.5,
                label=f"T{i + 1}",
                alpha=0.8,
            )

        ax.set_ylabel("Thruster Activity", fontsize=12)
        ax.set_title("Real Test Thruster Usage", fontsize=14, fontweight="bold")
        ax.legend(loc="upper right", ncol=4, fontsize=10)
        ax.grid(True, alpha=0.3)

        # Simulation thruster usage
        ax = axes[1]
        for i in range(8):
            ax.plot(
                sim_time,
                sim_thrusters[:, i] * (i + 1) * 0.5,
                color=colors[i],
                linewidth=1.5,
                label=f"T{i + 1}",
                alpha=0.8,
            )

        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Thruster Activity", fontsize=12)
        ax.set_title("Simulation Thruster Usage", fontsize=14, fontweight="bold")
        ax.legend(loc="upper right", ncol=4, fontsize=10)
        ax.grid(True, alpha=0.3)

        plt.suptitle("Thruster Usage Comparison", fontsize=16, fontweight="bold")
        plt.tight_layout()
        plt.savefig(plot_dir / "thruster_comparison.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_thruster_usage_overlay_plot(self, plot_dir: Path) -> None:
        """Generate a bar plot comparing thruster usage between real and sim (matching Visualize.py style)."""
        print("  - Thruster usage overlay plot...")

        # Prepare data
        real_thrusters = np.array(
            [
                self.parse_command_vector(row["Command_Vector"])
                for row in self._real_rows
            ]
        )
        sim_thrusters = np.array(
            [self.parse_command_vector(row["Command_Vector"]) for row in self._sim_rows]
        )

        # Calculate total active time per thruster (matching Visualize.py calculation)
        real_total_time = np.sum(real_thrusters, axis=0) * self.dt
        sim_total_time = np.sum(sim_thrusters, axis=0) * self.dt

        thruster_ids = np.arange(1, 9)  # Thrusters 1-8
        x = np.arange(len(thruster_ids))
        width = 0.35  # Width of bars

        fig, ax = plt.subplots(1, 1, figsize=(14, 8))

        # Create grouped bar plot
        bars_real = ax.bar(
            x - width / 2,
            real_total_time,
            width,
            label="Real Test",
            color="steelblue",
            alpha=0.8,
            edgecolor="darkblue",
            linewidth=1.5,
        )
        bars_sim = ax.bar(
            x + width / 2,
            sim_total_time,
            width,
            label="Simulation",
            color="coral",
            alpha=0.8,
            edgecolor="darkred",
            linewidth=1.5,
        )

        # Add value labels on top of bars
        for bars in [bars_real, bars_sim]:
            for bar in bars:
                height = bar.get_height()
                if height > 0.1:  # Only label if significant
                    ax.text(
                        bar.get_x() + bar.get_width() / 2.0,
                        height + 0.01,
                        f"{height:.2f}s",
                        ha="center",
                        va="bottom",
                        fontsize=9,
                    )

        ax.set_xlabel("Thruster ID", fontsize=14)
        ax.set_ylabel("Total Active Time (seconds)", fontsize=14)
        ax.set_title(
            "Thruster Usage Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.set_xticks(x)
        ax.set_xticklabels(thruster_ids)
        ax.legend(fontsize=12)
        ax.grid(True, alpha=0.4, axis="y")

        plt.tight_layout()
        plt.savefig(
            plot_dir / "thruster_usage_overlay.png", dpi=300, bbox_inches="tight"
        )
        plt.close()

    def _generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance comparison plot."""
        print("  - MPC performance comparison...")

        fig, ax = plt.subplots(1, 1, figsize=(14, 6))

        # Extract MPC solve times
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_solve = [
            float(row.get("MPC_Solve_Time", 0)) * 1000 for row in self._real_rows
        ]

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_solve = [
            float(row.get("MPC_Solve_Time", 0)) * 1000 for row in self._sim_rows
        ]

        ax.plot(real_time, real_solve, "b-", linewidth=2, label="Real Test", alpha=0.8)
        ax.plot(sim_time, sim_solve, "r--", linewidth=2, label="Simulation", alpha=0.8)
        ax.axhline(
            y=self.dt * 1000,
            color="orange",
            linestyle="--",
            linewidth=2,
            label=f"Control DT Limit ({self.dt * 1000:.0f}ms)",
            alpha=0.6,
        )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("MPC Solve Time (milliseconds)", fontsize=14)
        ax.set_title(
            "MPC Performance Comparison - Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
        )
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=12)

        # Add statistics
        real_avg = np.mean(real_solve)
        sim_avg = np.mean(sim_solve)
        ax.text(
            0.98,
            0.98,
            f"Average Solve Time:\nReal: {real_avg:.2f}ms\nSim: {sim_avg:.2f}ms",
            transform=ax.transAxes,
            fontsize=11,
            verticalalignment="top",
            horizontalalignment="right",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightyellow", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "mpc_performance.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_statistics_table(self, plot_dir: Path) -> None:
        """Generate statistical comparison table."""
        print("  - Statistics table...")

        # Calculate statistics
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_err_x = [float(row["Error_X"]) for row in self._real_rows]
        real_err_y = [float(row["Error_Y"]) for row in self._real_rows]
        real_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(real_err_x, real_err_y)
        ]
        real_angle_err = [abs(float(row["Error_Yaw"])) for row in self._real_rows]
        real_solve = [float(row.get("MPC_Solve_Time", 0)) for row in self._real_rows]

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_err_x = [float(row["Error_X"]) for row in self._sim_rows]
        sim_err_y = [float(row["Error_Y"]) for row in self._sim_rows]
        sim_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(sim_err_x, sim_err_y)
        ]
        sim_angle_err = [abs(float(row["Error_Yaw"])) for row in self._sim_rows]
        sim_solve = [float(row.get("MPC_Solve_Time", 0)) for row in self._sim_rows]

        stats_data = [
            [
                "Position Error Mean (m)",
                f"{np.mean(real_pos_err):.4f}",
                f"{np.mean(sim_pos_err):.4f}",
            ],
            [
                "Position Error Max (m)",
                f"{np.max(real_pos_err):.4f}",
                f"{np.max(sim_pos_err):.4f}",
            ],
            [
                "Position Error Std (m)",
                f"{np.std(real_pos_err):.4f}",
                f"{np.std(sim_pos_err):.4f}",
            ],
            [
                "Final Position Error (m)",
                f"{real_pos_err[-1]:.4f}",
                f"{sim_pos_err[-1]:.4f}",
            ],
            [
                "Angle Error Mean (deg)",
                f"{np.degrees(np.mean(real_angle_err)):.2f}",
                f"{np.degrees(np.mean(sim_angle_err)):.2f}",
            ],
            [
                "Angle Error Max (deg)",
                f"{np.degrees(np.max(real_angle_err)):.2f}",
                f"{np.degrees(np.max(sim_angle_err)):.2f}",
            ],
            [
                "Final Angle Error (deg)",
                f"{np.degrees(real_angle_err[-1]):.2f}",
                f"{np.degrees(sim_angle_err[-1]):.2f}",
            ],
        ]

        # Add velocity stats if available
        if "Current_VX" in self._real_rows[0] and "Current_VX" in self._sim_rows[0]:
            real_vx = [float(row["Current_VX"]) for row in self._real_rows]
            real_vy = [float(row["Current_VY"]) for row in self._real_rows]
            real_vel = [np.sqrt(vx**2 + vy**2) for vx, vy in zip(real_vx, real_vy)]

            sim_vx = [float(row["Current_VX"]) for row in self._sim_rows]
            sim_vy = [float(row["Current_VY"]) for row in self._sim_rows]
            sim_vel = [np.sqrt(vx**2 + vy**2) for vx, vy in zip(sim_vx, sim_vy)]

            stats_data.extend(
                [
                    [
                        "Velocity Mean (m/s)",
                        f"{np.mean(real_vel):.4f}",
                        f"{np.mean(sim_vel):.4f}",
                    ],
                    [
                        "Velocity Max (m/s)",
                        f"{np.max(real_vel):.4f}",
                        f"{np.max(sim_vel):.4f}",
                    ],
                    [
                        "Final Velocity (m/s)",
                        f"{real_vel[-1]:.4f}",
                        f"{sim_vel[-1]:.4f}",
                    ],
                ]
            )

        # Thruster usage statistics (per-thruster duty cycle and average active)
        try:
            real_thrusters = np.array(
                [
                    self.parse_command_vector(r["Command_Vector"])
                    for r in self._real_rows
                ]
            )
            sim_thrusters = np.array(
                [self.parse_command_vector(r["Command_Vector"]) for r in self._sim_rows]
            )

            # Total timesteps
            real_n = real_thrusters.shape[0]
            sim_n = sim_thrusters.shape[0]

            # Per-thruster duty cycles (percentage)
            for i in range(8):
                real_count = np.sum(real_thrusters[:, i])
                sim_count = np.sum(sim_thrusters[:, i])
                real_pct = 100.0 * real_count / max(1, real_n)
                sim_pct = 100.0 * sim_count / max(1, sim_n)
                stats_data.append(
                    [f"Thruster T{i + 1} Duty (%)", f"{real_pct:.2f}", f"{sim_pct:.2f}"]
                )

            # Average number of thrusters active per timestep
            real_avg_active = np.mean(np.sum(real_thrusters > 0.5, axis=1))
            sim_avg_active = np.mean(np.sum(sim_thrusters > 0.5, axis=1))
            stats_data.append(
                [
                    "Avg Thrusters Active",
                    f"{real_avg_active:.2f}",
                    f"{sim_avg_active:.2f}",
                ]
            )

            # Total thrust usage (sum of all thruster activations * dt)
            real_total_thrust = np.sum(real_thrusters) * self.dt
            sim_total_thrust = np.sum(sim_thrusters) * self.dt
            stats_data.append(
                [
                    "Total Thrust Usage (s)",
                    f"{real_total_thrust:.2f}",
                    f"{sim_total_thrust:.2f}",
                ]
            )
        except Exception:
            # If parsing fails, skip thruster stats
            pass

        stats_data.extend(
            [
                [
                    "Solve Time Mean (ms)",
                    f"{np.mean(real_solve) * 1000:.2f}",
                    f"{np.mean(sim_solve) * 1000:.2f}",
                ],
                [
                    "Solve Time Max (ms)",
                    f"{np.max(real_solve) * 1000:.2f}",
                    f"{np.max(sim_solve) * 1000:.2f}",
                ],
                ["Test Duration (s)", f"{real_time[-1]:.2f}", f"{sim_time[-1]:.2f}"],
            ]
        )

        # Add absolute error and error percentage columns
        for row in stats_data:
            try:
                real_val = float(row[1])
                sim_val = float(row[2])
                abs_err = abs(real_val - sim_val)
                pct_err = (abs_err / abs(real_val)) * 100 if real_val != 0 else 0.0
                row.extend([f"{abs_err:.4f}", f"{pct_err:.2f}"])
            except ValueError:
                # If not numeric, add empty strings
                row.extend(["N/A", "N/A"])

        # Create table plot
        fig, ax = plt.subplots(figsize=(16, 10))
        ax.axis("tight")
        ax.axis("off")

        table = ax.table(
            cellText=stats_data,
            colLabels=[
                "Metric",
                "Real Test",
                "Simulation",
                "Absolute Error",
                "Error %",
            ],
            cellLoc="center",
            loc="center",
            colWidths=[0.4, 0.15, 0.15, 0.15, 0.15],
        )

        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.8)

        # Style header
        for i in range(5):
            table[(0, i)].set_facecolor("#4CAF50")
            table[(0, i)].set_text_props(weight="bold", color="white")

        # Alternate row colors
        for i in range(1, len(stats_data) + 1):
            for j in range(5):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor("#f0f0f0")

        plt.title(
            "Statistical Comparison: Real Test vs Simulation",
            fontsize=16,
            fontweight="bold",
            pad=20,
        )

        plt.savefig(plot_dir / "statistics_table.png", dpi=300, bbox_inches="tight")
        plt.close()

        # Also save as CSV
        csv_path = plot_dir / "statistics.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["Metric", "Real Test", "Simulation", "Absolute Error", "Error %"]
            )
            writer.writerows(stats_data)

    def _generate_error_analysis_plots(self, plot_dir: Path) -> None:
        """Generate detailed error analysis plots."""
        print("  - Error analysis plots...")

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))

        # Extract error data
        real_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._real_rows)
        ]
        real_err_x = [float(row["Error_X"]) for row in self._real_rows]
        real_err_y = [float(row["Error_Y"]) for row in self._real_rows]
        real_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(real_err_x, real_err_y)
        ]
        real_angle_err = [
            abs(np.degrees(float(row["Error_Yaw"]))) for row in self._real_rows
        ]

        sim_time = [
            float(row.get("Control_Time", i * self.dt))
            for i, row in enumerate(self._sim_rows)
        ]
        sim_err_x = [float(row["Error_X"]) for row in self._sim_rows]
        sim_err_y = [float(row["Error_Y"]) for row in self._sim_rows]
        sim_pos_err = [
            np.sqrt(ex**2 + ey**2) for ex, ey in zip(sim_err_x, sim_err_y)
        ]
        sim_angle_err = [
            abs(np.degrees(float(row["Error_Yaw"]))) for row in self._sim_rows
        ]

        # 1. Error distribution histograms
        ax = axes[0, 0]
        ax.hist(real_pos_err, bins=30, alpha=0.5, label="Real Test", color="blue")
        ax.hist(sim_pos_err, bins=30, alpha=0.5, label="Simulation", color="red")
        ax.set_xlabel("Position Error (m)", fontsize=12)
        ax.set_ylabel("Frequency", fontsize=12)
        ax.set_title("Position Error Distribution", fontsize=14, fontweight="bold")
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)

        # 2. Error difference over time
        ax = axes[0, 1]
        min_len = min(len(real_time), len(sim_time))
        time_common = real_time[:min_len]
        error_diff = [real_pos_err[i] - sim_pos_err[i] for i in range(min_len)]

        ax.plot(time_common, error_diff, "g-", linewidth=2)
        ax.axhline(0, color="k", linestyle="--", alpha=0.3)
        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Error Difference (m)", fontsize=12)
        ax.set_title(
            "Position Error: Real - Simulation", fontsize=14, fontweight="bold"
        )
        ax.grid(True, alpha=0.3)

        # 3. Angle error magnitude comparison
        ax = axes[1, 0]
        ax.plot(
            real_time, real_angle_err, "b-", linewidth=2, label="Real Test", alpha=0.8
        )
        ax.plot(
            sim_time, sim_angle_err, "r--", linewidth=2, label="Simulation", alpha=0.8
        )
        ax.set_xlabel("Time (s)", fontsize=12)
        ax.set_ylabel("Angle Error (deg)", fontsize=12)
        ax.set_title("Angle Error Magnitude", fontsize=14, fontweight="bold")
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)

        # 4. Error correlation plot
        ax = axes[1, 1]
        ax.scatter(real_pos_err[:min_len], sim_pos_err[:min_len], alpha=0.5, s=20)

        # Add diagonal line
        max_error = max(max(real_pos_err[:min_len]), max(sim_pos_err[:min_len]))
        ax.plot(
            [0, max_error], [0, max_error], "r--", linewidth=2, label="Perfect Match"
        )

        # Calculate correlation
        correlation = np.corrcoef(real_pos_err[:min_len], sim_pos_err[:min_len])[0, 1]
        ax.text(
            0.05,
            0.95,
            f"Correlation: {correlation:.3f}",
            transform=ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

        ax.set_xlabel("Real Test Error (m)", fontsize=12)
        ax.set_ylabel("Simulation Error (m)", fontsize=12)
        ax.set_title("Error Correlation", fontsize=14, fontweight="bold")
        ax.legend(fontsize=11)
        ax.grid(True, alpha=0.3)

        plt.suptitle(
            "Error Analysis: Real Test vs Simulation", fontsize=16, fontweight="bold"
        )
        plt.tight_layout()
        plt.savefig(plot_dir / "error_analysis.png", dpi=300, bbox_inches="tight")
        plt.close()

    def generate_animation(self, output_filename: Optional[str] = None) -> None:
        """Generate and save the MP4 animation (matching Visualize.py).

        Args:
            output_filename: Name of output MP4 file (optional)
        """
        if output_filename is None:
            output_filename = "comparison_animation.mp4"

        print(f"\n{'=' * 70}")
        print("GENERATING COMPARISON ANIMATION")
        print(f"{'=' * 70}")
        print("Setting up animation...")
        self.setup_plot()
        if self.fig is None:
            raise RuntimeError("Failed to create figure")

        # Calculate number of frames
        total_frames = min(len(self._real_rows), len(self._sim_rows))

        print("Animation parameters:")
        print(f"  - Real test data points: {len(self._real_rows)}")
        print(f"  - Simulation data points: {len(self._sim_rows)}")
        print(f"  - Animation frames: {total_frames}")
        print(f"  - Frame rate: {self.fps} FPS")
        print(f"  - Animation duration: {total_frames / self.fps:.1f} seconds")

        # Create animation
        print("\nCreating animation...")
        ani = FuncAnimation(
            self.fig,
            self.animate_frame,
            frames=total_frames,
            interval=1000 / self.fps,
            blit=True,
            repeat=False,
        )

        # Save animation
        output_path = self.output_dir / output_filename
        print(f"Saving animation to: {output_path}")

        try:
            # Prefer explicit ffmpeg writer config with safe args
            Writer = writers["ffmpeg"]
            writer = Writer(
                fps=self.fps,
                metadata=dict(artist="Comparison Visualizer"),
                bitrate=2000,
                codec="libx264",
            )

            # Save animation with progress callback
            ani.save(
                str(output_path),
                writer=writer,
                progress_callback=self._progress_callback,
            )
            print("\n Animation saved successfully!")
            print(f" File location: {output_path}")

        except Exception as e:
            print(f" Error saving MP4 with ffmpeg: {e}")
            print("↪  Attempting GIF fallback (Pillow)...")
            try:
                from matplotlib.animation import PillowWriter

                gif_path = output_path.with_suffix(".gif")
                gif_writer = PillowWriter(fps=max(1, int(self.fps / 2)))
                ani.save(
                    str(gif_path),
                    writer=gif_writer,
                    progress_callback=self._progress_callback,
                )
                print("\n GIF animation saved successfully!")
                print(f" File location: {gif_path}")
            except Exception as e2:
                print(f" GIF fallback failed: {e2}")
                print("  Skipping animation export.")
        finally:
            plt.close(self.fig)

    def _progress_callback(self, current_frame: int, total_frames: int) -> None:
        """Progress callback for animation saving with visual progress bar.

        Args:
            current_frame: Current frame being saved
            total_frames: Total number of frames
        """
        progress = (current_frame / total_frames) * 100
        bar_length = 40
        filled = int(bar_length * current_frame / total_frames)
        bar = "█" * filled + "░" * (bar_length - filled)

        print(
            f"\rAnimating: |{bar}| {progress:.1f}% ({current_frame}/{total_frames} frames)",
            end="",
            flush=True,
        )

        if current_frame == total_frames - 1:
            print()  # New line when complete


def select_csv_file(prompt: str, search_dir: str) -> Optional[Path]:
    """Interactive CSV file selector.

    Args:
        prompt: Prompt to display to user
        search_dir: Directory to search for CSV files

    Returns:
        Path to selected CSV file or None if cancelled
    """
    print(f"\n{'=' * 70}")
    print(f"  {prompt}")
    print(f"{'=' * 70}")

    search_path = Path(search_dir)
    if not search_path.exists():
        print(f" Error: Directory not found: {search_path}")
        return None

    # Find all CSV files recursively
    csv_files = list(search_path.rglob("*.csv"))

    if not csv_files:
        print(f" No CSV files found in {search_path}")
        return None

    # Sort by modification time (newest first)
    csv_files.sort(key=lambda p: p.stat().st_mtime, reverse=True)

    # Display available files
    print(f"\nFound {len(csv_files)} CSV file(s):\n")
    for idx, csv_path in enumerate(csv_files[:20], 1):  # Show max 20
        file_size = csv_path.stat().st_size / 1024  # KB
        mod_time = datetime.fromtimestamp(csv_path.stat().st_mtime).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        rel_path = csv_path.relative_to(search_path)
        print(f"{idx:2d}. {rel_path} ({file_size:.1f} KB) - {mod_time}")

    if len(csv_files) > 20:
        print(f"    ... and {len(csv_files) - 20} more files")

    # Get user selection
    print("\n" + "-" * 70)
    while True:
        try:
            choice = input(
                f"Select file (1-{min(20, len(csv_files))}) or 'q' to quit: "
            ).strip()
            if choice.lower() == "q":
                print("Cancelled.")
                return None

            idx = int(choice) - 1
            if 0 <= idx < min(20, len(csv_files)):
                selected = csv_files[idx]
                print(f"\n Selected: {selected}")
                return selected
            else:
                print(f"Invalid selection. Please enter 1-{min(20, len(csv_files))}")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None


def main() -> int:
    """Main function for standalone usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate side-by-side comparison visualization of real test vs simulation"
    )
    parser.add_argument("--real", type=str, help="Path to real test CSV file")
    parser.add_argument("--sim", type=str, help="Path to simulation CSV file")
    parser.add_argument(
        "--interactive", action="store_true", help="Interactive file selection"
    )

    args = parser.parse_args()

    # Interactive mode if no arguments or explicitly requested
    if args.interactive or (args.real is None and args.sim is None):
        print("\n" + "=" * 70)
        print("   REAL TEST vs SIMULATION COMPARISON TOOL")
        print("=" * 70)

        # Select real test file
        real_csv = select_csv_file(" SELECT REAL TEST DATA", "Data/Real_Test")
        if real_csv is None:
            return 1

        # Select simulation file
        sim_csv = select_csv_file(" SELECT SIMULATION DATA", "Data/Simulation")
        if sim_csv is None:
            return 1

        args.real = str(real_csv)
        args.sim = str(sim_csv)

    # Validate inputs
    if args.real is None or args.sim is None:
        print(" Error: Both --real and --sim paths are required")
        return 1

    real_path = Path(args.real)
    sim_path = Path(args.sim)

    if not real_path.exists():
        print(f" Error: Real test file not found: {real_path}")
        return 1

    if not sim_path.exists():
        print(f" Error: Simulation file not found: {sim_path}")
        return 1

    try:
        # Create comparison visualizer
        viz = ComparisonVisualizationGenerator(
            real_csv_path=str(real_path), sim_csv_path=str(sim_path)
        )

        # Generate animation
        viz.generate_animation()

        # Generate comparison plots
        viz.generate_comparison_plots()

        print(f"\n{'=' * 70}")
        print(" COMPARISON COMPLETE")
        print(f"{'=' * 70}")
        print(f"Output directory: {viz.output_dir}")
        print(f"{'=' * 70}\n")

    except Exception as e:
        print(f" Error: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
