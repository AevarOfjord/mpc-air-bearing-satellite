#!/usr/bin/env python3
"""
Unified Visualization Module for MPC Satellite Control

Unified visualization system for both simulation and real testing systems.
Automatically generates MP4 animations and performance plots from CSV data.

Features:
- Unified interface for both simulation and real test data
- Automatic data detection and loading
- High-quality MP4 animation generation
- Comprehensive performance analysis plots
- Configurable titles and labels based on data source
- Real-time animation with proper timing
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
from typing import Any, Dict, List, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, writers
from matplotlib.axes import Axes
from matplotlib.figure import Figure

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

# Import cycler and Circle from correct modules
from cycler import cycler
from matplotlib.patches import Circle

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


class UnifiedVisualizationGenerator:
    """Unified visualization generator for both simulation and real test data."""

    def __init__(
        self,
        data_directory: str,
        mode: str = "simulation",
        interactive: bool = False,
        load_data: bool = True,
        prefer_pandas: bool = True,
        overlay_dxf: bool = False,
    ):
        """Initialize the visualization generator.

        Args:
            data_directory: Base directory containing the data
            mode: Either "simulation" or "real" to determine titles and labels
            interactive: If True, allow user to select which data to visualize
            load_data: If True, automatically locate and load the newest CSV
            prefer_pandas: If True, try pandas; otherwise use csv module backend
            overlay_dxf: If True, overlay DXF shape on trajectory (if available)
        """
        self.data_directory = Path(data_directory)
        self.mode = mode.lower()
        self.csv_path: Optional[Path] = None
        self.output_dir: Optional[Path] = None
        self.data: Optional[Any] = None  # DataFrame when pandas, self when csv backend
        self.fig: Optional[Figure] = None
        self.ax_main: Optional[Axes] = None
        self.ax_info: Optional[Axes] = None
        # Data backend management
        self._data_backend: Optional[str] = None  # 'pandas' or 'csv'
        self._rows: Optional[
            List[Dict[str, Any]]
        ] = None  # list of dicts when csv backend
        self._col_data: Optional[
            Dict[str, np.ndarray]
        ] = None  # dict of column -> array when csv backend
        self.use_pandas = prefer_pandas
        self.overlay_dxf = overlay_dxf  # Option to overlay DXF shape

        # Validate mode
        if self.mode not in ["simulation", "real"]:
            raise ValueError("Mode must be either 'simulation' or 'real'")

        # Set titles and labels based on mode
        self._setup_labels()

        self.fps: Optional[float] = None  # Will be calculated from simulation timestep
        self.speedup_factor = 1.0  # Real-time playback (no speedup)
        self.real_time = True  # Enable real-time animation
        self.dt: Optional[float] = None  # Simulation timestep (will be auto-detected)

        self.satellite_size = SatelliteConfig.SATELLITE_SIZE
        self.satellite_color = "blue"
        self.target_color = "red"
        self.trajectory_color = "cyan"

        self.dxf_base_shape: List[Any] = getattr(SatelliteConfig, "DXF_BASE_SHAPE", [])
        self.dxf_offset_path = getattr(SatelliteConfig, "DXF_SHAPE_PATH", [])
        self.dxf_center = getattr(SatelliteConfig, "DXF_SHAPE_CENTER", None)

        self.thrusters = {}
        for thruster_id, pos in SatelliteConfig.THRUSTER_POSITIONS.items():
            self.thrusters[thruster_id] = pos

        self.thruster_forces = SatelliteConfig.THRUSTER_FORCES.copy()

        if load_data:
            if interactive:
                self.select_data_interactively()
            else:
                self.find_newest_data()

    def _setup_labels(self) -> None:
        """Setup titles and labels based on the mode."""
        if self.mode == "simulation":
            self.system_name = "Simulation"
            self.system_title = "MPC Simulation"
            self.data_source = "Simulation Data"
            self.plot_prefix = "Simulation"
            self.animation_title = "MPC Satellite Simulation Visualization"
            self.trajectory_title = "MPC - Satellite Trajectory"
            self.frame_title_template = "MPC - Frame {}"
        else:  # real
            self.system_name = "Real Test"
            self.system_title = "Real MPC Test"
            self.data_source = "Real Test Data"
            self.plot_prefix = "Real_Test"
            self.animation_title = "Real MPC Satellite Test Visualization"
            self.trajectory_title = "Real MPC - Satellite Trajectory"
            self.frame_title_template = "Real MPC - Frame {}"

    def find_newest_data(self) -> None:
        """Find the newest data folder and CSV file."""
        print(f"Searching for {self.system_name} data in: {self.data_directory}")

        if not self.data_directory.exists():
            raise FileNotFoundError(f"Data directory not found: {self.data_directory}")

        # First, check if the current directory itself contains CSV files
        csv_patterns = ["simulation_data.csv", "real_test_data.csv", "*.csv"]
        csv_files = []

        for pattern in csv_patterns:
            csv_files = list(self.data_directory.glob(pattern))
            if csv_files:
                # Current directory has CSV files - use it directly
                self.csv_path = csv_files[0]
                self.output_dir = self.data_directory
                print(f"Using CSV file: {self.csv_path.name} in current directory")
                self.load_csv_data()
                return

        # If no CSV in current directory, search subdirectories
        data_folders = [d for d in self.data_directory.iterdir() if d.is_dir()]

        if not data_folders:
            raise FileNotFoundError(f"No data folders found in: {self.data_directory}")

        data_folders.sort(key=lambda x: x.stat().st_mtime, reverse=True)

        print(f"Found {len(data_folders)} data folders")

        # Try folders until we find one with CSV data
        newest_folder = None
        csv_files = []

        for folder in data_folders:
            print(f"Checking folder: {folder.name}")

            for pattern in csv_patterns:
                csv_files = list(folder.glob(pattern))
                if csv_files:
                    break

            if csv_files:
                newest_folder = folder
                print(f"Using folder: {newest_folder.name}")
                break
            else:
                print("  No CSV files found, trying next folder...")

        if not csv_files or newest_folder is None:
            raise FileNotFoundError("No CSV files found in any data folder")

        self.csv_path = csv_files[0]
        self.output_dir = newest_folder

        print(f"Using CSV file: {self.csv_path.name}")
        print(f"Output directory: {self.output_dir}")

        # Load and validate data
        self.load_csv_data()

    def select_data_interactively(self) -> None:
        """Allow user to interactively select which data to visualize."""
        print(f"\n{'=' * 60}")
        print(f"INTERACTIVE {self.system_name.upper()} DATA SELECTION")
        print(f"{'=' * 60}")

        if not self.data_directory.exists():
            raise FileNotFoundError(f"Data directory not found: {self.data_directory}")

        # Find all subdirectories with CSV files
        data_folders = []
        for folder in self.data_directory.iterdir():
            if folder.is_dir():
                csv_files = list(folder.glob("*.csv"))
                if csv_files:
                    data_folders.append((folder, csv_files))

        if not data_folders:
            raise FileNotFoundError(
                f"No folders with CSV data found in: {self.data_directory}"
            )

        data_folders.sort(key=lambda x: x[0].stat().st_mtime, reverse=True)

        print(f"Found {len(data_folders)} folders with data:")
        print()

        for i, (folder, csv_files) in enumerate(data_folders, 1):
            # Get folder timestamp
            timestamp = datetime.fromtimestamp(folder.stat().st_mtime)
            print(f"{i:2}. {folder.name}")
            print(f"    Modified: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"    CSV files: {len(csv_files)}")
            print()

        # Get user selection
        while True:
            try:
                choice = input(
                    f"Select folder (1-{len(data_folders)}) or 'q' to quit: "
                ).strip()
                if choice.lower() == "q":
                    print("Visualization cancelled.")
                    sys.exit(0)
                folder_idx = int(choice) - 1
                if 0 <= folder_idx < len(data_folders):
                    selected_folder, csv_files = data_folders[folder_idx]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(data_folders)}")
            except ValueError:
                print("Please enter a valid number or 'q' to quit")

        if len(csv_files) > 1:
            print(f"\nMultiple CSV files found in {selected_folder.name}:")
            for i, csv_file in enumerate(csv_files, 1):
                print(f"{i}. {csv_file.name}")

            while True:
                try:
                    csv_choice = input(
                        f"Select CSV file (1-{len(csv_files)}): "
                    ).strip()
                    csv_idx = int(csv_choice) - 1
                    if 0 <= csv_idx < len(csv_files):
                        selected_csv = csv_files[csv_idx]
                        break
                    else:
                        print(f"Please enter a number between 1 and {len(csv_files)}")
                except ValueError:
                    print("Please enter a valid number")
        else:
            selected_csv = csv_files[0]

        self.csv_path = selected_csv
        self.output_dir = selected_folder

        print(f"\n Selected: {selected_folder.name}/{selected_csv.name}")

        # Load and validate data
        self.load_csv_data()

    def load_csv_data(self) -> None:
        """Load and validate CSV data."""
        assert self.csv_path is not None, "CSV path must be set before loading data"

        try:
            if self.use_pandas:
                import pandas as pd

                print(f"Loading CSV data from: {self.csv_path}")
                self.data = pd.read_csv(self.csv_path)
                self._data_backend = "pandas"
                print(f"Loaded {len(self.data)} data points")
            else:
                print(f"Loading CSV data (csv backend) from: {self.csv_path}")
                self._load_csv_data_csvmodule()
                self._data_backend = "csv"
                print(f"Loaded {self._get_len()} data points")

            # Validate required columns
            required_cols = [
                "Current_X",
                "Current_Y",
                "Current_Yaw",
                "Target_X",
                "Target_Y",
                "Target_Yaw",
                "Command_Vector",
            ]
            if self._data_backend == "pandas" and self.data is not None:
                cols = self.data.columns
            elif self._col_data is not None:
                cols = self._col_data.keys()
            else:
                cols = []
            missing_cols = [col for col in required_cols if col not in cols]
            if missing_cols:
                raise ValueError(f"Missing required columns: {missing_cols}")

            self._detect_timestep()

            print("CSV data validation successful")
            if self.dt is not None:
                print(f"Time range: 0.0s to {self._get_len() * float(self.dt):.1f}s")
                print(f"Detected timestep (dt): {self.dt:.3f}s")
            if self.fps is not None:
                print(f"Calculated frame rate: {self.fps:.1f} FPS")

        except Exception as e:
            print(f"Error loading CSV data: {e}")
            raise

    def _detect_timestep(self) -> None:
        """Auto-detect timestep from data and calculate frame rate."""
        if self._data_backend == "pandas" and self.data is not None:
            cols = self.data.columns
        elif self._col_data is not None:
            cols = self._col_data.keys()
        else:
            cols = []
        if "CONTROL_DT" in cols:
            if self._data_backend == "pandas" and self.data is not None:
                val = self.data["CONTROL_DT"].iloc[0]
            else:
                val = self._col("CONTROL_DT")[0]
            self.dt = float(val)
            print(f" Detected timestep from CONTROL_DT: {self.dt}s")

        elif "Actual_Time_Interval" in cols and self._get_len() > 1:
            # Use average of actual time intervals
            intervals = self._col("Actual_Time_Interval")
            valid_intervals = intervals[intervals > 0]  # Filter out zero values
            if len(valid_intervals) > 0:
                self.dt = float(np.mean(valid_intervals))
                print(f" Detected timestep from Actual_Time_Interval: {self.dt:.3f}s")
            else:
                self.dt = 0.25  # Fallback
                print(f"Using fallback timestep: {self.dt}s")

        elif "Step" in cols and self._get_len() > 1:
            if "Control_Time" in cols:
                control_time_col = self._col("Control_Time")
                total_time = float(control_time_col[-1]) if len(control_time_col) > 0 else 0.0
                total_steps = self._get_len() - 1
                if total_steps > 0:
                    self.dt = total_time / total_steps
                    print(f" Calculated timestep from time: {self.dt:.3f}s")
                else:
                    self.dt = 0.25
                    print(f"Using fallback timestep: {self.dt}s")
            else:
                self.dt = 0.25
                print(f"Using default timestep assumption: {self.dt}s")

        elif "MPC_Start_Time" in cols and self._get_len() > 1:
            try:
                # Calculate average time difference between consecutive rows
                time_diffs = []
                for i in range(1, min(10, self._get_len())):  # Sample first 10 rows
                    curr_time_raw = self._row(i)["MPC_Start_Time"]
                    prev_time_raw = self._row(i - 1)["MPC_Start_Time"]
                    if curr_time_raw is not None and prev_time_raw is not None:
                        curr_time = float(curr_time_raw)
                        prev_time = float(prev_time_raw)
                        time_diff = curr_time - prev_time
                        if 0.01 <= time_diff <= 10.0:  # Reasonable timestep range
                            time_diffs.append(time_diff)

                if time_diffs:
                    self.dt = float(np.mean(time_diffs))
                else:
                    self.dt = 0.25  # Default fallback
            except Exception:
                self.dt = 0.25  # Default fallback
        else:
            # Default timestep
            self.dt = 0.25

        # This ensures each frame represents one timestep
        assert self.dt is not None and self.dt > 0, "dt must be positive"
        self.fps = 1.0 / self.dt

        self.fps = max(1.0, min(self.fps, 60.0))

        ideal_fps = 1.0 / self.dt
        if ideal_fps > 60.0:
            self.speedup_factor = ideal_fps / 60.0
            self.fps = 60.0
            print(
                f"Warning: Ideal fps ({ideal_fps:.1f}) too high, using {self.fps} fps with {self.speedup_factor:.1f}x speedup"
            )
        elif ideal_fps < 1.0:
            self.speedup_factor = 1.0 / ideal_fps
            self.fps = 1.0
            print(
                f"Warning: Ideal fps ({ideal_fps:.1f}) too low, using {self.fps} fps with {self.speedup_factor:.1f}x speedup"
            )

    @staticmethod
    def parse_command_vector(command_str: Any) -> np.ndarray:
        """Parse command vector string to numpy array.

        Args:
            command_str: String representation of command vector (or any type)

        Returns:
            numpy array of thruster commands
        """
        try:
            # Handle None or empty
            if command_str is None or command_str == "":
                return np.zeros(8)

            # Convert to string if not already
            command_str = str(command_str)

            # Remove brackets and split
            command_str = command_str.strip("[]")
            values = [float(x.strip()) for x in command_str.split(",")]
            return np.array(values)
        except Exception:
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
        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.suptitle(self.animation_title, fontsize=16)

        self.ax_main = plt.subplot2grid((1, 3), (0, 0), colspan=2)
        self.ax_main.set_xlim(-3, 3)
        self.ax_main.set_ylim(-3, 3)
        self.ax_main.set_aspect("equal")
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_xlabel("X Position (m)")
        self.ax_main.set_ylabel("Y Position (m)")
        self.ax_main.set_title(self.trajectory_title)

        self.ax_info = plt.subplot2grid((1, 3), (0, 2))
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")
        self.ax_info.set_title("System Info", fontsize=12)

        plt.tight_layout()

    def draw_satellite(
        self, x: float, y: float, yaw: float, active_thrusters: list
    ) -> None:
        """Draw satellite at given position and orientation.

        Args:
            x, y: Satellite position
            yaw: Satellite orientation (radians)
            active_thrusters: List of active thruster IDs
        """
        assert self.ax_main is not None, "ax_main must be initialized"

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
        self.ax_main.plot(
            rotated_body[:, 0],
            rotated_body[:, 1],
            color=self.satellite_color,
            linewidth=3,
            label="Satellite",
        )

        # Fill the satellite body
        self.ax_main.fill(
            rotated_body[:, 0],
            rotated_body[:, 1],
            color=self.satellite_color,
            alpha=0.3,
        )

        # Draw thrusters
        for thruster_id, (tx, ty) in self.thrusters.items():
            # Rotate thruster position
            thruster_pos = np.array([tx, ty]) @ rotation_matrix.T
            thruster_x = x + thruster_pos[0]
            thruster_y = y + thruster_pos[1]

            # Color and size based on activity
            if thruster_id in active_thrusters:
                color = "red"
                size = 80
                marker = "o"
                alpha = 1.0
            else:
                color = "gray"
                size = 40
                marker = "s"
                alpha = 0.5

            self.ax_main.scatter(
                thruster_x,
                thruster_y,
                c=color,
                s=size,
                marker=marker,
                alpha=alpha,
                edgecolors="black",
                linewidth=1,
            )

        # Draw orientation arrow
        arrow_length = self.satellite_size * 0.8
        arrow_end_x = x + arrow_length * cos_yaw
        arrow_end_y = y + arrow_length * sin_yaw
        self.ax_main.arrow(
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
        )

    def draw_target(self, target_x: float, target_y: float, target_yaw: float) -> None:
        """Draw target position and orientation.

        Args:
            target_x, target_y: Target position
            target_yaw: Target orientation (radians)
        """
        assert self.ax_main is not None, "ax_main must be initialized"

        self.ax_main.scatter(
            target_x,
            target_y,
            c=self.target_color,
            s=200,
            marker="x",
            linewidth=4,
            label="Target",
        )

        # Target circle
        circle = Circle(
            (target_x, target_y),
            0.05,
            color=self.target_color,
            fill=False,
            linewidth=2,
            alpha=0.7,
        )
        self.ax_main.add_patch(circle)

        # Target orientation arrow
        arrow_length = self.satellite_size * 0.6
        arrow_end_x = target_x + arrow_length * np.cos(target_yaw)
        arrow_end_y = target_y + arrow_length * np.sin(target_yaw)
        self.ax_main.arrow(
            target_x,
            target_y,
            arrow_end_x - target_x,
            arrow_end_y - target_y,
            head_width=0.06,
            head_length=0.06,
            fc=self.target_color,
            ec=self.target_color,
            alpha=0.8,
            linewidth=2,
        )

    def draw_trajectory(self, trajectory_x: list, trajectory_y: list) -> None:
        """Draw satellite trajectory.

        Args:
            trajectory_x, trajectory_y: Lists of trajectory points
        """
        assert self.ax_main is not None, "ax_main must be initialized"

        if len(trajectory_x) > 1:
            self.ax_main.plot(
                trajectory_x,
                trajectory_y,
                color=self.trajectory_color,
                linewidth=2,
                alpha=0.8,
                linestyle="-",
                label="Trajectory",
            )

    def draw_dxf_shape_overlays(self) -> None:
        """Draw Profile Following mission overlays: the base object shape and the offset path."""
        assert self.ax_main is not None, "ax_main must be initialized"

        try:
            if (
                isinstance(self.dxf_base_shape, (list, tuple))
                and len(self.dxf_base_shape) >= 3
            ):
                bx = [p[0] for p in self.dxf_base_shape] + [self.dxf_base_shape[0][0]]
                by = [p[1] for p in self.dxf_base_shape] + [self.dxf_base_shape[0][1]]
                self.ax_main.plot(
                    bx,
                    by,
                    color="#9b59b6",
                    linewidth=2.5,
                    alpha=0.7,
                    label="Object Shape",
                )
            if (
                isinstance(self.dxf_offset_path, (list, tuple))
                and len(self.dxf_offset_path) >= 2
            ):
                px = [p[0] for p in self.dxf_offset_path]
                py = [p[1] for p in self.dxf_offset_path]
                self.ax_main.plot(
                    px,
                    py,
                    color="#e67e22",
                    linewidth=2,
                    alpha=0.8,
                    linestyle="--",
                    label="Shape Path",
                )
            if isinstance(self.dxf_center, (list, tuple)) and len(self.dxf_center) == 2:
                self.ax_main.scatter(
                    self.dxf_center[0],
                    self.dxf_center[1],
                    c="#2ecc71",
                    s=60,
                    marker="+",
                    linewidth=2,
                    label="Shape Center",
                )
        except Exception:
            pass

    def draw_obstacles(self) -> None:
        """Draw obstacles if they are configured."""
        assert self.ax_main is not None, "ax_main must be initialized"

        if (
            hasattr(SatelliteConfig, "OBSTACLES_ENABLED")
            and SatelliteConfig.OBSTACLES_ENABLED
        ):
            obstacles = SatelliteConfig.get_obstacles()
            for i, (obs_x, obs_y, obs_radius) in enumerate(obstacles, 1):
                # Draw obstacle as red filled circle
                obstacle_circle = Circle(
                    (obs_x, obs_y),
                    obs_radius,
                    fill=True,
                    color="red",
                    alpha=0.6,
                    edgecolor="darkred",
                    linewidth=2,
                    zorder=15,
                    label="Obstacle" if i == 1 else "",
                )
                self.ax_main.add_patch(obstacle_circle)

                # Add obstacle label
                self.ax_main.text(
                    obs_x,
                    obs_y,
                    f"O{i}",
                    fontsize=8,
                    color="white",
                    ha="center",
                    va="center",
                    fontweight="bold",
                    zorder=16,
                )

    def update_info_panel(self, step: int, current_data: Any) -> None:
        """Update information panel with current data.

        Args:
            step: Current step
            current_data: Current row of data
        """
        assert self.ax_info is not None, "ax_info must be initialized"
        assert self.dt is not None, "dt must be set"

        self.ax_info.clear()
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")
        self.ax_info.set_title("System Info", fontsize=12, weight="bold")

        # Parse command vector
        command_vector = self.parse_command_vector(current_data["Command_Vector"])
        active_thrusters = self.get_active_thrusters(command_vector)

        # Calculate errors
        pos_error = np.sqrt(current_data["Error_X"] ** 2 + current_data["Error_Y"] ** 2)
        angle_error = abs(np.degrees(current_data["Error_Yaw"]))

        mpc_time = current_data.get("MPC_Computation_Time", 0)
        mpc_status = current_data.get("MPC_Status", "Unknown")
        mpc_solver = current_data.get("MPC_Solver", "")
        mpc_limit = current_data.get("MPC_Solver_Time_Limit", None)
        mpc_exceeded = current_data.get("MPC_Time_Limit_Exceeded", False)
        mpc_fallback = current_data.get("MPC_Fallback_Used", False)

        # Information text
        info_text = [
            f"{self.system_title.upper()}",
            f"{'=' * 25}",
            f"Step: {step}/{self._get_len() - 1}",
            f"Time: {step * self.dt:.1f}s",
            "",
            "CURRENT STATE:",
            f"X: {current_data['Current_X']:.3f} m",
            f"Y: {current_data['Current_Y']:.3f} m",
            f"Yaw: {np.degrees(current_data['Current_Yaw']):.1f}°",
            "",
            "TARGET STATE:",
            f"X: {current_data['Target_X']:.3f} m",
            f"Y: {current_data['Target_Y']:.3f} m",
            f"Yaw: {np.degrees(current_data['Target_Yaw']):.1f}°",
            "",
            "CONTROL ERRORS:",
            f"Position: {pos_error:.3f} m",
            f"Angle: {angle_error:.1f}°",
            "",
            "MPC CONTROLLER:",
            f"Status: {mpc_status}",
            f"Solver: {mpc_solver if isinstance(mpc_solver, str) else ''}",
            f"Solve Time: {mpc_time:.3f}s"
            + (
                f" / Limit: {mpc_limit:.3f}s"
                if isinstance(mpc_limit, (int, float))
                else ""
            ),
            ("Time Limit Exceeded: YES" if bool(mpc_exceeded) else ""),
            ("Fallback Used: YES" if bool(mpc_fallback) else ""),
            "",
            "THRUSTER CONTROL:",
            f"Active: {active_thrusters}",
            f"Count: {len(active_thrusters)}/8",
        ]

        # Display text
        y_pos = 0.95
        for line in info_text:
            if line.startswith("="):
                weight = "normal"
                size = 9
            elif line.isupper() and line.endswith(":"):
                weight = "bold"
                size = 10
            elif line.startswith((self.system_title.upper(), "Step:", "Time:")):
                weight = "bold"
                size = 11
            else:
                weight = "normal"
                size = 9

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
        """Animation update function for each frame.

        Args:
            frame: Frame number
        """
        assert self.ax_main is not None, "ax_main must be initialized"
        assert self.dt is not None, "dt must be set"

        # Clear main plot
        self.ax_main.clear()
        self.ax_main.set_xlim(-3, 3)
        self.ax_main.set_ylim(-3, 3)
        self.ax_main.set_aspect("equal")
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_xlabel("X Position (m)")
        self.ax_main.set_ylabel("Y Position (m)")
        self.ax_main.set_title(self.frame_title_template.format(frame))

        # Get current data
        step = min(int(frame * self.speedup_factor), self._get_len() - 1)
        current_data = self._row(step)

        # Parse command vector and get active thrusters
        command_vector = self.parse_command_vector(current_data["Command_Vector"])
        active_thrusters = self.get_active_thrusters(command_vector)

        # Type-safe extraction with float conversion
        target_x_val = current_data["Target_X"]
        target_x = float(target_x_val) if target_x_val is not None else 0.0
        target_y_val = current_data["Target_Y"]
        target_y = float(target_y_val) if target_y_val is not None else 0.0
        target_yaw_val = current_data["Target_Yaw"]
        target_yaw = float(target_yaw_val) if target_yaw_val is not None else 0.0

        self.draw_target(target_x, target_y, target_yaw)

        # Draw DXF shape if mode is active OR if overlay_dxf option is enabled
        if getattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE", False) or self.overlay_dxf:
            self.draw_dxf_shape_overlays()

        # Draw trajectory up to current point
        trajectory_x = self._col("Current_X")[: step + 1].tolist()
        trajectory_y = self._col("Current_Y")[: step + 1].tolist()
        self.draw_trajectory(trajectory_x, trajectory_y)

        # Draw satellite with type-safe extraction
        current_x_val = current_data["Current_X"]
        current_x = float(current_x_val) if current_x_val is not None else 0.0
        current_y_val = current_data["Current_Y"]
        current_y = float(current_y_val) if current_y_val is not None else 0.0
        current_yaw_val = current_data["Current_Yaw"]
        current_yaw = float(current_yaw_val) if current_yaw_val is not None else 0.0

        self.draw_satellite(
            current_x,
            current_y,
            current_yaw,
            active_thrusters,
        )

        # Draw obstacles
        self.draw_obstacles()

        # Add legend
        self.ax_main.legend(loc="upper right", fontsize=9)

        # Update info panel
        self.update_info_panel(step, current_data)

        return []

    def generate_animation(self, output_filename: Optional[str] = None) -> None:
        """Generate and save the MP4 animation.

        Args:
            output_filename: Name of output MP4 file (optional)
        """
        assert (
            self.fps is not None
        ), "FPS must be calculated before generating animation"
        assert self.dt is not None, "dt must be set before generating animation"
        assert self.output_dir is not None, "Output directory must be set"

        if output_filename is None:
            output_filename = f"{self.plot_prefix}_animation.mp4"

        print(f"\n{'=' * 60}")
        print(f"GENERATING {self.system_name.upper()} ANIMATION")
        print(f"{'=' * 60}")
        print("Setting up animation...")
        self.setup_plot()

        # Verify figure was created successfully
        assert self.fig is not None, "Figure must be set up before generating animation"

        # Calculate number of frames
        total_frames = self._get_len() // int(self.speedup_factor)
        if total_frames == 0:
            total_frames = 1

        print("Animation parameters:")
        print(f"  - Total data points: {self._get_len()}")
        print(f"  - Animation frames: {total_frames}")
        print(f"  - Frame rate: {self.fps} FPS")
        print(f"  - Speedup factor: {self.speedup_factor}x")
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
            # Prefer explicit ffmpeg writer config with safe args (no audio) to avoid stream index issues
            Writer = writers["ffmpeg"]
            # Note: extra_args parameter name may vary by matplotlib version
            # writer_kwargs = {  # Defined but not currently used
            #     "fps": self.fps,
            #     "metadata": dict(artist=f"{self.system_name} Visualizer"),
            #     "bitrate": 2000,
            #     "codec": "libx264",
            # }
            try:
                writer = Writer(
                    fps=int(self.fps),
                    metadata=dict(artist=f"{self.system_name} Visualizer"),
                    bitrate=2000,
                    codec="libx264",
                    extra_args=[  # type: ignore
                        "-pix_fmt",
                        "yuv420p",
                        "-movflags",
                        "+faststart",
                        "-an",
                    ],
                )
            except TypeError:
                writer = Writer(
                    fps=int(self.fps),
                    metadata=dict(artist=f"{self.system_name} Visualizer"),
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
            # Graceful fallbacks to avoid aborting the entire run on ffmpeg issues
            print(f" Error saving MP4 with ffmpeg: {e}")
            print("↪  Attempting WebM fallback (VP9)...")
            try:
                Writer = writers["ffmpeg"]
                webm_path = output_path.with_suffix(".webm")
                # webm_kwargs = {  # Defined but not currently used
                #     "fps": self.fps,
                #     "metadata": dict(artist=f"{self.system_name} Visualizer"),
                #     "bitrate": 2000,
                #     "codec": "libvpx-vp9",
                # }
                try:
                    webm_writer = Writer(
                        fps=int(self.fps),
                        metadata=dict(artist=f"{self.system_name} Visualizer"),
                        bitrate=2000,
                        codec="libvpx-vp9",
                        extra_args=[  # type: ignore
                            "-pix_fmt",
                            "yuv420p",
                            "-b:v",
                            "1M",
                            "-an",
                        ],
                    )
                except TypeError:
                    webm_writer = Writer(
                        fps=int(self.fps),
                        metadata=dict(artist=f"{self.system_name} Visualizer"),
                        bitrate=2000,
                        codec="libvpx-vp9",
                    )
                ani.save(
                    str(webm_path),
                    writer=webm_writer,
                    progress_callback=self._progress_callback,
                )
                print("\n WebM animation saved successfully!")
                print(f" File location: {webm_path}")
            except Exception as e2:
                print(f" WebM fallback failed: {e2}")
                print("↪  Attempting GIF fallback (Pillow)...")
                try:
                    from matplotlib.animation import PillowWriter

                    gif_path = output_path.with_suffix(".gi")
                    gif_writer = PillowWriter(fps=max(1, int(self.fps)))
                    ani.save(
                        str(gif_path),
                        writer=gif_writer,
                        progress_callback=self._progress_callback,
                    )
                    print("\n GIF animation saved successfully!")
                    print(f" File location: {gif_path}")
                except Exception as e3:
                    print(f" GIF fallback failed: {e3}")
                    print(
                        "  Skipping animation export. You can still find performance plots in the Plots folder."
                    )
        finally:
            plt.close(self.fig)

    def _progress_callback(self, current_frame: int, total_frames: int) -> None:
        """Progress callback for animation saving with visual progress bar.

        Args:
            current_frame: Current frame being processed
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

    def generate_performance_plots(self) -> None:
        """Generate performance analysis plots."""
        assert self.dt is not None, "dt must be set before generating plots"
        assert (
            self.output_dir is not None
        ), "output_dir must be set before generating plots"

        print("Generating performance analysis plots...")

        # Create Plots subfolder
        plots_dir = self.output_dir / "Plots"
        plots_dir.mkdir(exist_ok=True)
        print(f" Created Plots directory: {plots_dir}")

        # Generate specific performance plots
        self._generate_position_tracking_plot(plots_dir)
        self._generate_position_error_plot(plots_dir)
        self._generate_angular_tracking_plot(plots_dir)
        self._generate_angular_error_plot(plots_dir)
        self._generate_trajectory_plot(plots_dir)
        self._generate_thruster_usage_plot(plots_dir)
        self._generate_control_effort_plot(plots_dir)
        self._generate_velocity_magnitude_plot(plots_dir)
        self._generate_mpc_performance_plot(plots_dir)
        self._generate_timing_intervals_plot(plots_dir)

        print(f"Performance plots saved to: {plots_dir}")

    def _generate_position_tracking_plot(self, plot_dir: Path) -> None:
        """Generate position tracking over time plot."""
        assert self.dt is not None, "dt must be set"

        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle(f"Position Tracking - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        # X position tracking
        axes[0].plot(time, self._col("Current_X"), "b-", linewidth=2, label="Current X")
        axes[0].plot(time, self._col("Target_X"), "r--", linewidth=2, label="Target X")
        axes[0].set_ylabel("X Position (m)")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[0].set_title("X Position Tracking")

        # Y position tracking
        axes[1].plot(time, self._col("Current_Y"), "b-", linewidth=2, label="Current Y")
        axes[1].plot(time, self._col("Target_Y"), "r--", linewidth=2, label="Target Y")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Y Position (m)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        axes[1].set_title("Y Position Tracking")

        plt.tight_layout()
        plt.savefig(plot_dir / "position_tracking.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_position_error_plot(self, plot_dir: Path) -> None:
        """Generate position error plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        time = np.arange(self._get_len()) * float(self.dt)
        pos_error = np.sqrt(self._col("Error_X") ** 2 + self._col("Error_Y") ** 2)

        ax.plot(time, pos_error, "b-", linewidth=3, label="Position Error")
        ax.axhline(
            y=0.1,
            color="r",
            linestyle="--",
            linewidth=2,
            alpha=0.8,
            label="Target Threshold (0.1m)",
        )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Position Error (meters)", fontsize=14)
        ax.set_title(f"Position Error Over Time - {self.system_title}")
        ax.grid(True, alpha=0.4, linewidth=1)
        ax.legend(fontsize=12)

        # Add final error as text
        final_error = pos_error[-1] if len(pos_error) > 0 else 0.0
        ax.text(
            0.02,
            0.98,
            f"Final Error: {final_error:.3f}m",
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "position_error.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_angular_tracking_plot(self, plot_dir: Path) -> None:
        """Generate angular tracking plot (0–360° visualization).
        This only changes the plot mapping; all internal code still uses -180..180 conventions.
        """
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        time = np.arange(self._get_len()) * float(self.dt)

        def to_0_360(rad_series):
            deg = np.degrees(np.array(rad_series))
            return np.mod(deg, 360.0)

        cur_deg = to_0_360(self._col("Current_Yaw"))
        tgt_deg = to_0_360(self._col("Target_Yaw"))

        ax.plot(time, cur_deg, "b-", linewidth=2, label="Current Yaw")
        ax.plot(time, tgt_deg, "r--", linewidth=2, label="Target Yaw")

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Yaw Angle (degrees, 0–360°)", fontsize=14)
        ax.set_title(f"Angular Tracking - {self.system_title}")
        # Y-axis: 0 to 360 with ticks every 20 degrees
        ax.set_ylim(0, 360)
        ax.set_yticks(np.arange(0, 361, 20))
        ax.grid(True, alpha=0.4, linewidth=1)
        ax.legend(fontsize=12)

        plt.tight_layout()
        plt.savefig(plot_dir / "angular_tracking.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate angular error plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        time = np.arange(self._get_len()) * float(self.dt)
        angle_error = np.abs(np.degrees(self._col("Error_Yaw")))

        ax.plot(time, angle_error, "g-", linewidth=3, label="Angular Error")
        ax.axhline(
            y=5,
            color="r",
            linestyle="--",
            linewidth=2,
            alpha=0.8,
            label="Target Threshold (5°)",
        )

        ax.set_xlabel("Time (seconds)", fontsize=14)
        ax.set_ylabel("Angular Error (degrees)", fontsize=14)
        ax.set_title(f"Angular Error Over Time - {self.system_title}")
        ax.grid(True, alpha=0.4, linewidth=1)
        ax.legend(fontsize=12)

        # Add final error as text
        final_error = angle_error[-1] if len(angle_error) > 0 else 0.0
        ax.text(
            0.02,
            0.98,
            f"Final Error: {final_error:.1f}°",
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "angular_error.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_trajectory_plot(self, plot_dir: Path) -> None:
        """Generate trajectory plot."""
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        x_pos = self._col("Current_X")
        y_pos = self._col("Current_Y")
        # Handle target columns possibly being arrays/series
        target_x_col = self._col("Target_X")
        target_y_col = self._col("Target_Y")
        target_x = target_x_col[0] if len(target_x_col) > 0 else 0.0
        target_y = target_y_col[0] if len(target_y_col) > 0 else 0.0

        # Plot trajectory
        ax.plot(x_pos, y_pos, "b-", linewidth=3, alpha=0.8, label="Satellite Path")
        if len(x_pos) > 0:
            ax.plot(x_pos[0], y_pos[0], "go", markersize=12, label="Start Position")
            ax.plot(x_pos[-1], y_pos[-1], "ro", markersize=12, label="Final Position")
        ax.plot(target_x, target_y, "r*", markersize=20, label="Target")

        try:
            if (
                isinstance(self.dxf_base_shape, (list, tuple))
                and len(self.dxf_base_shape) >= 3
            ):
                bx = [p[0] for p in self.dxf_base_shape] + [self.dxf_base_shape[0][0]]
                by = [p[1] for p in self.dxf_base_shape] + [self.dxf_base_shape[0][1]]
                ax.plot(
                    bx,
                    by,
                    color="#9b59b6",
                    linewidth=2.5,
                    alpha=0.7,
                    label="Object Shape",
                )
            if (
                isinstance(self.dxf_offset_path, (list, tuple))
                and len(self.dxf_offset_path) >= 2
            ):
                px = [p[0] for p in self.dxf_offset_path]
                py = [p[1] for p in self.dxf_offset_path]
                ax.plot(
                    px,
                    py,
                    color="#e67e22",
                    linewidth=2,
                    alpha=0.8,
                    linestyle="--",
                    label="Shape Path",
                )
            if isinstance(self.dxf_center, (list, tuple)) and len(self.dxf_center) == 2:
                ax.scatter(
                    self.dxf_center[0],
                    self.dxf_center[1],
                    c="#2ecc71",
                    s=60,
                    marker="+",
                    linewidth=2,
                    label="Shape Center",
                )
        except Exception:
            pass

        # Add target circle
        circle = Circle(
            (target_x, target_y),
            0.1,
            color="red",
            fill=False,  # type: ignore[arg-type]
            linewidth=2,
            linestyle="--",
            alpha=0.7,
            label="Target Zone (±0.1m)",
        )
        ax.add_patch(circle)

        # Set fixed axis limits
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_xlabel("X Position (meters)", fontsize=14)
        ax.set_ylabel("Y Position (meters)", fontsize=14)
        ax.set_title(f"Satellite Trajectory - {self.system_title}")
        ax.grid(True, alpha=0.4, linewidth=1)
        ax.legend(fontsize=12)
        ax.set_aspect("equal")

        # Add distance info
        if len(x_pos) > 0:
            final_distance = np.sqrt(
                (x_pos[-1] - target_x) ** 2 + (y_pos[-1] - target_y) ** 2
            )
        else:
            final_distance = 0.0
        ax.text(
            0.02,
            0.98,
            f"Final Distance to Target: {final_distance:.3f}m",
            transform=ax.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "trajectory.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_thruster_usage_plot(self, plot_dir: Path) -> None:
        """Generate thruster usage plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        # Parse command vectors
        command_data = []
        for idx in range(self._get_len()):
            row = self._row(idx)
            cmd_vec = self.parse_command_vector(row["Command_Vector"])
            command_data.append(cmd_vec)
        command_matrix = np.array(command_data)

        thruster_ids = np.arange(1, 9)  # Thrusters 1-8
        total_activation_time = np.sum(command_matrix, axis=0) * float(
            self.dt
        )  # Convert to time units

        # Create bar plot
        bars = ax.bar(
            thruster_ids,
            total_activation_time,
            color="steelblue",
            alpha=0.7,
            edgecolor="darkblue",
            linewidth=1.5,
        )

        # Add value labels on top of bars
        for _i, bar in enumerate(bars):
            height = bar.get_height()
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                height + 0.01,
                f"{height:.2f}s",
                ha="center",
                va="bottom",
            )

        ax.set_xlabel("Thruster ID", fontsize=14)
        ax.set_ylabel("Total Active Time (seconds)", fontsize=14)
        ax.set_title(f"Thruster Usage Summary - {self.system_title}")
        ax.grid(True, alpha=0.4, linewidth=1)
        ax.set_xticks(thruster_ids)

        total_thruster_seconds = float(np.sum(total_activation_time))
        ax.text(
            0.98,
            0.95,
            f"Total active time (sum of thrusters): {total_thruster_seconds:.2f}s",
            transform=ax.transAxes,
            ha="right",
            va="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
        )

        plt.tight_layout()
        plt.savefig(plot_dir / "thruster_usage.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_control_effort_plot(self, plot_dir: Path) -> None:
        """Generate control effort plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        time = np.arange(self._get_len()) * float(self.dt)

        # Parse command vectors
        command_data = []
        for idx in range(self._get_len()):
            row = self._row(idx)
            cmd_vec = self.parse_command_vector(row["Command_Vector"])
            command_data.append(cmd_vec)
        command_matrix = np.array(command_data)

        total_effort_per_step = np.sum(command_matrix, axis=1)
        ax.plot(time, total_effort_per_step, "c-", linewidth=2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Total Control Effort")
        ax.set_title(f"Control Effort Over Time - {self.system_title}")
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(plot_dir / "control_effort.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_velocity_magnitude_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude over time plot (speed vs time)."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        n = self._get_len()
        if n < 2:
            ax.text(
                0.5,
                0.5,
                "Not enough data to compute velocity",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=16,
            )
            ax.set_title(f"Velocity Magnitude - {self.system_title}")
            plt.tight_layout()
            plt.savefig(
                plot_dir / "velocity_magnitude.png", dpi=300, bbox_inches="tight"
            )
            plt.close()
            return

        dt_float = float(self.dt)
        time = np.arange(n) * dt_float
        x = self._col("Current_X")
        y = self._col("Current_Y")

        vx = np.gradient(x, dt_float)
        vy = np.gradient(y, dt_float)
        speed = np.sqrt(vx**2 + vy**2)

        ax.plot(time, speed, color="teal", linewidth=2.5, label="Speed (|v|)")
        ax.axhline(
            y=float(np.mean(speed)),
            color="gray",
            linestyle="--",
            alpha=0.7,
            label=f"Mean: {np.mean(speed):.3f} m/s",
        )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Speed (m/s)")
        ax.set_title(f"Velocity Magnitude Over Time - {self.system_title}")
        ax.grid(True, alpha=0.3)
        ax.legend()

        plt.tight_layout()
        plt.savefig(plot_dir / "velocity_magnitude.png", dpi=300, bbox_inches="tight")
        plt.close()

    def _generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        if self._data_backend == "pandas" and self.data is not None:
            cols = self.data.columns
        elif self._col_data is not None:
            cols = list(self._col_data.keys())
        else:
            cols = []

        if "MPC_Computation_Time" in cols:
            time = np.arange(self._get_len()) * float(self.dt)

            # Ensure we have numeric data - convert and handle non-numeric values
            raw_comp_times = self._col("MPC_Computation_Time")
            comp_times = []
            for val in raw_comp_times:
                try:
                    comp_times.append(float(val) * 1000)  # Convert to ms
                except (ValueError, TypeError):
                    comp_times.append(0.0)  # Default for invalid values
            comp_times = np.array(comp_times)

            # Optional: solver time limit per step
            limit_ms = None
            if "MPC_Solver_Time_Limit" in cols:
                raw_limits = self._col("MPC_Solver_Time_Limit")
                limits = []
                for val in raw_limits:
                    try:
                        limits.append(float(val) * 1000)
                    except (ValueError, TypeError):
                        limits.append(0.0)
                limit_ms = np.array(limits)

            ax.plot(time, comp_times, "purple", linewidth=2, label="Computation Time")
            mean_ms = float(np.mean(comp_times))
            max_ms = float(np.max(comp_times))
            ax.axhline(
                y=mean_ms,
                color="r",
                linestyle="--",
                alpha=0.7,
                label=f"Mean: {mean_ms:.1f} ms",
            )
            ax.axhline(
                y=max_ms,
                color="g",
                linestyle=":",
                alpha=0.7,
                label=f"Max: {max_ms:.1f} ms",
            )
            if limit_ms is not None and np.any(limit_ms > 0):
                # If limit varies, plot as line; else constant hline
                if len(np.unique(limit_ms)) > 1:
                    ax.plot(
                        time,
                        limit_ms,
                        color="black",
                        linestyle=":",
                        alpha=0.6,
                        label="Time Limit",
                    )
                else:
                    limit_val = float(
                        limit_ms[0] if isinstance(limit_ms, np.ndarray) else limit_ms
                    )
                    if limit_val > 0:
                        ax.axhline(
                            y=limit_val,
                            color="black",
                            linestyle=":",
                            alpha=0.6,
                            label="Time Limit",
                        )
                # Highlight exceedances
                if "MPC_Time_Limit_Exceeded" in cols:
                    exceeded_vals = self._col("MPC_Time_Limit_Exceeded")
                    exceeded_idx = []
                    for i, val in enumerate(exceeded_vals):
                        try:
                            if bool(val) or (
                                isinstance(val, str) and val.lower() == "true"
                            ):
                                exceeded_idx.append(i)
                        except Exception:
                            pass
                    exceeded_idx = np.array(exceeded_idx)
                    if len(exceeded_idx) > 0:
                        ax.scatter(
                            time[exceeded_idx],
                            comp_times[exceeded_idx],
                            color="red",
                            s=25,
                            label="Exceeded",
                            zorder=5,
                        )
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Computation Time (ms)")
            ax.set_title(f"MPC Computation Time - {self.system_title}")
            ax.grid(True, alpha=0.3)
            ax.legend()
        else:
            ax.text(
                0.5,
                0.5,
                "MPC Computation Time\nData Not Available",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=16,
            )
            ax.set_title(f"MPC Computation Time - {self.system_title}")

        plt.tight_layout()
        plt.savefig(plot_dir / "mpc_performance.png", dpi=300, bbox_inches="tight")
        plt.close()

    # --- CSV backend helpers ---
    def _load_csv_data_csvmodule(self) -> None:
        """Load CSV data using csv module backend."""
        assert self.csv_path is not None, "CSV path must be set"

        with open(self.csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            rows = []
            for r in reader:
                rows.append(r)
        # Build column-wise data with type conversion
        cols = reader.fieldnames or []
        col_data = {c: [] for c in cols}
        for r in rows:
            for c in cols:
                v = r.get(c, "")
                if v is None:
                    col_data[c].append("")
                    continue
                # Convert booleans and numbers where applicable
                lv = v.strip()
                if lv.lower() in ("true", "false"):
                    col_data[c].append(lv.lower() == "true")
                else:
                    try:
                        col_data[c].append(float(lv))
                    except Exception:
                        col_data[c].append(v)
        # Store
        self._rows = rows
        self._col_data = {k: np.array(v) for k, v in col_data.items()}
        self.data = self  # allow attribute access in unchanged code paths

    def _get_len(self) -> int:
        """Get length of data."""
        if self._data_backend == "pandas":
            return len(self.data) if self.data is not None else 0
        return len(self._rows) if self._rows is not None else 0

    def _col(self, name: str) -> np.ndarray:
        """Get column data."""
        if self._data_backend == "pandas" and self.data is not None:
            return (
                self.data[name].values
                if hasattr(self.data[name], "values")
                else np.array(self.data[name])
            )
        return self._col_data.get(name, np.array([])) if self._col_data is not None else np.array([])  # type: ignore[return-value]

    def _row(self, idx: int) -> Dict[str, Any]:
        """Get row data."""
        if self._data_backend == "pandas" and self.data is not None:
            return self.data.iloc[idx]
        # Build a dict using typed column arrays so consumers see floats/bools
        if self._col_data is not None:
            return {
                k: (self._col_data[k][idx] if k in self._col_data else None)
                for k in self._col_data.keys()
            }
        return {}

    def _generate_timing_intervals_plot(self, plot_dir: Path) -> None:
        """Generate timing intervals plot."""
        assert self.dt is not None, "dt must be set"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))

        if self._data_backend == "pandas" and self.data is not None:
            cols = self.data.columns
        elif self._col_data is not None:
            cols = self._col_data.keys()
        else:
            cols = []

        if "Actual_Time_Interval" in cols:
            time = np.arange(self._get_len()) * float(self.dt)
            intervals = self._col("Actual_Time_Interval")

            ax.plot(time, intervals, "orange", linewidth=2, label="Actual Intervals")
            ax.axhline(
                y=self.dt,
                color="r",
                linestyle="--",
                alpha=0.7,
                label=f"Target: {self.dt:.3f}s",
            )
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Time Interval (s)")
            ax.set_title(f"Timing Intervals - {self.system_title}")
            ax.grid(True, alpha=0.3)
            ax.legend()
        else:
            ax.text(
                0.5,
                0.5,
                "Timing Interval\nData Not Available",
                ha="center",
                va="center",
                transform=ax.transAxes,
                fontsize=16,
            )
            ax.set_title(f"Timing Intervals - {self.system_title}")

        plt.tight_layout()
        plt.savefig(plot_dir / "timing_intervals.png", dpi=300, bbox_inches="tight")
        plt.close()


class LinearizedVisualizationGenerator(UnifiedVisualizationGenerator):
    """Legacy compatibility class for simulation visualization."""

    def __init__(self, data_directory: str = "Data/Simulation"):
        super().__init__(data_directory, mode="simulation")


class RealLinearizedVisualizationGenerator(UnifiedVisualizationGenerator):
    """Legacy compatibility class for real test visualization."""

    def __init__(
        self, data_directory: str = "Data/Real_Test", interactive: bool = False
    ):
        super().__init__(data_directory, mode="real", interactive=interactive)


def get_demo_shape(shape_type: str) -> List[tuple]:
    """Get predefined demo shape points (matches Mission.py implementation)."""
    if shape_type == "rectangle":
        # 0.4m x 0.3m rectangle centered at origin
        return [
            (-0.2, -0.15),
            (0.2, -0.15),
            (0.2, 0.15),
            (-0.2, 0.15),
            (-0.2, -0.15),  # Close the shape
        ]
    elif shape_type == "triangle":
        # Equilateral triangle with 0.4m sides
        return [(0.0, 0.2), (-0.173, -0.1), (0.173, -0.1), (0.0, 0.2)]
    elif shape_type == "hexagon":
        # Regular hexagon with 0.2m radius
        points = []
        for i in range(7):  # 7 points to close the shape
            angle = i * np.pi / 3
            x = 0.2 * np.cos(angle)
            y = 0.2 * np.sin(angle)
            points.append((x, y))
        return points
    else:
        # Default to rectangle
        return get_demo_shape("rectangle")


def transform_shape(points: List[tuple], center: tuple, rotation: float) -> List[tuple]:
    """Transform shape points to specified center and rotation."""
    transformed = []
    cos_r = np.cos(rotation)
    sin_r = np.sin(rotation)

    for x, y in points:
        # Rotate
        x_rot = x * cos_r - y * sin_r
        y_rot = x * sin_r + y * cos_r

        # Translate
        x_final = x_rot + center[0]
        y_final = y_rot + center[1]

        transformed.append((x_final, y_final))

    return transformed


def make_offset_path(points: List[tuple], offset_distance: float) -> List[tuple]:
    """Create an outward offset path (matches Mission.py implementation)."""
    if len(points) < 3:
        return points

    # Try to use DXF_Viewer's make_offset_path if available
    try:
        from DXF.dxf_viewer import (
            make_offset_path as viewer_make_offset_path,  # type: ignore[import-not-found]
        )

        return viewer_make_offset_path(
            points,
            float(offset_distance),
            1.0,
            join="round",
            resolution=24,
            mode="buffer",
        )
    except Exception:
        pass

    # Fallback: simple centroid-based offset
    pts = points[:]
    if np.linalg.norm(np.array(pts[0]) - np.array(pts[-1])) > 1e-8:
        pts = pts + [pts[0]]

    # Calculate shape centroid
    centroid_x = np.mean([p[0] for p in pts])
    centroid_y = np.mean([p[1] for p in pts])
    centroid = np.array([centroid_x, centroid_y])

    upscaled = []
    for i in range(len(pts) - 1):
        current = np.array(pts[i])
        next_point = np.array(pts[i + 1])

        edge_vec = next_point - current
        edge_normal = np.array([-edge_vec[1], edge_vec[0]])
        if np.linalg.norm(edge_normal) > 0:
            edge_normal = edge_normal / np.linalg.norm(edge_normal)

        mid_point = (current + next_point) / 2
        to_mid = mid_point - centroid
        if np.dot(edge_normal, to_mid) < 0:
            edge_normal = -edge_normal

        offset_current = current + edge_normal * offset_distance
        upscaled.append(tuple(offset_current))

    if np.linalg.norm(np.array(pts[0]) - np.array(pts[-1])) < 1e-6:
        upscaled.append(upscaled[0])

    return upscaled


def load_dxf_shape(dxf_path: str) -> List[tuple]:
    """Load shape points from DXF file using DXF_Viewer pipeline (matches Mission.py)."""
    import ezdxf

    try:
        from DXF.dxf_viewer import (  # type: ignore[import-not-found]
            extract_boundary_polygon,
            sanitize_boundary,
            units_code_to_name_and_scale,
        )
    except Exception as e:
        raise ImportError(f"dxf_viewer utilities unavailable: {e}")

    # Read DXF and determine units
    doc = ezdxf.readfile(dxf_path)  # type: ignore[attr-defined]
    msp = doc.modelspace()
    insunits = int(doc.header.get("$INSUNITS", 0))
    units_name, to_m = units_code_to_name_and_scale(insunits)

    # Extract and sanitize boundary in native units, then scale to meters
    boundary = extract_boundary_polygon(msp)
    boundary = sanitize_boundary(boundary, to_m)
    boundary_m = (
        [(float(x) * to_m, float(y) * to_m) for (x, y) in boundary] if boundary else []
    )

    if not boundary_m:
        raise ValueError("No usable DXF boundary could be constructed.")

    print(f" DXF Loaded: {len(boundary_m)} points")
    print(f"   Units: {units_name} (INSUNITS={insunits}), scaled → meters (x{to_m})")
    return boundary_m


def configure_dxf_overlay_interactive() -> bool:
    """Interactive configuration of DXF shape overlay.

    Prompts user for shape selection, center, rotation, and offset,
    then sets the configuration in SatelliteConfig.

    Returns:
        True if overlay was configured, False if user cancelled
    """
    print("\n" + "=" * 60)
    print("   DXF SHAPE OVERLAY CONFIGURATION")
    print("=" * 60)

    # Ask if user wants overlay
    print("\nAdd DXF shape overlay to animation?")
    response = (
        input("Enter 'yes' or 'y' to configure overlay (or press Enter to skip): ")
        .strip()
        .lower()
    )
    if response not in ["yes", "y"]:
        print("Skipping DXF overlay.")
        return False

    # Shape selection
    print("\nShape configuration:")
    shape_points = None

    try:
        import ezdxf  # noqa: F401

        dxf_available = True
    except ImportError:
        dxf_available = False
        print("  ezdxf library not installed. Using demo shapes only.")

    if dxf_available:
        print("\nShape source:")
        print("1. Load from DXF file")
        print("2. Use demo rectangle")
        print("3. Use demo triangle")
        print("4. Use demo hexagon")
        choice = input("Select option (1-4): ").strip()

        if choice == "1":
            dxf_path = input("Enter DXF file path: ").strip()
            try:
                shape_points = load_dxf_shape(dxf_path)
                print(f" Loaded shape with {len(shape_points)} points from DXF")
            except Exception as e:
                print(f" Failed to load DXF: {e}")
                print("Falling back to demo rectangle")
                shape_points = get_demo_shape("rectangle")
        elif choice == "2":
            shape_points = get_demo_shape("rectangle")
        elif choice == "3":
            shape_points = get_demo_shape("triangle")
        elif choice == "4":
            shape_points = get_demo_shape("hexagon")
        else:
            print("Invalid choice. Using demo rectangle.")
            shape_points = get_demo_shape("rectangle")
    else:
        print("\nDemo shapes available:")
        print("1. Rectangle")
        print("2. Triangle")
        print("3. Hexagon")
        choice = input("Select demo shape (1-3): ").strip()

        if choice == "1":
            shape_points = get_demo_shape("rectangle")
        elif choice == "2":
            shape_points = get_demo_shape("triangle")
        elif choice == "3":
            shape_points = get_demo_shape("hexagon")
        else:
            print("Invalid choice. Using rectangle.")
            shape_points = get_demo_shape("rectangle")

    # Get shape center
    try:
        center_x = float(input("Shape center X position (meters): "))
        center_y = float(input("Shape center Y position (meters): "))
        shape_center = (center_x, center_y)
    except ValueError:
        print("Invalid input. Using default center (0.0, 0.0).")
        shape_center = (0.0, 0.0)

    # Get shape rotation
    try:
        shape_rotation_input = input(
            "Shape rotation angle (degrees, default 0): "
        ).strip()
        shape_rotation_deg = (
            float(shape_rotation_input) if shape_rotation_input else 0.0
        )
        shape_rotation_rad = np.radians(shape_rotation_deg)
    except ValueError:
        print("Invalid input. Using default rotation 0°.")
        shape_rotation_deg = 0.0
        shape_rotation_rad = 0.0

    print(f"Shape center: ({shape_center[0]:.2f}, {shape_center[1]:.2f}) m")
    print(f"Shape rotation: {shape_rotation_deg:.1f}°")

    # Get offset distance
    try:
        offset_input = input(
            "Offset distance from shape (meters, default 0.5): "
        ).strip()
        offset_distance = float(offset_input) if offset_input else 0.5
        if offset_distance < 0.1:
            print("Minimum offset 0.1m. Using 0.1m.")
            offset_distance = 0.1
        elif offset_distance > 2.0:
            print("Maximum offset 2.0m. Using 2.0m.")
            offset_distance = 2.0
    except ValueError:
        print("Invalid input. Using default 0.5m offset.")
        offset_distance = 0.5

    print(f"Offset distance: {offset_distance:.2f} m")

    # Transform and compute offset path
    transformed_shape = transform_shape(shape_points, shape_center, shape_rotation_rad)
    offset_path = make_offset_path(transformed_shape, offset_distance)
    print(f" Created offset path with {len(offset_path)} points")

    # Store in SatelliteConfig
    SatelliteConfig.DXF_SHAPE_MODE_ACTIVE = True  # type: ignore[assignment]
    SatelliteConfig.DXF_BASE_SHAPE = transformed_shape
    SatelliteConfig.DXF_SHAPE_PATH = offset_path
    SatelliteConfig.DXF_SHAPE_CENTER = shape_center  # type: ignore[assignment]

    print("\n DXF overlay configured successfully!")
    return True


def select_data_file_interactive() -> tuple:
    """Interactive file browser to select a CSV data file."""
    print("\n" + "=" * 60)
    print("   VISUALIZATION DATA FILE SELECTOR")
    print("=" * 60)

    # Scan for available data directories
    data_root = Path("Data")
    if not data_root.exists():
        print(f" Error: Data directory not found at {data_root.absolute()}")
        return None, None

    # Find all CSV files in Data/Simulation and Data/Real_Test
    sim_csvs = (
        list((data_root / "Simulation").rglob("simulation_data.csv"))
        if (data_root / "Simulation").exists()
        else []
    )
    real_csvs = (
        list((data_root / "Real_Test").rglob("real_test_data.csv"))
        if (data_root / "Real_Test").exists()
        else []
    )

    all_csvs = []

    # Add simulation data
    for csv_path in sorted(sim_csvs, key=lambda p: p.stat().st_mtime, reverse=True):
        timestamp_dir = csv_path.parent.name
        all_csvs.append(("Simulation", timestamp_dir, csv_path))

    # Add real data
    for csv_path in sorted(real_csvs, key=lambda p: p.stat().st_mtime, reverse=True):
        timestamp_dir = csv_path.parent.name
        all_csvs.append(("Real", timestamp_dir, csv_path))

    if not all_csvs:
        print(" No data files found in Data/Simulation or Data/Real")
        return None, None

    # Display available files
    print(f"\nFound {len(all_csvs)} data file(s):\n")
    for idx, (mode, timestamp, csv_path) in enumerate(all_csvs, 1):
        file_size = csv_path.stat().st_size / 1024  # KB
        mod_time = datetime.fromtimestamp(csv_path.stat().st_mtime).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        print(f"{idx:2d}. [{mode:10s}] {timestamp} ({file_size:.1f} KB) - {mod_time}")

    # Get user selection
    print("\n" + "-" * 60)
    while True:
        try:
            choice = input(f"Select file (1-{len(all_csvs)}) or 'q' to quit: ").strip()
            if choice.lower() == "q":
                print("Cancelled.")
                return None, None

            idx = int(choice) - 1
            if 0 <= idx < len(all_csvs):
                mode, timestamp, csv_path = all_csvs[idx]
                print(f"\n Selected: {csv_path}")
                return str(csv_path.parent), mode.lower()
            else:
                print(f"Invalid selection. Please enter 1-{len(all_csvs)}")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None, None


def main() -> int:
    """Main function for standalone usage."""
    import argparse

    parser = argparse.ArgumentParser(description="Generate MPC visualization")
    parser.add_argument(
        "--mode",
        choices=["simulation", "real"],
        default=None,
        help="Visualization mode (simulation or real)",
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        help="Data directory (optional, uses defaults based on mode)",
    )
    parser.add_argument(
        "--interactive", action="store_true", help="Interactive data selection"
    )
    parser.add_argument(
        "--plots-only", action="store_true", help="Generate only plots, skip animation"
    )
    parser.add_argument(
        "--overlay-dx",
        action="store_true",
        help="Overlay DXF shape on trajectory (if available in Config)",
    )

    args = parser.parse_args()

    # If no arguments provided, use interactive mode by default
    if args.data_dir is None and args.mode is None and not args.interactive:
        print("No arguments provided - using interactive file selector")
        args.interactive = True

    # Interactive file selection
    if args.interactive or (args.data_dir is None and args.mode is None):
        data_dir, mode = select_data_file_interactive()
        if data_dir is None:
            return 1
        args.data_dir = data_dir
        if args.mode is None:
            args.mode = mode

        # After file selection, ask about DXF overlay (unless already specified via CLI)
        if not args.overlay_dxf:
            overlay_configured = configure_dxf_overlay_interactive()
            if overlay_configured:
                args.overlay_dxf = True

    # Set default data directory based on mode if still not set
    if args.data_dir is None:
        if args.mode == "simulation":
            args.data_dir = "Data/Simulation"
        else:
            args.data_dir = "Data/Real"

    # Default mode if not specified
    if args.mode is None:
        args.mode = "simulation"

    try:
        # Create visualizer
        viz = UnifiedVisualizationGenerator(
            data_directory=args.data_dir,
            mode=args.mode,
            interactive=False,
            overlay_dxf=args.overlay_dxf,
        )

        # Generate plots
        viz.generate_performance_plots()

        # Generate animation unless plots-only
        if not args.plots_only:
            viz.generate_animation()

    except Exception as e:
        print(f" Error: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
