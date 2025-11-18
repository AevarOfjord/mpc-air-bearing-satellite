"""
Real Hardware MPC Control System for Satellite Thruster Platform

Real-time testing framework for linearized MPC control of satellite thrusters.
Integrates motion capture feedback, camera streams, and hardware control.

System architecture:
- Motion capture: Motive OptiTrack system via motive_data_server.py
- Control: Optimized MPC with Gurobi solver for thruster commands
- Hardware interface: Serial communication with Arduino-based thruster controller
- Visualization: Live PyQt5 dashboard with cameras and 2D navigation map

Key features:
- Real-time feedback control with sub-100ms MPC solve times
- Multi-camera recording and live streaming
- Mission execution (waypoint, shape following)
- Collision avoidance with configured obstacles
- Comprehensive data logging and post-mission reports
- Safety monitoring and emergency stop
- Graceful shutdown with data preservation

Configuration:
- All parameters centralized in config package
- Structured config system for clean parameter access
- Hot-reload support for rapid testing iteration
"""

import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from PyQt5.QtWidgets import QApplication

from camera_manager import SatelliteCameraWindow, create_camera_manager
from config import (
    SatelliteConfig,
    StructuredConfig,
    build_structured_config,
    use_structured_config,
)
from data_logger import create_data_logger

# Import logging
from logging_config import setup_logging
from mission_report_generator import create_mission_report_generator
from mission_state_manager import MissionStateManager
from mode_setup_manager import ModeSetupManager
from mpc import SatelliteMPCOptimized as SatelliteMPCLinearized
from navigation_utils import (
    angle_difference,
    calculate_safe_path_to_waypoint,
    normalize_angle,
    point_to_line_distance,
)
from path_planning_manager import create_path_planning_manager
from real_ui_components import LiveAnimationWindow
from satellite_hardware_interface import SatelliteHardwareInterface
from telemetry_client import create_telemetry_client
from visualization_manager import PostTestVisualizer, create_realtime_visualizer

# Set up logger
logger = setup_logging(__name__, log_file="Data/real_control.log", simple_format=True)

# Import visualization generator with error handling
try:
    from mission import (
        MissionManager,
        get_path_tangent_orientation,
        get_position_on_path,
    )
    from visualize import UnifiedVisualizationGenerator
except ImportError:
    UnifiedVisualizationGenerator = None
    MissionManager = None
    get_position_on_path = None
    get_path_tangent_orientation = None
    logger.warning(
        "WARNING: Visualization or Mission components not available. Limited functionality available."
    )

plt.rcParams["animation.ffmpeg_path"] = SatelliteConfig.FFMPEG_PATH


# Use camera configuration from Config
CAMERA_URLS = SatelliteConfig.CAMERA_URLS
CAMERA_ORDER = SatelliteConfig.CAMERA_ORDER


class RealSatelliteMPCLinearized:
    def get_dashboard_map_state(self) -> Dict[str, Any]:
        """Provide a rich map state dict so the live map matches the animation style."""
        from config import SatelliteConfig as _Cfg

        state = {}
        # Current
        state["x"], state["y"] = float(self.current_position[0]), float(
            self.current_position[1]
        )
        state["yaw"] = float(self.current_angle)
        state["satellite_size"] = float(getattr(_Cfg, "SATELLITE_SIZE", 0.29))
        # Trajectory
        state["trajectory"] = [tuple(p) for p in getattr(self, "trajectory", [])]
        # Target
        if hasattr(self, "target_state") and self.target_state is not None:
            state["target_x"] = float(self.target_state[0])  # type: ignore[arg-type,index]
            state["target_y"] = float(self.target_state[1])
            state["target_yaw"] = float(self.target_state[4])  # type: ignore[arg-type,index]
        else:
            state["target_x"] = state["target_y"] = state["target_yaw"] = None
        # Thrusters
        state["thrusters"] = getattr(_Cfg, "THRUSTER_POSITIONS", {}).copy()
        if hasattr(self, "current_thrusters") and isinstance(
            self.current_thrusters, np.ndarray
        ):
            state["active_thrusters"] = [
                i + 1 for i, v in enumerate(self.current_thrusters.tolist()) if v > 0.5
            ]
        else:
            state["active_thrusters"] = []
        # Obstacles
        state["obstacles_enabled"] = bool(getattr(_Cfg, "OBSTACLES_ENABLED", False))
        state["obstacles"] = getattr(_Cfg, "OBSTACLES", []).copy()
        # DXF overlays
        state["dxf_base_shape"] = getattr(_Cfg, "DXF_BASE_SHAPE", [])
        state["dxf_offset_path"] = getattr(_Cfg, "DXF_SHAPE_PATH", [])
        state["dxf_center"] = getattr(_Cfg, "DXF_SHAPE_CENTER", None)
        return state

    def calculate_obstacle_avoiding_path(
        self, start_pos: np.ndarray, target_pos: np.ndarray
    ) -> list:
        """
        Calculate a path from start to target that avoids all configured obstacles (delegated to PathPlanningManager).

        Args:
            start_pos: Starting position as numpy array
            target_pos: Target position as numpy array

        Returns:
            List of waypoints [(x, y), ...] from start to target
        """
        return self.path_planner.calculate_obstacle_avoiding_path(start_pos, target_pos)

    def update_target_with_obstacle_avoidance(
        self, final_target_pos: Tuple[float, float], final_target_angle: float = 0.0
    ) -> None:
        """
        Update simulation target while avoiding obstacles (delegated to PathPlanningManager).

        Args:
            final_target_pos: Final target position (x, y)
            final_target_angle: Final target orientation in radians
        """
        target_pos, target_angle = self.path_planner.update_target_with_obstacle_avoidance(
            current_position=self.current_position,
            final_target_pos=final_target_pos,
            final_target_angle=final_target_angle,
            control_time=self.control_time,
            position_tolerance=self.position_tolerance,
        )
        self.target_state = np.array(
            [target_pos[0], target_pos[1], 0.0, 0.0, target_angle, 0.0]
        )

    def check_target_reached(self) -> bool:
        """
        Check if the satellite has reached the target within tolerances.
        Returns True if position, velocity, angle, and angular velocity are all within tolerance.
        """
        # Get current state and target state
        current_state = self.get_current_state()
        target_state = self.target_state

        # Use config tolerances
        pos_tol = getattr(self, "position_tolerance", 0.05)
        vel_tol = getattr(self, "velocity_tolerance", 0.05)
        ang_tol = getattr(self, "angle_tolerance", 0.05)
        angvel_tol = getattr(self, "angular_velocity_tolerance", 0.05)

        pos_error = np.linalg.norm(current_state[:2] - target_state[:2])
        vel_error = np.linalg.norm(current_state[2:4] - target_state[2:4])
        ang_error = abs(self.angle_difference(target_state[4], current_state[4]))  # type: ignore[arg-type,index]
        angvel_error = abs(current_state[5] - target_state[5])  # type: ignore[index]

        return (  # type: ignore[return-value]
            pos_error < pos_tol
            and vel_error < vel_tol
            and ang_error < ang_tol
            and angvel_error < angvel_tol
        )

    def update_target_state_for_mode(self, current_state: np.ndarray) -> None:
        """
        Update the target state based on the current control mode.

        Delegates to MissionStateManager for centralized mission logic.
        Replaces ~330 lines of complex nested code with clean delegation.

        Args:
            current_state: Current state vector [x, y, vx, vy, theta, omega]
        """
        # Delegate to MissionStateManager for centralized mission logic
        target_state = self.mission_manager.update_target_state(
            current_position=self.current_position,
            current_angle=self.current_angle,
            current_time=self.control_time,
            current_state=current_state,
        )

        # If mission returns None, it means mission is complete
        if target_state is None:
            # Check if it's a completion signal (mission finished)
            if self.mission_manager.dxf_completed:
                logger.info("DXF PROFILE COMPLETED! Profile successfully traversed.")
                self.is_running = False
                self.print_performance_summary()
                return
        else:
            # Update our target state from the mission manager
            self.target_state = target_state

        return

    def poll_distance_sensors(self) -> None:
        """
        Polls distance sensors and updates internal state.
        Replace this with actual sensor polling if available.
        """
        pass

    def get_dashboard_distances(self) -> Dict[str, str]:
        """
        Returns a dictionary of distance sensor readings for the dashboard.
        Replace this with actual sensor data if available.

        Returns:
            Dictionary mapping camera names to distance readings
        """
        return {cam: "--" for cam in ["Front", "Right", "Back", "Left"]}

    def get_dashboard_position_angle(self) -> Tuple[np.ndarray, float]:
        """
        Returns the latest position and angle for the dashboard display.

        Returns:
            Tuple of (position array, angle in radians)
        """
        pos = getattr(self, "latest_position", self.current_position)
        angle = getattr(self, "latest_angle", self.current_angle)
        return pos, angle

    """
    Real-time MPC control system for satellite testing.

    Integrates motion capture telemetry from the Motive system with hardware control
    for comprehensive satellite attitude and position control testing.
    """

    def __init__(
        self,
        target_pos: Optional[Tuple[float, float]] = None,
        target_angle: Optional[float] = None,
        motive_server_url: str = "http://127.0.0.1:5000/data",
        hardware_type: str = "custom",
        config: Optional[StructuredConfig] = None,
        config_overrides: Optional[Dict[str, Dict[str, Any]]] = None,
        **hardware_kwargs,
    ):
        """
        Initialize the real-time MPC control system.

        Args:
            target_pos: Desired position coordinates (defaults to Config values)
            target_angle: Desired orientation in radians (defaults to Config values)
            motive_server_url: Motive data server endpoint
            hardware_type: Hardware interface type ("serial", "udp", "tcp", "custom")
            config: Optional structured config snapshot to run against
            config_overrides: Nested override dict passed to build_structured_config()
            **hardware_kwargs: Additional hardware configuration parameters
        """
        self.structured_config = (
            config.clone() if config else build_structured_config(config_overrides)
        )
        with use_structured_config(self.structured_config.clone()):
            self._initialize_from_active_config(
                target_pos=target_pos,
                target_angle=target_angle,
                motive_server_url=motive_server_url,
                hardware_type=hardware_type,
                hardware_kwargs=hardware_kwargs,
            )

    def _initialize_from_active_config(
        self,
        target_pos: Optional[Tuple[float, float]],
        target_angle: Optional[float],
        motive_server_url: str,
        hardware_type: str,
        hardware_kwargs: Dict[str, Any],
    ) -> None:
        if target_pos is None:
            target_pos = SatelliteConfig.DEFAULT_TARGET_POS
        if target_angle is None:
            target_angle = SatelliteConfig.DEFAULT_TARGET_ANGLE

        # Motive telemetry configuration
        self.motive_server_url = motive_server_url
        self.telemetry_timeout = SatelliteConfig.TELEMETRY_TIMEOUT
        self.telemetry_update_interval = (
            SatelliteConfig.CONTROL_DT
        )  # Match control rate

        # Initialize telemetry client
        self.telemetry_client = create_telemetry_client(
            server_url=motive_server_url, timeout=self.telemetry_timeout
        )

        # Hardware interface setup
        self.hardware = SatelliteHardwareInterface(hardware_type, **hardware_kwargs)
        if not self.hardware.is_connected:
            logger.warning("Warning: Hardware interface not connected!")
            logger.warning("Commands will be logged but not executed on hardware.")

        self.current_position = np.array([0.0, 0.0])  # [x, z] coordinates in meters
        self.current_angle = 0.0  # yaw angle in radians
        self.current_velocity = np.array([0.0, 0.0])
        self.current_angular_velocity = 0.0

        self.previous_position = np.array([0.0, 0.0])
        self.previous_angle = 0.0
        self.previous_telemetry_time = 0.0

        self.target_state = np.array(
            [target_pos[0], target_pos[1], 0.0, 0.0, target_angle, 0.0]
        )
        print(f"POINT-TO-POINT MODE: Target ({target_pos[0]:.2f}, {target_pos[1]:.2f})")

        satellite_params = SatelliteConfig.get_satellite_params()
        mpc_params = SatelliteConfig.get_mpc_params()
        self.mpc_controller = SatelliteMPCLinearized(satellite_params, mpc_params)

        self.is_running = False
        self.control_time = 0.0
        self.max_control_time = SatelliteConfig.MAX_SIMULATION_TIME
        self.control_update_interval = SatelliteConfig.CONTROL_DT
        self.last_control_update = 0.0
        self.next_control_time = 0.0

        self.target_reached_time = None
        self.approach_phase_start_time = 0.0
        self.target_maintenance_time = 0.0
        self.times_lost_target = 0
        self.maintenance_position_errors = []
        self.maintenance_angle_errors = []

        # Data logging
        self.state_history = []
        self.control_history = []
        self.target_history = []
        self.mpc_solve_times = []
        self.mpc_info_history = []
        self.telemetry_history = []

        # Current control
        self.current_thrusters = np.zeros(8, dtype=np.float64)
        self.previous_thrusters = np.zeros(8, dtype=np.float64)

        self.position_tolerance = SatelliteConfig.POSITION_TOLERANCE
        self.angle_tolerance = SatelliteConfig.ANGLE_TOLERANCE
        self.velocity_tolerance = SatelliteConfig.VELOCITY_TOLERANCE
        self.angular_velocity_tolerance = SatelliteConfig.ANGULAR_VELOCITY_TOLERANCE

        # Initialize MissionStateManager for centralized mission logic
        self.mission_manager = MissionStateManager(
            position_tolerance=self.position_tolerance,
            angle_tolerance=self.angle_tolerance,
            normalize_angle_func=self.normalize_angle,
            angle_difference_func=self.angle_difference,
            point_to_line_distance_func=self.point_to_line_distance,
            calculate_safe_path_func=self.calculate_safe_path_to_waypoint,
        )

        # Initialize data logger
        self.data_logger = create_data_logger(mode="real")
        self.data_save_path = None

        # Initialize mission report generator
        self.report_generator = create_mission_report_generator(SatelliteConfig)

        # Initialize path planning manager
        self.path_planner = create_path_planning_manager(SatelliteConfig)

        # Initialize visualization manager
        self.visualizer = create_realtime_visualizer()
        self.setup_visualization()

        # Detect and setup operation mode
        ModeSetupManager.setup_mode_specific_variables(self)

        print("Real-time MPC Controller initialized:")
        print(
            f"  Hardware: {hardware_type} ({' Connected' if self.hardware.is_connected else ' Not Connected'})"
        )
        if not self.hardware.is_connected:
            print("  WARNING: Hardware not connected - commands will not be sent!")
        print(f"  Motive server: {self.motive_server_url}")
        if target_pos is not None and target_angle is not None:
            print(
                f"  Target: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
            )
        print(f"  Control frequency: {1 / self.control_update_interval:.1f} Hz")
        print(f"  Prediction horizon: {mpc_params['prediction_horizon']}")
        print(f"  Control horizon: {mpc_params['control_horizon']}")

    def setup_visualization(self) -> None:
        """Configure matplotlib display for real-time monitoring (delegated to RealTimeVisualizer)."""
        self.visualizer.setup()
        self.fig = self.visualizer.fig
        self.ax = self.visualizer.ax
        self.trajectory = self.visualizer.trajectory
        self.max_trajectory_points = self.visualizer.max_trajectory_points

    def get_telemetry_data(self, max_retries: int = 2) -> Optional[dict]:
        """
        Retrieve current position data from the Motive system (delegated to TelemetryClient).

        Args:
            max_retries: Maximum number of retry attempts on failure

        Returns:
            Dictionary containing x, y, yaw, timestamp measurements or None if unavailable
        """
        return self.telemetry_client.get_telemetry(max_retries=max_retries)

    def update_state_from_telemetry(self, telemetry: dict) -> None:
        """
        Update satellite state using telemetry measurements and compute velocity estimates.

        Args:
            telemetry: Dictionary containing x, y, yaw, timestamp data
        """
        current_time = telemetry["timestamp"]
        current_pos = np.array([telemetry["x"], telemetry["y"]])
        current_angle = telemetry["yaw"]

        # Calculate velocities using finite differences
        if self.previous_telemetry_time > 0:
            dt = current_time - self.previous_telemetry_time
            if dt > 0:
                # Linear velocity estimation
                self.current_velocity = (current_pos - self.previous_position) / dt

                # Angular velocity estimation
                angle_diff = self.normalize_angle(current_angle - self.previous_angle)
                self.current_angular_velocity = angle_diff / dt

        # Update current state
        self.current_position = current_pos
        self.current_angle = current_angle

        self.previous_position = current_pos.copy()
        self.previous_angle = current_angle
        self.previous_telemetry_time = current_time

        self.trajectory.append(current_pos.copy())
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)

        self.latest_position = self.current_position.copy()
        self.latest_angle = self.current_angle

    def get_current_state(self) -> np.ndarray:
        """Return current satellite state vector as [x, y, vx, vy, theta, omega]."""
        return np.array(
            [
                self.current_position[0],  # x
                self.current_position[1],  # z (treating as y)
                self.current_velocity[0],  # vx
                self.current_velocity[1],  # vz (treating as vy)
                self.current_angle,  # theta
                self.current_angular_velocity,  # omega
            ]
        )

    def send_thruster_command(self, thruster_action: np.ndarray) -> bool:
        """
        Execute thruster command on satellite hardware.

        Args:
            thruster_action: Array of 8 thruster activation signals (0 or 1)

        Returns:
            True if command was sent successfully, False otherwise
        """
        success = self.hardware.send_thruster_command(thruster_action)

        if not success:
            logger.warning("Warning: Hardware command transmission failed")

        return success

    def create_data_directories(self) -> Path:
        """
        Create timestamped directory structure for test data storage.
        Returns the path to the created subdirectory.
        """
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

        # Create directory structure: Data/Real_Test/timestamp
        base_path = Path("Data")
        real_path = base_path / "Real_Test"
        timestamped_path = real_path / timestamp

        # Ensure directories exist
        timestamped_path.mkdir(parents=True, exist_ok=True)

        print(f"Data directory created: {timestamped_path}")
        return timestamped_path

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range (delegated to navigation_utils)."""
        return normalize_angle(angle)

    def angle_difference(self, target_angle: float, currentAngle: float) -> float:
        """
        Calculate the shortest angular difference between target and current angles (delegated to navigation_utils).
        This prevents the 270° transition issue by always taking the shortest path.

        Returns: angle difference in [-pi, pi] range, positive = CCW rotation needed
        """
        return angle_difference(target_angle, currentAngle)

    def log_control_step(
        self,
        mpc_start_time: float,
        mpc_computation_time: float,
        command_sent_time: float,
        thruster_action: np.ndarray,
        telemetry: dict,
        mpc_info: Optional[dict] = None,
    ):
        """
        Record detailed control step data with timing analysis for post-test evaluation.
        """
        current_state = self.get_current_state()

        # Calculate control errors
        error_x = current_state[0] - self.target_state[0]
        error_y = current_state[1] - self.target_state[1]
        error_yaw = self.angle_difference(self.target_state[4], current_state[4])  # type: ignore[arg-type,index]

        # Generate command string representation
        command_vector = thruster_action.astype(int)
        command_hex = "0x" + "".join([str(x) for x in command_vector])

        # Timing analysis
        total_mpc_loop_time = command_sent_time - mpc_start_time
        actual_time_interval = (
            self.control_time - self.last_control_update
            if self.last_control_update > 0
            else self.control_update_interval
        )

        info = mpc_info or {}
        mpc_status_name = info.get("status_name")
        mpc_solver_type = info.get("solver_type")
        mpc_time_limit = info.get("solver_time_limit")
        mpc_time_exceeded = info.get("time_limit_exceeded")
        mpc_fallback_used = info.get("solver_fallback")
        mpc_objective = info.get("objective_value")
        mpc_solve_time = info.get("solve_time")
        mpc_iterations = info.get("iterations")
        mpc_optimality_gap = info.get("optimality_gap")

        # Determine mission phase. Only use DXF state when shape mode is active.
        if getattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE", False) and getattr(
            SatelliteConfig, "DXF_SHAPE_PHASE", ""
        ):
            mission_phase = SatelliteConfig.DXF_SHAPE_PHASE  # POSITIONING, TRACKING, or STABILIZING
        else:
            mission_phase = (
                "STABILIZING" if self.target_reached_time is not None else "APPROACHING"
            )

        # Waypoint number (0 if not in waypoint mode)
        waypoint_number = 0

        # Compute velocity errors
        error_vx = self.target_state[2] - current_state[2]
        error_vy = self.target_state[3] - current_state[3]
        error_angular_vel = self.target_state[5] - current_state[5]

        # Count active thrusters
        total_active_thrusters = int(np.sum(command_vector > 0.5))

        # Count thruster switches (compare to previous command)
        thruster_switches = 0
        if hasattr(self, 'previous_command') and self.previous_command is not None:
            thruster_switches = int(np.sum(np.abs(command_vector - self.previous_command) > 0.5))
        self.previous_command = command_vector.copy()

        # Comprehensive log entry
        log_entry = {
            "Step": self.data_logger.current_step,
            "MPC_Start_Time": mpc_start_time,
            "Control_Time": self.control_time,
            "Actual_Time_Interval": actual_time_interval,
            "CONTROL_DT": self.control_update_interval,
            "Mission_Phase": mission_phase,
            "Waypoint_Number": waypoint_number,
            "Telemetry_X_mm": telemetry["x"] * 1000,  # Convert back to mm for logging
            "Telemetry_Z_mm": telemetry["y"] * 1000,  # Use 'y' key instead of 'z'
            "Telemetry_Yaw_deg": np.degrees(telemetry["yaw"]),
            "Current_X": current_state[0],
            "Current_Y": current_state[1],
            "Current_Yaw": current_state[4],
            "Current_VX": current_state[2],
            "Current_VY": current_state[3],
            "Current_Angular_Vel": current_state[5],
            "Target_X": self.target_state[0],
            "Target_Y": self.target_state[1],
            "Target_Yaw": self.target_state[4],  # type: ignore[index]
            "Target_VX": self.target_state[2],
            "Target_VY": self.target_state[3],
            "Target_Angular_Vel": self.target_state[5],
            "Error_X": error_x,
            "Error_Y": error_y,
            "Error_Yaw": error_yaw,
            "Error_VX": error_vx,
            "Error_VY": error_vy,
            "Error_Angular_Vel": error_angular_vel,
            "MPC_Computation_Time": mpc_computation_time,
            "MPC_Status": mpc_status_name,
            "MPC_Solver": mpc_solver_type,
            "MPC_Solver_Time_Limit": mpc_time_limit,
            "MPC_Solve_Time": mpc_solve_time,
            "MPC_Time_Limit_Exceeded": mpc_time_exceeded,
            "MPC_Fallback_Used": mpc_fallback_used,
            "MPC_Objective": mpc_objective,
            "MPC_Iterations": mpc_iterations,
            "MPC_Optimality_Gap": mpc_optimality_gap,
            "Command_Vector": str(command_vector.tolist()),
            "Command_Hex": command_hex,
            "Command_Sent_Time": command_sent_time,
            "Total_Active_Thrusters": total_active_thrusters,
            "Thruster_Switches": thruster_switches,
            "Total_MPC_Loop_Time": total_mpc_loop_time,
            "Timing_Violation": mpc_computation_time
            > (self.control_update_interval - 0.02),
        }

        self.data_logger.log_entry(log_entry)

    def save_csv_data(self) -> None:
        """Export all recorded data to CSV file for analysis (delegated to DataLogger)."""
        self.data_logger.save_csv_data()

    def update_mpc_control(self) -> None:
        """Execute MPC control update cycle with telemetry feedback."""
        if self.control_time >= self.next_control_time:
            # Retrieve current telemetry
            telemetry = self.get_telemetry_data()
            if telemetry is None:
                logger.warning("Warning: No telemetry available, skipping control update")
                # Schedule next attempt
                self.next_control_time += self.control_update_interval
                return  # Process telemetry and get current state
            self.update_state_from_telemetry(telemetry)
            current_state = self.get_current_state()

            # Update target state based on operation mode
            self.update_target_state_for_mode(current_state)

            # MPC timing
            mpc_start_time = self.control_time

            # Execute MPC optimization
            start_compute_time = time.time()
            thruster_action, mpc_info = self.mpc_controller.get_control_action(
                current_state, self.target_state, self.previous_thrusters  # type: ignore[arg-type]
            )
            end_compute_time = time.time()

            mpc_computation_time = end_compute_time - start_compute_time

            # Send control commands
            if thruster_action is not None:
                if thruster_action.ndim == 2:
                    thruster_action = thruster_action[0, :]

                # Validate thruster count
                if len(thruster_action) != 8:
                    logger.warning(
                        f"WARNING: Invalid thruster array length: {len(thruster_action)}"
                    )
                    thruster_action = np.zeros(8, dtype=np.float64)

                # Execute command
                self.send_thruster_command(thruster_action)
                self.current_thrusters = thruster_action.copy()
                self.previous_thrusters = thruster_action.copy()
            else:
                # MPC failure - safety shutdown
                thruster_action = np.zeros(8, dtype=np.float64)
                self.send_thruster_command(thruster_action)
                self.current_thrusters = thruster_action.copy()
                logger.warning("Warning: MPC optimization failed, thrusters disabled")

            # Record command execution time
            command_sent_time = self.control_time

            if self.data_save_path is not None:
                self.log_control_step(
                    mpc_start_time,
                    mpc_computation_time,
                    command_sent_time,
                    thruster_action,
                    telemetry,
                    mpc_info,
                )

            self.state_history.append(current_state.copy())
            self.control_history.append(thruster_action.copy())
            self.target_history.append(self.target_state.copy())  # type: ignore[attr-defined]
            self.mpc_solve_times.append(mpc_computation_time)
            self.mpc_info_history.append(mpc_info if mpc_info else {})
            self.telemetry_history.append(telemetry.copy())

            self.last_control_update = self.control_time
            # Status reporting
            pos_error = np.linalg.norm(current_state[:2] - self.target_state[:2])
            ang_error = abs(self.angle_difference(self.target_state[4], current_state[4]))  # type: ignore[arg-type,index]

            # Determine status message
            # For shape following, use DXF_SHAPE_PHASE; for waypoint mode, use target_reached_time
            stabilization_time = None  # Initialize to avoid UnboundLocalError
            shape_mode_active = getattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE", False)
            if shape_mode_active and getattr(SatelliteConfig, "DXF_SHAPE_PHASE", ""):
                phase = SatelliteConfig.DXF_SHAPE_PHASE
                # Add timing info for all Shape Following phases
                phase_time = None
                if phase == "POSITIONING" and hasattr(SatelliteConfig, 'DXF_POSITIONING_START_TIME') and SatelliteConfig.DXF_POSITIONING_START_TIME is not None:
                    phase_time = self.control_time - SatelliteConfig.DXF_POSITIONING_START_TIME
                elif phase == "PATH_STABILIZATION" and hasattr(SatelliteConfig, 'DXF_PATH_STABILIZATION_START_TIME') and SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME is not None:
                    phase_time = self.control_time - SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME
                elif phase == "TRACKING" and hasattr(SatelliteConfig, 'DXF_TRACKING_START_TIME') and SatelliteConfig.DXF_TRACKING_START_TIME is not None:
                    phase_time = self.control_time - SatelliteConfig.DXF_TRACKING_START_TIME
                elif phase == "RETURNING" and hasattr(SatelliteConfig, 'DXF_RETURN_START_TIME') and SatelliteConfig.DXF_RETURN_START_TIME is not None:
                    phase_time = self.control_time - SatelliteConfig.DXF_RETURN_START_TIME
                elif phase == "STABILIZING" and hasattr(SatelliteConfig, 'DXF_STABILIZATION_START_TIME') and SatelliteConfig.DXF_STABILIZATION_START_TIME is not None:
                    phase_time = self.control_time - SatelliteConfig.DXF_STABILIZATION_START_TIME

                if phase_time is not None:
                    status_msg = f"{phase} (t={phase_time:.1f}s)"
                else:
                    status_msg = phase
            else:
                # Waypoint mode status
                if self.target_reached_time is not None:
                    stabilization_time = self.control_time - self.target_reached_time
                    status_msg = f"STABILIZING (t={stabilization_time:.1f}s)"
                else:
                    approach_time = max(
                        self.control_time - self.approach_phase_start_time, 0.0
                    )
                    status_msg = f"APPROACHING (t={approach_time:.1f}s)"

            active_thruster_ids = [
                int(x) for x in np.where(thruster_action > 0.5)[0] + 1
            ]

            # Two-line format with empty line separator
            logger.info(
                f"t={self.control_time:5.1f}s: {status_msg} pos_err={pos_error:.3f}m, ang_err={np.degrees(ang_error):5.1f}°\n"
                f" solve_time={mpc_computation_time:.3f}s, thrusters={active_thruster_ids}\n"
            )

            # Log terminal message to CSV
            terminal_entry = {
                "Time": self.control_time,
                "Status": status_msg,
                "Stabilization_Time": stabilization_time if stabilization_time is not None else "",
                "Position_Error_m": pos_error,
                "Angle_Error_deg": np.degrees(ang_error),
                "Active_Thrusters": str(active_thruster_ids),
                "Solve_Time_s": mpc_computation_time,
                "Next_Update_s": "",  # Not applicable for real hardware
            }
            self.data_logger.log_terminal_message(terminal_entry)

            # Real-time constraint check
            if mpc_computation_time > (self.control_update_interval - 0.02):
                logger.warning(
                    f"WARNING: MPC computation time ({mpc_computation_time:.3f}s) exceeds real-time limit!"
                )

    def detect_operation_mode(self) -> str:
        """
        Detect which operation mode is active (delegated to ModeSetupManager).

        Returns:
            String indicating the operation mode
        """
        return ModeSetupManager.detect_operation_mode()

    def setup_mode_specific_variables(self) -> None:
        """Setup mode-specific variables (delegated to ModeSetupManager)."""
        ModeSetupManager.setup_mode_specific_variables(self)

    def point_to_line_distance(
        self, point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray
    ) -> float:
        """Calculate the shortest distance from a point to a line segment (delegated to navigation_utils)."""
        return point_to_line_distance(point, line_start, line_end)

    def calculate_safe_path_to_waypoint(
        self,
        start_pos: np.ndarray,
        target_pos: np.ndarray,
        all_targets: list,
        safety_radius: float,
    ) -> list:
        """
        Calculate a safe path from start to target that avoids obstacles (delegated to navigation_utils).

        Args:
            start_pos: Starting position as numpy array
            target_pos: Target position as numpy array
            all_targets: List of (x, y) target center positions
            safety_radius: Minimum distance to maintain from target centers

        Returns:
            List of waypoints [(x, y), ...] from start to target
        """
        return calculate_safe_path_to_waypoint(
            start_pos, target_pos, all_targets, safety_radius
        )

    def cleanup(self) -> None:
        """Clean up resources and close connections."""
        print("\nCleaning up...")

        # Send emergency stop to hardware
        if self.hardware and self.hardware.is_connected:
            print("Sending emergency stop command...")
            self.hardware.send_emergency_stop()
            time.sleep(0.1)

            # Close hardware connection
            self.hardware.close_connection()

        print(" Cleanup complete")

    def update_control_loop(self) -> None:
        """Execute main control cycle and manage test progression."""
        if not self.is_running:
            return

        # Execute MPC control update
        self.update_mpc_control()

        # Advance control time
        self.control_time += self.control_update_interval

        # Target status evaluation
        target_currently_reached = self.check_target_reached()

        if target_currently_reached:
            if self.target_reached_time is None:
                # Initial target acquisition
                self.target_reached_time = self.control_time
                print(
                    f"\nTARGET REACHED! Time: {self.control_time:.1f}s - MPC will continue to maintain position"
                )
            else:
                # Track maintenance performance
                self.target_maintenance_time = (
                    self.control_time - self.target_reached_time
                )
                current_state = self.get_current_state()
                pos_error = np.linalg.norm(current_state[:2] - self.target_state[:2])
                ang_error = abs(self.angle_difference(self.target_state[4], current_state[4]))  # type: ignore[arg-type,index]
                self.maintenance_position_errors.append(pos_error)
                self.maintenance_angle_errors.append(ang_error)
        else:
            if self.target_reached_time is not None:
                self.times_lost_target += 1
                print(
                    f"WARNING: Target lost at t={self.control_time:.1f}s - MPC working to regain control"
                )
                # Require continuous hold: reset maintenance tracking on loss
                self.target_reached_time = None
                self.target_maintenance_time = 0.0
                self.approach_phase_start_time = self.control_time

        # Test completion check
        # --- Stability stop logic ---
        if not (
            hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
            and SatelliteConfig.ENABLE_WAYPOINT_MODE
        ) and not (
            hasattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
            and SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
        ):
            if (
                self.target_maintenance_time
                >= SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
            ):
                print(
                    f"\n WAYPOINT MISSION COMPLETE! Stable at target for {SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME:.1f} seconds."
                )
                self.is_running = False
                self.print_performance_summary()
                return
        # Waypoint navigation (multiple waypoints)
        elif (
            hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
            and SatelliteConfig.ENABLE_WAYPOINT_MODE
        ):
            if getattr(SatelliteConfig, "MULTI_POINT_PHASE", None) == "COMPLETE":
                if (
                    self.target_maintenance_time
                    >= SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
                ):
                    print(
                        f"\n WAYPOINT MISSION COMPLETE! Stable at final target for {SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME:.1f} seconds."
                    )
                    self.is_running = False
                    self.print_performance_summary()
                    return

        # Fallback: Only stop when max time is reached
        if self.control_time >= self.max_control_time:
            print(f"\nTEST SESSION COMPLETE at {self.control_time:.1f}s")
            self.is_running = False
            self.print_performance_summary()
            return

    def draw_visualization(self) -> None:
        """Draw the real-time visualization (delegated to RealTimeVisualizer)."""
        self.visualizer.draw(
            current_position=self.current_position,
            current_angle=self.current_angle,
            target_state=self.target_state,  # type: ignore[arg-type]
            position_tolerance=self.position_tolerance,
        )

    def print_performance_summary(self) -> None:
        """Print comprehensive real test performance summary."""
        if not self.state_history:
            return

        # Save mission summary to file (no terminal output)
        self.save_mission_summary()

        # Print brief success message
        success = self.check_target_reached()

        print(f"\n{'=' * 60}")
        print(" MISSION COMPLETE!")
        print(f"{'=' * 60}")
        print(f"Status:            {' SUCCESS' if success else '  INCOMPLETE'}")

        if self.target_reached_time is not None:
            print(f"Target Reached:    {self.target_reached_time:.1f} s")
            print(f"Maintenance Time:  {self.target_maintenance_time:.1f} s")
            if self.maintenance_position_errors:
                avg_pos_error = np.mean(self.maintenance_position_errors)
                print(f"Avg Pos Error:     {avg_pos_error:.4f} m")
        else:
            print("Target:            Never reached")

        print(f"Duration:          {self.control_time:.1f} s")
        print(f"Control Updates:   {len(self.state_history)}")
        print(f"{'=' * 60}\n")

    def save_mission_summary(self) -> None:
        """
        Save comprehensive mission summary and performance metrics to a text file (delegated to MissionReportGenerator).
        """
        if not self.state_history:
            logger.warning(
                "WARNING: Cannot save mission summary: No state history available"
            )
            return

        if not self.data_save_path:
            logger.warning(
                "WARNING: Cannot save mission summary: No data save path set"
            )
            return

        summary_file_path = self.data_save_path / "mission_summary.txt"

        # Delegate to mission report generator
        self.report_generator.generate_report(
            output_path=summary_file_path,
            state_history=self.state_history,
            target_state=self.target_state,  # type: ignore[arg-type]
            control_time=self.control_time,
            mpc_solve_times=self.mpc_solve_times,
            control_history=self.control_history,
            target_reached_time=self.target_reached_time,
            target_maintenance_time=self.target_maintenance_time,
            times_lost_target=self.times_lost_target,
            maintenance_position_errors=self.maintenance_position_errors,
            maintenance_angle_errors=self.maintenance_angle_errors,
            position_tolerance=self.position_tolerance,
            angle_tolerance=self.angle_tolerance,
            control_update_interval=self.control_update_interval,
            angle_difference_func=self.angle_difference,
            check_target_reached_func=self.check_target_reached,
            motive_server_url=getattr(self, "motive_server_url", "N/A"),
            rigid_body_name=getattr(self.telemetry_client, "rigid_body_name", "N/A"),
            test_mode="REAL HARDWARE TEST",
        )

    def auto_generate_visualizations(self) -> None:
        """
        Automatically generate all visualizations after test completion (delegated to PostTestVisualizer).
        This matches the functionality in Simulation_Linearized.py.
        """
        PostTestVisualizer.generate_visualizations(self.data_save_path, UnifiedVisualizationGenerator)  # type: ignore[arg-type]

    def run_real_test(self, show_visualization: bool = True) -> None:
        """
        Run the real hardware test within an isolated config snapshot.

        Args:
            show_visualization: Whether to display real-time visualization
        """
        # Set up signal handler for Ctrl+C
        def signal_handler(sig, frame):
            print("\n\nReal hardware test cancelled by user (Ctrl+C)")
            self.is_running = False

        # Register the signal handler
        original_sigint = signal.signal(signal.SIGINT, signal_handler)

        try:
            with use_structured_config(self.structured_config.clone()):
                return self._run_real_test_with_globals(
                    show_visualization=show_visualization
                )
        finally:
            # Restore original signal handler
            signal.signal(signal.SIGINT, original_sigint)

    def _run_real_test_with_globals(self, show_visualization: bool = True) -> None:
        """
        Internal method to run real hardware test with active config globals.

        Args:
            show_visualization: Whether to display real-time visualization
        """
        print("\nStarting Real Satellite MPC Test...")
        print("Press Ctrl+C to stop")
        self.is_running = True

        # Clear any previous data from the logger
        self.data_logger.clear_logs()

        self.data_save_path = self.create_data_directories()
        self.data_logger.set_save_path(self.data_save_path)
        print(f" Data will be saved to: {self.data_save_path}")
        print("Testing telemetry connection...")
        telemetry = self.get_telemetry_data()
        if telemetry is None:
            logger.error("ERROR: Cannot connect to Motive telemetry server!")
            logger.error(f"Make sure the server is running at: {self.motive_server_url}")
            return
        else:
            print(
                f" Telemetry connection OK - Initial position: ({telemetry['x']:.3f}, {telemetry['y']:.3f}) m"
            )
            self.update_state_from_telemetry(telemetry)

        try:
            show_cameras = (
                input("Show live camera video during mission? (y/n): ").strip().lower()
                == "y"
            )
        except Exception:
            show_cameras = False
        try:
            show_live_anim = (
                input("Show live animation during mission? (y/n): ").strip().lower()
                == "y"
            )
        except Exception:
            show_live_anim = False

        # Start UI as requested
        from config import SatelliteConfig as _Cfg

        app = None
        self.camera_window = None
        self.anim_window = None
        self.headless_recorder = None

        if show_cameras or show_live_anim:
            app = QApplication.instance() or QApplication(sys.argv)
            if show_cameras:
                # Create camera manager for window mode
                camera_mgr = create_camera_manager(
                    camera_urls=CAMERA_URLS,
                    camera_order=CAMERA_ORDER,
                    data_dir=self.data_save_path,
                    frame_size=_Cfg.CAMERA_FRAME_SIZE,
                    fps=_Cfg.CAMERA_FPS,
                )
                camera_mgr.start(
                    recording_enabled=getattr(_Cfg, "CAMERA_RECORDING_ENABLED", False)
                )
                self.camera_window = SatelliteCameraWindow(
                    camera_manager=camera_mgr,
                    get_distances_func=self.get_dashboard_distances,
                    overlay_height=_Cfg.OVERLAY_HEIGHT,
                )
                self.camera_window.show()
            if show_live_anim:
                self.anim_window = LiveAnimationWindow(self.get_dashboard_map_state)
                self.anim_window.show()
        else:
            # Headless mode: use camera manager for background recording
            try:
                self.headless_recorder = create_camera_manager(
                    camera_urls=CAMERA_URLS,
                    camera_order=CAMERA_ORDER,
                    data_dir=self.data_save_path,
                    frame_size=_Cfg.CAMERA_FRAME_SIZE,
                    fps=_Cfg.CAMERA_FPS,
                )
                self.headless_recorder.start(
                    recording_enabled=getattr(_Cfg, "CAMERA_RECORDING_ENABLED", True)
                )
            except Exception as e:
                print(f"  Could not start headless camera recorder: {e}")

        # Main control loop in background thread
        def control_loop():
            try:
                while self.is_running:
                    start_time = time.time()
                    self.poll_distance_sensors()
                    self.update_control_loop()
                    elapsed = time.time() - start_time
                    sleep_time = max(0, self.telemetry_update_interval - elapsed)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
            except KeyboardInterrupt:
                print("\nTest stopped by user")
                self.is_running = False
            finally:
                self.cleanup()
                if self.data_save_path is not None:
                    print("\nSaving test data...")
                    self.save_csv_data()
                    self.save_mission_summary()  # Save mission summary
                    print(f" Data saved to: {self.data_save_path}")
                    print("\n Auto-generating visualizations...")
                    self.auto_generate_visualizations()
                    print(" All visualizations complete!")
                if getattr(self, "headless_recorder", None):
                    try:
                        self.headless_recorder.stop()  # type: ignore
                    except Exception:
                        pass
                # Close UI windows
                for w in [
                    getattr(self, "camera_window", None),
                    getattr(self, "anim_window", None),
                ]:
                    try:
                        if w:
                            w.close()
                    except Exception:
                        pass

        ctrl_thread = threading.Thread(target=control_loop, daemon=True)
        ctrl_thread.start()

        try:
            if app is not None:
                app.exec_()
                self.is_running = False
                ctrl_thread.join(timeout=2.0)
            else:
                # Use polling loop instead of blocking join to allow KeyboardInterrupt
                while ctrl_thread.is_alive():
                    ctrl_thread.join(timeout=0.5)  # Check every 0.5 seconds
        except KeyboardInterrupt:
            print("\n\nReal hardware test cancelled by user")
            self.is_running = False

            # Give the control thread a moment to finish its cleanup
            ctrl_thread.join(timeout=2.0)

            # If control thread cleanup didn't run, do it here
            if self.data_save_path is not None and self.data_logger.get_log_count() > 0:
                print("\nSaving test data...")
                try:
                    self.cleanup()
                    self.save_csv_data()
                    self.save_mission_summary()
                    print(f" Data saved to: {self.data_save_path}")

                    # Try to generate visualizations if we have enough data
                    if self.data_logger.get_log_count() > 10:
                        try:
                            print("\n Auto-generating visualizations...")
                            self.auto_generate_visualizations()
                            print(" All visualizations complete!")
                        except Exception as e:
                            logger.warning(f"WARNING: Could not generate visualizations: {e}")
                except Exception as e:
                    logger.error(f"Error during cleanup: {e}")

                # Stop camera recorder
                if getattr(self, "headless_recorder", None):
                    try:
                        self.headless_recorder.stop()  # type: ignore
                    except Exception:
                        pass

                # Close UI windows
                for w in [
                    getattr(self, "camera_window", None),
                    getattr(self, "anim_window", None),
                ]:
                    try:
                        if w:
                            w.close()
                    except Exception:
                        pass


if __name__ == "__main__":
    from real_test_modes import main

    main()
