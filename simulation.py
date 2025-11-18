"""
Linearized MPC Simulation for Satellite Thruster Control

Physics-based simulation environment for testing MPC control algorithms.
Implements realistic satellite dynamics with thruster actuation and disturbances.

Simulation features:
- Linearized dynamics with A, B matrices around equilibrium
- Eight-thruster configuration with individual force calibration
- Collision avoidance with circular obstacles
- Mission execution (waypoint, shape following)
- Sensor noise and disturbance simulation
- Real-time visualization with matplotlib

Physics modeling:
- 2D planar motion (x, y, θ) with velocities (vx, vy, ω)
- Thruster force and torque calculations
- Moment of inertia and mass properties
- Integration with configurable time steps

Data collection:
- Complete state history logging
- Control input recording
- MPC solve time statistics
- Mission performance metrics
- CSV export for analysis

Configuration:
- Modular config package for all parameters
- Structured config system for clean access
- Consistent with real hardware configuration
"""

import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

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
from mpc import SatelliteMPCOptimized as SatelliteMPCLinearized
from navigation_utils import angle_difference, normalize_angle, point_to_line_distance
from simulation_state_validator import create_state_validator_from_config
from simulation_visualization import create_simulation_visualizer
from testing_environment import SatelliteThrusterTester


# Set up logger with simple format for clean output
logger = setup_logging(__name__, log_file="Data/simulation.log", simple_format=True)

import platform

if platform.system() == "Darwin":  # macOS
    plt.rcParams["animation.ffmpeg_path"] = "/opt/homebrew/bin/ffmpeg"
elif platform.system() == "Windows":
    # Note: Update FFMPEG_PATH_WINDOWS in config/constants.py to the FFmpeg installation path
    from config.constants import Constants
    plt.rcParams["animation.ffmpeg_path"] = Constants.FFMPEG_PATH
# For Linux, ffmpeg is usually in PATH, so no need to set path

try:
    from mission import MissionManager
    from visualize import UnifiedVisualizationGenerator
except ImportError:
    logger.warning(
        "WARNING: Could not import visualization or mission components. Limited functionality available."
    )
    UnifiedVisualizationGenerator = None
    MissionManager = None


class SatelliteMPCLinearizedSimulation:
    """
    Simulation environment for linearized MPC satellite control.

    Combines physics from TestingEnvironment with linearized MPC controller
    for satellite navigation using linearized dynamics.
    """

    def __init__(
        self,
        start_pos: Optional[Tuple[float, float]] = None,
        target_pos: Optional[Tuple[float, float]] = None,
        start_angle: Optional[float] = None,
        target_angle: Optional[float] = None,
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_omega: float = 0.0,
        config: Optional[StructuredConfig] = None,
        config_overrides: Optional[Dict[str, Dict[str, Any]]] = None,
    ):
        """
        Initialize linearized MPC simulation.

        Args:
            start_pos: Starting position coordinates (uses Config default if None)
            target_pos: Target position coordinates (uses Config default if None)
            start_angle: Starting angle in radians (uses Config default if None)
            target_angle: Target angle in radians (uses Config default if None)
            start_vx: Initial X velocity in m/s (default: 0.0)
            start_vy: Initial Y velocity in m/s (default: 0.0)
            start_omega: Initial angular velocity in rad/s (default: 0.0)
            config: Optional structured config snapshot to run against
            config_overrides: Nested override dict passed to build_structured_config()
        """
        self.structured_config = (
            config.clone() if config else build_structured_config(config_overrides)
        )
        with use_structured_config(self.structured_config.clone()):
            self._initialize_from_active_config(
                start_pos,
                target_pos,
                start_angle,
                target_angle,
                start_vx,
                start_vy,
                start_omega,
            )

    def _initialize_from_active_config(
        self,
        start_pos: Optional[Tuple[float, float]],
        target_pos: Optional[Tuple[float, float]],
        start_angle: Optional[float],
        target_angle: Optional[float],
        start_vx: float = 0.0,
        start_vy: float = 0.0,
        start_omega: float = 0.0,
    ) -> None:
        if start_pos is None:
            start_pos = SatelliteConfig.DEFAULT_START_POS
        if target_pos is None:
            target_pos = SatelliteConfig.DEFAULT_TARGET_POS
        if start_angle is None:
            start_angle = SatelliteConfig.DEFAULT_START_ANGLE
        if target_angle is None:
            target_angle = SatelliteConfig.DEFAULT_TARGET_ANGLE

        # Type assertions for Pylance
        assert start_pos is not None
        assert target_pos is not None
        assert start_angle is not None
        assert target_angle is not None

        self.satellite = SatelliteThrusterTester()
        self.satellite.external_simulation_mode = True

        # Set initial state (including velocities)
        self.satellite.position = np.array(start_pos, dtype=np.float64)
        self.satellite.velocity = np.array([start_vx, start_vy], dtype=np.float64)
        self.satellite.angle = start_angle
        self.satellite.angular_velocity = start_omega

        # Store initial starting position and angle for reset functionality
        self.initial_start_pos = np.array(start_pos, dtype=np.float64)
        self.initial_start_angle = start_angle

        # Point-to-point mode
        self.target_state = np.array(
            [target_pos[0], target_pos[1], 0.0, 0.0, target_angle, 0.0]
        )
        logger.info(
            f"INFO: POINT-TO-POINT MODE: Target ({target_pos[0]:.2f}, {target_pos[1]:.2f})"
        )

        satellite_params = SatelliteConfig.get_satellite_params()
        satellite_params.update(
            {
                "satellite_size": self.satellite.satellite_size,
                "total_mass": self.satellite.total_mass,
                "moment_of_inertia": self.satellite.moment_of_inertia,
                "com_offset": self.satellite.com_offset,
            }
        )

        mpc_params = SatelliteConfig.get_mpc_params()
        # Use Config timing - do not override with satellite.dt

        self.mpc_controller = SatelliteMPCLinearized(satellite_params, mpc_params)

        # Simulation state
        self.is_running = False
        self.simulation_time = 0.0
        self.max_simulation_time = SatelliteConfig.MAX_SIMULATION_TIME
        self.control_update_interval = SatelliteConfig.CONTROL_DT
        self.last_control_update = 0.0
        self.next_control_simulation_time = 0.0  # Track next scheduled control update

        # ===== HARDWARE COMMAND DELAY SIMULATION =====
        # Simulates the delay between sending a command and thrusters actually firing
        # Uses Config parameters for realistic physics when enabled
        if SatelliteConfig.USE_REALISTIC_PHYSICS:
            self.VALVE_DELAY = (
                SatelliteConfig.THRUSTER_VALVE_DELAY
            )  # 50ms valve open/close delay
            self.THRUST_RAMPUP_TIME = (
                SatelliteConfig.THRUSTER_RAMPUP_TIME
            )  # 15ms ramp-up after valve opens
        else:
            self.VALVE_DELAY = 0.0  # Instant response for idealized physics
            self.THRUST_RAMPUP_TIME = 0.0

        # Valve state tracking for each thruster (independent state machines)
        # Each thruster tracks: last command, valve open time, valve close time, ramp start time
        self.thruster_last_command = np.zeros(8, dtype=np.float64)  # 0=OFF, 1=ON (last commanded state)
        self.thruster_open_command_time = np.full(
            8, -np.inf
        )  # Time when ON command was sent
        self.thruster_close_command_time = np.full(
            8, -np.inf
        )  # Time when OFF command was sent
        self.thruster_valve_open_time = np.full(
            8, -np.inf
        )  # Time when valve actually opened
        self.thruster_actual_output = np.zeros(8, dtype=np.float64)  # Current actual thrust output [0, 1]
        # ==============================================

        # Target maintenance tracking
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

        # Initialize state validator for centralized state validation
        self.state_validator = create_state_validator_from_config(
            {
                "position_tolerance": self.position_tolerance,
                "angle_tolerance": self.angle_tolerance,
                "velocity_tolerance": self.velocity_tolerance,
                "angular_velocity_tolerance": self.angular_velocity_tolerance,
            }
        )

        # Initialize data logger
        self.data_logger = create_data_logger(mode="simulation")
        self.report_generator = create_mission_report_generator(SatelliteConfig)
        self.data_save_path = None
        self.animation_frames = []

        logger.info("Linearized MPC Simulation initialized:")
        logger.info("INFO: Formulation: A*x[k] + B*u[k] (Linearized Dynamics)")
        logger.info(
            f"INFO: Start: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f}°"
        )
        logger.info(
            f"INFO: Target: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
        )
        logger.info(
            f"INFO: Control update rate: {1 / self.control_update_interval:.1f} Hz"
        )
        logger.info(f"INFO: Prediction horizon: {mpc_params['prediction_horizon']}")
        logger.info(f"INFO: Control horizon: {mpc_params['control_horizon']}")

        if SatelliteConfig.USE_REALISTIC_PHYSICS:
            logger.info("WARNING: REALISTIC PHYSICS ENABLED:")
            logger.info(
                f"WARNING: - Valve open/close delay: {self.VALVE_DELAY * 1000:.0f} ms"
            )
            logger.info(
                f"WARNING: - Thrust ramp-up time: {self.THRUST_RAMPUP_TIME * 1000:.0f} ms"
            )
            logger.info(
                f"WARNING: - Linear damping: {SatelliteConfig.LINEAR_DAMPING_COEFF:.3f} N/(m/s)"
            )
            logger.info(
                f"WARNING: - Rotational damping: {SatelliteConfig.ROTATIONAL_DAMPING_COEFF:.4f} N*m/(rad/s)"
            )
            logger.info(
                f"WARNING: - Position noise: {SatelliteConfig.POSITION_NOISE_STD * 1000:.2f} mm std"
            )
            logger.info(
                f"WARNING: - Angle noise: {np.degrees(SatelliteConfig.ANGLE_NOISE_STD):.2f}° std"
            )
        else:
            logger.info("INFO: Idealized physics (no delays, noise, or damping)")

        # Apply obstacle avoidance based on mode
        if SatelliteConfig.OBSTACLES_ENABLED and SatelliteConfig.get_obstacles():
            # For point-to-point mode (Mode 1)
            if (
                not hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                or not SatelliteConfig.ENABLE_WAYPOINT_MODE
            ):
                self.update_target_with_obstacle_avoidance(target_pos, target_angle)
                logger.info("Obstacle avoidance enabled for point-to-point navigation")

            # For waypoint mode (Mode 1) - initialize obstacle avoidance for first target
            elif (
                hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                and SatelliteConfig.ENABLE_WAYPOINT_MODE
            ):
                self.update_target_with_obstacle_avoidance(target_pos, target_angle)
                logger.info("Obstacle avoidance enabled for multi-point navigation")

        # Initialize visualization manager
        self.visualizer = create_simulation_visualizer(self)

    def get_current_state(self) -> np.ndarray:
        """Get current satellite state as [x, y, vx, vy, theta, omega]."""
        return np.array(
            [
                self.satellite.position[0],
                self.satellite.position[1],
                self.satellite.velocity[0],
                self.satellite.velocity[1],
                self.satellite.angle,
                self.satellite.angular_velocity,
            ]
        )

    def get_noisy_state(self, true_state: np.ndarray) -> np.ndarray:
        """
        Add realistic sensor noise to state measurements.
        Models OptiTrack measurement uncertainty and velocity estimation errors.

        Delegates to SimulationStateValidator for noise application.

        Args:
            true_state: True state [x, y, vx, vy, theta, omega]

        Returns:
            Noisy state with measurement errors added
        """
        return self.state_validator.apply_sensor_noise(true_state)

    def create_data_directories(self) -> Path:
        """
        Create the directory structure for saving data.
        Returns the path to the timestamped subdirectory.
        """
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

        # Create directory path: Data/Simulation/timestamp
        base_path = Path("Data")
        sim_path = base_path / "Simulation"
        timestamped_path = sim_path / timestamp

        # Create directories
        timestamped_path.mkdir(parents=True, exist_ok=True)

        logger.info(f"Created data directory: {timestamped_path}")
        return timestamped_path

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range (delegated to navigation_utils)."""
        return normalize_angle(angle)

    def angle_difference(self, target_angle: float, currentAngle: float) -> float:
        """
        Calculate the shortest angular difference between target and current angles (delegated to navigation_utils).
        This prevents the 360°/0° transition issue by always taking the shortest path.
        Returns: angle difference in [-pi, pi] range, positive = CCW rotation needed
        """
        return angle_difference(target_angle, currentAngle)

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
        Calculate a safe path from start to target that avoids obstacles.

        Args:
            start_pos: Starting position as numpy array
            target_pos: Target position as numpy array
            all_targets: List of (x, y) target center positions
            safety_radius: Minimum distance to maintain from target centers

        Returns:
            List of waypoints [(x, y), ...] from start to target
        """
        # Simple approach: If direct path crosses obstacles, generate intermediate waypoint
        waypoints = [tuple(start_pos)]

        blocking_obstacles = []
        for target_center_tuple in all_targets:
            target_center = np.array(target_center_tuple)
            distance_to_path = self.point_to_line_distance(
                target_center, start_pos, target_pos
            )
            if distance_to_path < safety_radius:
                blocking_obstacles.append(target_center)

        if blocking_obstacles:
            # For simplicity, create a waypoint that goes around the closest blocking obstacle
            closest_obstacle = min(
                blocking_obstacles,
                key=lambda obs: float(np.linalg.norm(obs - start_pos)),
            )

            # Create intermediate waypoint by going around the obstacle
            # direction_to_target_norm = direction_to_target / np.linalg.norm(
            #     direction_to_target

            direction_to_obstacle = closest_obstacle - start_pos
            direction_to_obstacle_norm = direction_to_obstacle / np.linalg.norm(
                direction_to_obstacle
            )

            perp_direction = np.array(
                [-direction_to_obstacle_norm[1], direction_to_obstacle_norm[0]]
            )

            # Determine which side gets us closer to target
            test_point1 = closest_obstacle + perp_direction * (safety_radius + 0.25)
            test_point2 = closest_obstacle - perp_direction * (safety_radius + 0.25)

            if np.linalg.norm(test_point1 - target_pos) < np.linalg.norm(
                test_point2 - target_pos
            ):
                intermediate_waypoint = test_point1
            else:
                intermediate_waypoint = test_point2

            waypoints.append(tuple(intermediate_waypoint))

        waypoints.append(tuple(target_pos))
        return waypoints

    # OBSTACLE AVOIDANCE METHODS

    def calculate_obstacle_avoiding_path(
        self, start_pos: np.ndarray, target_pos: np.ndarray
    ) -> list:
        """
        Calculate a path from start to target that avoids all configured obstacles.
        Uses simple waypoint generation around obstacles.

        Args:
            start_pos: Starting position as numpy array
            target_pos: Target position as numpy array

        Returns:
            List of waypoints [(x, y), ...] from start to target
        """
        if not SatelliteConfig.OBSTACLES_ENABLED:
            return [tuple(start_pos), tuple(target_pos)]

        if SatelliteConfig.is_path_clear(tuple(start_pos), tuple(target_pos)):
            return [tuple(start_pos), tuple(target_pos)]

        # Find blocking obstacles
        blocking_obstacles = []
        for obs_x, obs_y, obs_radius in SatelliteConfig.get_obstacles():
            obs_center = np.array([obs_x, obs_y])
            effective_radius = obs_radius + 0.25  # Fixed 0.25m safety margin

            distance = SatelliteConfig._point_to_line_distance(
                obs_center, start_pos, target_pos
            )
            if distance < effective_radius:
                blocking_obstacles.append((obs_center, effective_radius))

        if not blocking_obstacles:
            return [tuple(start_pos), tuple(target_pos)]

        # For simplicity, avoid the closest blocking obstacle
        closest_obstacle, closest_radius = min(
            blocking_obstacles,
            key=lambda x: float(np.linalg.norm(x[0] - start_pos)),
        )

        # Generate waypoint to go around obstacle
        waypoints = [tuple(start_pos)]

        # direction_to_target_norm = direction_to_target / np.linalg.norm(
        #     direction_to_target

        direction_to_obstacle = closest_obstacle - start_pos
        direction_to_obstacle_norm = direction_to_obstacle / np.linalg.norm(
            direction_to_obstacle
        )

        # Choose perpendicular direction to go around obstacle
        perp_direction = np.array(
            [-direction_to_obstacle_norm[1], direction_to_obstacle_norm[0]]
        )

        # Test both sides and choose the one that gets us closer to target
        # Use closest_radius directly without additional buffer to go as close as allowed
        test_point1 = closest_obstacle + perp_direction * closest_radius
        test_point2 = closest_obstacle - perp_direction * closest_radius

        if np.linalg.norm(test_point1 - target_pos) < np.linalg.norm(
            test_point2 - target_pos
        ):
            intermediate_waypoint = test_point1
        else:
            intermediate_waypoint = test_point2

        waypoints.append(tuple(intermediate_waypoint))
        waypoints.append(tuple(target_pos))

        logger.info(f"Generated obstacle-avoiding path: {len(waypoints)} waypoints")
        return waypoints

    def update_target_with_obstacle_avoidance(
        self, final_target_pos: Tuple[float, float], final_target_angle: float = 0.0
    ) -> None:
        """
        Update simulation target while avoiding obstacles.

        Args:
            final_target_pos: Final target position (x, y)
            final_target_angle: Final target orientation in radians
        """
        current_pos = np.array(self.satellite.position)
        target_pos = np.array(final_target_pos)

        # Generate obstacle-avoiding path
        if (
            not hasattr(self, "obstacle_avoiding_waypoints")
            or not self.obstacle_avoiding_waypoints
        ):
            self.obstacle_avoiding_waypoints = self.calculate_obstacle_avoiding_path(
                current_pos, target_pos
            )
            self.current_obstacle_waypoint_idx = (
                0 if len(self.obstacle_avoiding_waypoints) > 1 else -1
            )
            self.obstacle_waypoint_reached_time = None

            if len(self.obstacle_avoiding_waypoints) > 2:
                logger.info(
                    f"Multi-waypoint path planned: {len(self.obstacle_avoiding_waypoints)} waypoints"
                )

        if (
            self.current_obstacle_waypoint_idx >= 0
            and self.current_obstacle_waypoint_idx
            < len(self.obstacle_avoiding_waypoints) - 1
        ):
            # Navigate to intermediate waypoint
            waypoint_pos = self.obstacle_avoiding_waypoints[
                self.current_obstacle_waypoint_idx + 1
            ]  # +1 to skip start position
            waypoint_angle = 0.0  # Face forward during obstacle avoidance

            self.target_state = np.array(
                [waypoint_pos[0], waypoint_pos[1], 0.0, 0.0, waypoint_angle, 0.0]
            )

            pos_error = np.linalg.norm(self.satellite.position - np.array(waypoint_pos))
            if pos_error < self.position_tolerance:
                if self.obstacle_waypoint_reached_time is None:
                    self.obstacle_waypoint_reached_time = self.simulation_time
                elif (
                    self.simulation_time - self.obstacle_waypoint_reached_time
                    > SatelliteConfig.OBSTACLE_WAYPOINT_STABILIZATION_TIME
                ):
                    self.current_obstacle_waypoint_idx += 1
                    self.obstacle_waypoint_reached_time = None

                    if (
                        self.current_obstacle_waypoint_idx
                        >= len(self.obstacle_avoiding_waypoints) - 1
                    ):
                        logger.info(
                            "Obstacle avoidance complete. Proceeding to final target."
                        )
        else:
            # Navigate to final target
            self.target_state = np.array(
                [
                    final_target_pos[0],
                    final_target_pos[1],
                    0.0,
                    0.0,
                    final_target_angle,
                    0.0,
                ]
            )

    def log_simulation_step(
        self,
        mpc_start_time: float,
        mpc_computation_time: float,
        command_sent_time: float,
        thruster_action: np.ndarray,
        mpc_info: Optional[dict],
    ):
        """
        Log detailed simulation step data for CSV export with timing analysis.
        """
        current_state = self.get_current_state()

        # Calculate errors
        error_x = current_state[0] - self.target_state[0]
        error_y = current_state[1] - self.target_state[1]
        error_yaw = self.angle_difference(self.target_state[4], current_state[4])  # type: ignore[arg-type]

        # Convert command vector to hex string
        command_vector = thruster_action.astype(int)
        command_hex = "0x" + "".join([str(x) for x in command_vector])

        # Calculate total MPC loop time and actual time interval
        total_mpc_loop_time = command_sent_time - mpc_start_time
        actual_time_interval = (
            self.simulation_time - self.last_control_update
            if self.last_control_update > 0
            else self.control_update_interval
        )

        mpc_status_name = mpc_info.get("status_name") if mpc_info else None
        mpc_solver_type = mpc_info.get("solver_type") if mpc_info else None
        mpc_time_limit = mpc_info.get("solver_time_limit") if mpc_info else None
        mpc_time_exceeded = mpc_info.get("time_limit_exceeded") if mpc_info else None
        mpc_fallback_used = mpc_info.get("solver_fallback") if mpc_info else None
        mpc_objective = mpc_info.get("objective_value") if mpc_info else None
        mpc_solve_time = mpc_info.get("solve_time") if mpc_info else None
        mpc_iterations = mpc_info.get("iterations") if mpc_info else None
        mpc_optimality_gap = mpc_info.get("optimality_gap") if mpc_info else None

        # Determine mission phase. Use DXF state only when shape mode is active.
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

        # Create log entry with enhanced timing and MPC data (matches real hardware format)
        log_entry = {
            "Step": self.data_logger.current_step,
            "MPC_Start_Time": mpc_start_time,
            "Control_Time": self.simulation_time,
            "Actual_Time_Interval": actual_time_interval,
            "CONTROL_DT": self.control_update_interval,
            "Mission_Phase": mission_phase,
            "Waypoint_Number": waypoint_number,
            "Telemetry_X_mm": current_state[0] * 1000,  # Convert m to mm
            "Telemetry_Z_mm": current_state[1] * 1000,  # Convert m to mm (Y->Z coordinate)
            "Telemetry_Yaw_deg": np.degrees(current_state[4]),  # Convert rad to deg
            "Current_X": current_state[0],
            "Current_Y": current_state[1],
            "Current_Yaw": current_state[4],
            "Current_VX": current_state[2],
            "Current_VY": current_state[3],
            "Current_Angular_Vel": current_state[5],
            "Target_X": self.target_state[0],
            "Target_Y": self.target_state[1],
            "Target_Yaw": self.target_state[4],
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
        """Save all logged data to CSV file (delegated to DataLogger)."""
        self.data_logger.save_csv_data()

    def save_mission_summary(self) -> None:
        """Generate and save mission summary report using MissionReportGenerator."""
        if not self.state_history:
            logger.warning("WARNING: Cannot save mission summary: No state history available")
            return

        if not self.data_save_path:
            logger.warning("WARNING: Cannot save mission summary: No data save path set")
            return

        summary_path = self.data_save_path / "mission_summary.txt"
        self.report_generator.generate_report(
            output_path=summary_path,
            state_history=self.state_history,
            target_state=self.target_state,  # type: ignore[arg-type]
            control_time=self.simulation_time,
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
            motive_server_url="SIMULATION",
            rigid_body_name="SIM_MODEL",
            test_mode="SIMULATION",
        )

    def save_animation_mp4(self, fig: Any, ani: Any) -> Optional[str]:
        """
        Save the animation as MP4 file (delegated to SimulationVisualizationManager).

        Args:
            fig: Matplotlib figure object
            ani: Matplotlib animation object

        Returns:
            Path to saved MP4 file or None if save failed
        """
        self.visualizer.sync_from_controller()
        return self.visualizer.save_animation_mp4(fig, ani)

    def set_thruster_pattern(self, thruster_pattern: np.ndarray):
        """
        Send thruster command (immediate command, but valve response is delayed).

        Command is sent at current simulation_time, but valve opening/closing
        takes VALVE_DELAY (50ms) to complete. This tracks when commands are sent
        so that valve physics can be simulated correctly.

        Args:
            thruster_pattern: Binary array [0,1] indicating thruster ON/OFF commands
        """
        if thruster_pattern.ndim == 2:
            thruster_pattern = thruster_pattern[:, 0]

        self.current_thrusters = thruster_pattern.copy()

        # Record command time for each thruster that changed state
        for i in range(8):
            new_command = thruster_pattern[i]
            old_command = self.thruster_last_command[i]

            if new_command != old_command:
                # Command state changed
                if new_command > 0.5:  # ON command
                    self.thruster_open_command_time[i] = self.simulation_time
                else:  # OFF command
                    self.thruster_close_command_time[i] = self.simulation_time

                self.thruster_last_command[i] = new_command

    def process_command_queue(self) -> None:
        """
        Update actual thruster output based on valve delays and ramp-up dynamics.

        Valve behavior:
        - ON command sent at t → valve opens at t + VALVE_DELAY (50ms)
        - OFF command sent at t → valve closes at t + VALVE_DELAY (50ms)
        - Once valve opens, thrust ramps linearly over THRUST_RAMPUP_TIME (15ms)
        - If thruster stays ON across multiple control intervals, no re-ramping

        Called every simulation timestep to update actual thruster forces.
        """
        for i in range(8):
            current_command = self.thruster_last_command[i]
            open_cmd_time = self.thruster_open_command_time[i]
            close_cmd_time = self.thruster_close_command_time[i]

            # Determine which command is most recent
            if current_command > 0.5:  # Currently commanded ON
                # Check if valve has had time to open
                valve_open_time = open_cmd_time + self.VALVE_DELAY

                if self.simulation_time >= valve_open_time:
                    # Valve is open, apply ramp-up
                    time_since_valve_open = self.simulation_time - valve_open_time

                    # Check if we just opened (need to record valve open time for ramp-up)
                    if (
                        self.thruster_valve_open_time[i] < valve_open_time - 0.001
                    ):  # New opening
                        self.thruster_valve_open_time[i] = valve_open_time

                    # Linear ramp from 0 to 1 over THRUST_RAMPUP_TIME
                    if (
                        SatelliteConfig.USE_REALISTIC_PHYSICS
                        and self.THRUST_RAMPUP_TIME > 0
                    ):
                        ramp_progress = time_since_valve_open / self.THRUST_RAMPUP_TIME
                        self.thruster_actual_output[i] = min(1.0, ramp_progress)
                    else:
                        # Instant full thrust (idealized physics)
                        self.thruster_actual_output[i] = 1.0
                else:
                    # Valve hasn't opened yet (still in 50ms delay)
                    self.thruster_actual_output[i] = 0.0

            else:  # Currently commanded OFF
                # Check if valve has had time to close
                valve_close_time = close_cmd_time + self.VALVE_DELAY

                if self.simulation_time >= valve_close_time:
                    # Valve is closed
                    self.thruster_actual_output[i] = 0.0
                else:
                    # Valve hasn't closed yet, check if it was previously open
                    # If close command was sent before valve opened, output is 0
                    valve_open_time = open_cmd_time + self.VALVE_DELAY
                    if self.simulation_time < valve_open_time:
                        # Never opened
                        self.thruster_actual_output[i] = 0.0
                    else:
                        # Was open, now closing (keep last output during close delay)
                        # In reality, valves close immediately cutting thrust, so set to 0
                        self.thruster_actual_output[i] = 0.0

        # Update satellite active thrusters based on actual output
        # Only update activation times for NEW activations (OFF->ON transitions)
        # Set deactivation times for NEW deactivations (ON->OFF transitions)
        previous_active = self.satellite.active_thrusters.copy()
        self.satellite.active_thrusters.clear()

        for i, output in enumerate(self.thruster_actual_output):
            thruster_id = i + 1  # Thrusters are 1-indexed
            if output > 0.01:  # Threshold for activation (must have some thrust)
                self.satellite.active_thrusters.add(thruster_id)

                # Only set activation time if this is a NEW activation (wasn't active before)
                if thruster_id not in previous_active:
                    self.satellite.thruster_activation_time[
                        thruster_id
                    ] = self.simulation_time
                    # Clear any pending deactivation
                    if thruster_id in self.satellite.thruster_deactivation_time:
                        del self.satellite.thruster_deactivation_time[thruster_id]
            else:
                # Thruster commanded OFF
                if thruster_id in previous_active:
                    # This is a NEW deactivation (was ON, now commanded OFF)
                    self.satellite.thruster_deactivation_time[
                        thruster_id
                    ] = self.simulation_time

                # Clean up activation time (will be set again if re-activated)
                if thruster_id in self.satellite.thruster_activation_time:
                    del self.satellite.thruster_activation_time[thruster_id]

        # Clean up deactivation times for thrusters that have completed ramp-down
        if SatelliteConfig.USE_REALISTIC_PHYSICS:
            total_shutoff_time = (
                SatelliteConfig.THRUSTER_VALVE_DELAY
                + SatelliteConfig.THRUSTER_RAMPUP_TIME
            )
            thrusters_to_cleanup = []
            for (
                thruster_id,
                deact_time,
            ) in self.satellite.thruster_deactivation_time.items():
                if self.simulation_time - deact_time >= total_shutoff_time:
                    thrusters_to_cleanup.append(thruster_id)
            for thruster_id in thrusters_to_cleanup:
                del self.satellite.thruster_deactivation_time[thruster_id]

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
            current_position=self.satellite.position,
            current_angle=self.satellite.angle,
            current_time=self.simulation_time,
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

    def update_mpc_control(self) -> None:
        """Update control action using linearized MPC controller with strict 250ms timing."""
        # Force MPC to send commands at fixed 250ms intervals
        if self.simulation_time >= self.next_control_simulation_time:
            # Get true state
            current_state = self.get_current_state()

            # Add sensor noise if realistic physics enabled
            # This simulates what the controller actually "sees" from OptiTrack
            measured_state = self.get_noisy_state(current_state)

            # Update target state based on control mode
            # For point-to-point mode, target_state remains constant

            mpc_start_time = self.simulation_time

            start_compute_time = time.time()
            # Controller uses noisy measurements (like real hardware)
            thruster_action, mpc_info = self.mpc_controller.get_control_action(
                measured_state, self.target_state, self.previous_thrusters
            )
            end_compute_time = time.time()

            mpc_computation_time = end_compute_time - start_compute_time

            # Apply control immediately after computation
            if thruster_action is not None:
                if thruster_action.ndim == 2:
                    thruster_action = thruster_action[
                        0, :
                    ]  # Take first row, all columns

                # Ensure we have exactly 8 thrusters and convert to proper format
                if len(thruster_action) != 8:
                    logger.warning(
                        f"WARNING: Unexpected thruster array length: {len(thruster_action)}"
                    )
                    thruster_action = np.zeros(8, dtype=np.float64)

                self.set_thruster_pattern(thruster_action)
                self.previous_thrusters = thruster_action.copy()
            else:
                thruster_action = np.zeros(8, dtype=np.float64)
                self.set_thruster_pattern(thruster_action)
                logger.warning("WARNING: MPC solver failed, disabling all thrusters")

            # Record command sent time
            command_sent_time = self.simulation_time

            if self.data_save_path is not None:
                self.log_simulation_step(
                    mpc_start_time,
                    mpc_computation_time,
                    command_sent_time,
                    thruster_action,
                    mpc_info,
                )

            self.state_history.append(current_state.copy())
            self.control_history.append(thruster_action.copy())
            self.target_history.append(self.target_state.copy())
            self.mpc_solve_times.append(mpc_computation_time)
            self.mpc_info_history.append(mpc_info if mpc_info else {})

            # Schedule next control update at exactly 250ms intervals
            self.next_control_simulation_time += self.control_update_interval
            self.last_control_update = self.simulation_time

            # Verify timing constraint
            if mpc_computation_time > (self.control_update_interval - 0.02):
                logger.warning(
                    f"WARNING: MPC computation time ({mpc_computation_time:.3f}s) exceeds real-time constraint!"
                )

            # Print status with timing information
            pos_error = np.linalg.norm(current_state[:2] - self.target_state[:2])
            ang_error = abs(self.angle_difference(self.target_state[4], current_state[4]))  # type: ignore[arg-type]

            # Determine status message
            # For shape following, use DXF_SHAPE_PHASE; for waypoint mode, use target_reached_time
            stabilization_time = None  # Initialize to avoid UnboundLocalError
            shape_mode_active = getattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE", False)
            if shape_mode_active and getattr(SatelliteConfig, "DXF_SHAPE_PHASE", ""):
                phase = SatelliteConfig.DXF_SHAPE_PHASE
                # Add timing info for all Shape Following phases
                phase_time = None
                if phase == "POSITIONING" and hasattr(SatelliteConfig, 'DXF_POSITIONING_START_TIME') and SatelliteConfig.DXF_POSITIONING_START_TIME is not None:
                    phase_time = self.simulation_time - SatelliteConfig.DXF_POSITIONING_START_TIME
                elif phase == "PATH_STABILIZATION" and hasattr(SatelliteConfig, 'DXF_PATH_STABILIZATION_START_TIME') and SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME is not None:
                    phase_time = self.simulation_time - SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME
                elif phase == "TRACKING" and hasattr(SatelliteConfig, 'DXF_TRACKING_START_TIME') and SatelliteConfig.DXF_TRACKING_START_TIME is not None:
                    phase_time = self.simulation_time - SatelliteConfig.DXF_TRACKING_START_TIME
                elif phase == "RETURNING" and hasattr(SatelliteConfig, 'DXF_RETURN_START_TIME') and SatelliteConfig.DXF_RETURN_START_TIME is not None:
                    phase_time = self.simulation_time - SatelliteConfig.DXF_RETURN_START_TIME
                elif phase == "STABILIZING" and hasattr(SatelliteConfig, 'DXF_STABILIZATION_START_TIME') and SatelliteConfig.DXF_STABILIZATION_START_TIME is not None:
                    phase_time = self.simulation_time - SatelliteConfig.DXF_STABILIZATION_START_TIME

                if phase_time is not None:
                    status_msg = f"{phase} (t={phase_time:.1f}s)"
                else:
                    status_msg = phase
            else:
                # Waypoint mode status
                if self.target_reached_time is not None:
                    stabilization_time = self.simulation_time - self.target_reached_time
                    status_msg = f"STABILIZING (t={stabilization_time:.1f}s)"
                else:
                    approach_time = max(
                        self.simulation_time - self.approach_phase_start_time, 0.0
                    )
                    status_msg = f"APPROACHING (t={approach_time:.1f}s)"

            obj_value = mpc_info.get("objective_value", 0) if mpc_info else 0
            if obj_value is None:
                obj_value = 0

            if thruster_action.ndim > 1:
                display_thrusters = thruster_action[0, :]  # Take first row
            else:
                display_thrusters = thruster_action

            if len(display_thrusters) == 8:
                active_thruster_ids = [
                    int(x) for x in np.where(display_thrusters > 0.5)[0] + 1
                ]
            else:
                active_thruster_ids = []
                logger.warning(
                    f"WARNING: Invalid thruster array length for display: {len(display_thrusters)}"
                )

            # Two-line format with empty line separator
            logger.info(
                f"t={self.simulation_time:5.1f}s: {status_msg} pos_err={pos_error:.3f}m, ang_err={np.degrees(ang_error):5.1f}°\n"
                f" solve_time={mpc_computation_time:.3f}s, next_update={self.next_control_simulation_time:.3f}s,  thrusters={active_thruster_ids}\n"
            )

            # Log terminal message to CSV
            terminal_entry = {
                "Time": self.simulation_time,
                "Status": status_msg,
                "Stabilization_Time": stabilization_time if stabilization_time is not None else "",
                "Position_Error_m": pos_error,
                "Angle_Error_deg": np.degrees(ang_error),
                "Active_Thrusters": str(active_thruster_ids),
                "Solve_Time_s": mpc_computation_time,
                "Next_Update_s": self.next_control_simulation_time,
            }
            self.data_logger.log_terminal_message(terminal_entry)

    def check_target_reached(self) -> bool:
        """
        Check if satellite has reached the target within tolerances.

        Delegates to SimulationStateValidator for tolerance checking.
        """
        current_state = self.get_current_state()
        return self.state_validator.check_target_reached(
            current_state, self.target_state
        )

    def update_simulation(self, frame: int) -> List[Any]:
        """
        Update simulation step (called by matplotlib animation).

        Args:
            frame: Current frame number

        Returns:
            List of artists for matplotlib animation
        """
        if not self.is_running:
            return []

        # Handle deferred multi-point obstacle avoidance
        if hasattr(self, "next_multi_point_target") and self.next_multi_point_target:
            target_pos, target_angle = self.next_multi_point_target
            self.update_target_with_obstacle_avoidance(target_pos, target_angle)
            self.next_multi_point_target = None  # Clear the deferred target

        # Update target state based on mission mode (unified with Real.py)
        current_state = self.get_current_state()
        self.update_target_state_for_mode(current_state)

        self.update_mpc_control()

        # Process command queue to apply delayed commands
        self.process_command_queue()

        dt = self.satellite.dt

        net_force, net_torque = self.satellite.calculate_forces_and_torques()

        acceleration = net_force / self.satellite.total_mass
        self.satellite.velocity += acceleration * dt
        self.satellite.position += self.satellite.velocity * dt

        angular_acceleration = net_torque / self.satellite.moment_of_inertia
        self.satellite.angular_velocity += angular_acceleration * dt
        self.satellite.angle += self.satellite.angular_velocity * dt

        # CRITICAL: Normalize satellite angle to prevent wraparound confusion
        self.satellite.angle = self.normalize_angle(float(self.satellite.angle))

        # Apply damping
        if SatelliteConfig.USE_REALISTIC_PHYSICS:
            # Realistic damping: F_drag = -b*v, τ_drag = -c*ω
            drag_force = -SatelliteConfig.LINEAR_DAMPING_COEFF * self.satellite.velocity
            drag_accel = drag_force / self.satellite.total_mass
            self.satellite.velocity += drag_accel * dt

            drag_torque = (
                -SatelliteConfig.ROTATIONAL_DAMPING_COEFF
                * self.satellite.angular_velocity
            )
            drag_angular_accel = drag_torque / self.satellite.moment_of_inertia
            self.satellite.angular_velocity += drag_angular_accel * dt

            # Add random disturbances if enabled (air currents, vibrations, etc.)
            if SatelliteConfig.ENABLE_RANDOM_DISTURBANCES:
                disturbance_force = np.random.normal(
                    0, SatelliteConfig.DISTURBANCE_FORCE_STD, 2
                )
                self.satellite.velocity += (
                    disturbance_force / self.satellite.total_mass
                ) * dt

                disturbance_torque = np.random.normal(
                    0, SatelliteConfig.DISTURBANCE_TORQUE_STD
                )
                self.satellite.angular_velocity += (
                    disturbance_torque / self.satellite.moment_of_inertia
                ) * dt
        else:
            # Idealized: no damping (matches frictionless air table assumption)
            pass

        # Update trajectory
        self.satellite.trajectory.append(self.satellite.position.copy())
        if len(self.satellite.trajectory) > self.satellite.max_trajectory_points:
            self.satellite.trajectory.pop(0)

        # Update simulation time
        self.simulation_time += dt

        # Sync satellite's simulation time for realistic physics delays
        self.satellite.simulation_time = self.simulation_time

        if not (
            hasattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
            and SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
        ):
            target_currently_reached = self.check_target_reached()

            if target_currently_reached:
                if self.target_reached_time is None:
                    # First time reaching target
                    self.target_reached_time = self.simulation_time
                    print(
                        f"\nTARGET REACHED! Time: {self.simulation_time:.1f}s - MPC will continue to maintain position"
                    )
                else:
                    # Update maintenance tracking
                    self.target_maintenance_time = (
                        self.simulation_time - self.target_reached_time
                    )
                    current_state = self.get_current_state()
                    pos_error = np.linalg.norm(
                        current_state[:2] - self.target_state[:2]
                    )
                    ang_error = abs(self.angle_difference(self.target_state[4], current_state[4]))  # type: ignore[arg-type]
                    self.maintenance_position_errors.append(pos_error)
                    self.maintenance_angle_errors.append(ang_error)

                if (
                    hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                    and SatelliteConfig.ENABLE_WAYPOINT_MODE
                ):
                    stabilization_time = self.target_maintenance_time

                    is_final_target = (
                        SatelliteConfig.CURRENT_TARGET_INDEX
                        >= len(SatelliteConfig.WAYPOINT_TARGETS) - 1
                    )
                    if is_final_target:
                        required_hold_time = (
                            SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
                        )
                    else:
                        required_hold_time = getattr(
                            SatelliteConfig, "TARGET_HOLD_TIME", 3.0
                        )

                    if stabilization_time >= required_hold_time:
                        # Advance to next target
                        next_available = SatelliteConfig.advance_to_next_target()
                        if next_available:
                            # Update target state to next target with obstacle avoidance
                            (
                                target_pos,
                                target_angle,
                            ) = SatelliteConfig.get_current_waypoint_target()
                            if target_pos is not None:
                                if (
                                    SatelliteConfig.OBSTACLES_ENABLED
                                    and SatelliteConfig.get_obstacles()
                                ):
                                    logger.info(
                                        f"MOVING TO NEXT TARGET WITH OBSTACLE AVOIDANCE: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
                                    )
                                    if hasattr(self, "obstacle_avoiding_waypoints"):
                                        delattr(self, "obstacle_avoiding_waypoints")
                                    if hasattr(self, "current_obstacle_waypoint_idx"):
                                        delattr(self, "current_obstacle_waypoint_idx")
                                    if hasattr(self, "obstacle_waypoint_reached_time"):
                                        delattr(self, "obstacle_waypoint_reached_time")
                                    # Target will be set by update_target_with_obstacle_avoidance in next frame
                                    self.next_multi_point_target = (
                                        target_pos,
                                        target_angle,
                                    )
                                else:
                                    logger.info(
                                        f"MOVING TO NEXT TARGET: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
                                    )
                                self.target_state = np.array(
                                    [
                                        target_pos[0],
                                        target_pos[1],
                                        0.0,
                                            0.0,
                                            target_angle,
                                            0.0,
                                        ]
                                    )
                                self.target_reached_time = None
                                self.approach_phase_start_time = self.simulation_time
                                self.target_maintenance_time = 0.0
                        else:
                            # All targets completed - end simulation
                            logger.info(
                                "WAYPOINT MISSION COMPLETED! All targets visited successfully."
                            )
                            if (
                                not SatelliteConfig.USE_FINAL_STABILIZATION_IN_SIMULATION
                            ):
                                self.is_running = False
                                self.print_performance_summary()
                                return []
                            SatelliteConfig.MULTI_POINT_PHASE = "COMPLETE"  # type: ignore[assignment]
            else:
                if self.target_reached_time is not None:
                    self.times_lost_target += 1
                    print(
                        f"WARNING: Target lost at t={self.simulation_time:.1f}s - MPC working to regain control"
                    )

        if (
            not SatelliteConfig.USE_FINAL_STABILIZATION_IN_SIMULATION
            and self.target_reached_time is not None
            and not (
                hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                and SatelliteConfig.ENABLE_WAYPOINT_MODE
            )
            and not (
                hasattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
                and SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
            )
        ):
            # Mission 1: Waypoint Navigation (single waypoint) with immediate termination after target reached
            current_maintenance_time = self.simulation_time - self.target_reached_time
            if (
                current_maintenance_time
                >= SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
            ):
                print(
                    f"\n WAYPOINT MISSION COMPLETE! Stable at target for {SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME:.1f} seconds."
                )
                self.is_running = False
                self.print_performance_summary()
                return []

        if (
            SatelliteConfig.USE_FINAL_STABILIZATION_IN_SIMULATION
            and self.target_reached_time is not None
        ):
            current_maintenance_time = self.simulation_time - self.target_reached_time

            # Waypoint navigation: check completion for single or multiple waypoints
            if not (
                hasattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
                and SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
            ):
                # Single waypoint (no ENABLE_WAYPOINT_MODE set)
                if not (
                    hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                    and SatelliteConfig.ENABLE_WAYPOINT_MODE
                ):
                    if (
                        current_maintenance_time
                        >= SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
                    ):
                        print(
                            f"\n WAYPOINT MISSION COMPLETE! Stable at target for {SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME:.1f} seconds."
                        )
                        self.is_running = False
                        self.print_performance_summary()
                        return []
                # Multiple waypoints (ENABLE_WAYPOINT_MODE = True)
                elif getattr(SatelliteConfig, "MULTI_POINT_PHASE", None) == "COMPLETE":
                    if (
                        current_maintenance_time
                        >= SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME
                    ):
                        print(
                            f"\n WAYPOINT MISSION COMPLETE! All targets stable for {SatelliteConfig.WAYPOINT_FINAL_STABILIZATION_TIME:.1f} seconds."
                        )
                        self.is_running = False
                        self.print_performance_summary()
                        return []

        # Only stop simulation when max time is reached
        if self.simulation_time >= self.max_simulation_time:
            print(f"\nSIMULATION COMPLETE at {self.simulation_time:.1f}s")
            self.is_running = False
            self.print_performance_summary()

        # Redraw
        self.draw_simulation()
        self.update_mpc_info_panel()  # Use custom MPC info panel instead

        return []

    def draw_simulation(self) -> None:
        """Draw the simulation including satellite, target, and trajectory (delegated)."""
        self.visualizer.sync_from_controller()
        return self.visualizer.draw_simulation()

    def _draw_obstacles(self) -> None:
        """Draw configured obstacles on the visualization (delegated)."""
        return self.visualizer._draw_obstacles()

    def _draw_obstacle_avoidance_waypoints(self) -> None:
        """Draw obstacle avoidance waypoints for point-to-point and multi-point modes (delegated)."""
        return self.visualizer._draw_obstacle_avoidance_waypoints()

    def _draw_satellite_elements(self) -> None:
        """Draw satellite elements manually to avoid conflicts (delegated)."""
        return self.visualizer._draw_satellite_elements()

    def update_mpc_info_panel(self) -> None:
        """Update the information panel to match visualization format (delegated)."""
        self.visualizer.sync_from_controller()
        return self.visualizer.update_mpc_info_panel()

    def print_performance_summary(self) -> None:
        """Print performance summary at the end of simulation (delegated)."""
        self.visualizer.sync_from_controller()
        return self.visualizer.print_performance_summary()

    def reset_simulation(self) -> None:
        """Reset simulation to initial state (delegated)."""
        self.visualizer.sync_from_controller()
        return self.visualizer.reset_simulation()

    def auto_generate_visualizations(self) -> None:
        """Automatically generate all visualizations after simulation completion (delegated)."""
        self.visualizer.sync_from_controller()
        return self.visualizer.auto_generate_visualizations()

    def _run_simulation_with_globals(self, show_animation: bool = True) -> None:
        """
        Run linearized MPC simulation.

        Args:
            show_animation: Whether to display animation during simulation
        """
        print("\nStarting Linearized MPC Simulation...")
        print("Press 'q' to quit early, Space to pause/resume")
        self.is_running = True

        # Clear any previous data from the logger
        self.data_logger.clear_logs()

        self.data_save_path = self.create_data_directories()
        self.data_logger.set_save_path(self.data_save_path)
        print(f" Data will be saved to: {self.data_save_path}")

        try:
            if show_animation:
                fig = self.satellite.fig
                ani = FuncAnimation(
                    fig,
                    self.update_simulation,
                    interval=int(self.satellite.dt * 1000),
                    blit=False,
                    repeat=False,
                    cache_frame_data=False,
                )
                plt.show()  # Show the animation window live

                # After animation is complete, save files
                if self.data_save_path is not None:
                    print("\nSaving simulation data...")
                    self.save_csv_data()
                    self.visualizer.sync_from_controller()
                    self.visualizer.save_mission_summary()
                    self.save_mission_summary()
                    self.save_animation_mp4(fig, ani)
                    print(f" Data saved to: {self.data_save_path}")

                    print("\n Auto-generating performance plots...")
                    self.auto_generate_visualizations()
                    print(" All visualizations complete!")
            else:
                # Run without animation
                while self.is_running:
                    self.update_simulation(None)  # type: ignore[arg-type]
                    if not self.is_running:
                        break

                if self.data_save_path is not None:
                    print("\nSaving simulation data...")
                    self.save_csv_data()
                    self.visualizer.sync_from_controller()
                    self.visualizer.save_mission_summary()
                    self.save_mission_summary()
                    print(f" CSV data saved to: {self.data_save_path}")

                    # Auto-generate all visualizations
                    print("\n Auto-generating visualizations...")
                    self.auto_generate_visualizations()
                    print(" All visualizations complete!")

        except KeyboardInterrupt:
            print("\n\nSimulation cancelled by user")
            self.is_running = False

            # Save data when interrupted
            if self.data_save_path is not None and self.data_logger.get_log_count() > 0:
                print("\nSaving simulation data...")
                self.save_csv_data()
                self.visualizer.sync_from_controller()
                self.visualizer.save_mission_summary()
                self.save_mission_summary()
                print(f" Data saved to: {self.data_save_path}")

                # Try to generate visualizations if we have enough data
                if self.data_logger.get_log_count() > 10:
                    try:
                        print("\n Auto-generating visualizations...")
                        self.auto_generate_visualizations()
                        print(" All visualizations complete!")
                    except Exception as e:
                        logger.warning(
                            f"WARNING: Could not generate visualizations: {e}"
                        )

        finally:
            # Cleanup
            pass
        return self.data_save_path  # type: ignore[return-value]

    def run_simulation(self, show_animation: bool = True) -> None:
        """
        Run simulation with a per-instance structured config sandbox.

        Args:
            show_animation: Whether to display animation during simulation
        """
        with use_structured_config(self.structured_config.clone()):
            return self._run_simulation_with_globals(show_animation=show_animation)


if __name__ == "__main__":
    from simulation_test_modes import main

    main()
