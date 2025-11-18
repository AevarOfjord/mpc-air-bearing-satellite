"""
Mission Report Generator for Satellite Control System

Generates comprehensive post-mission analysis reports with detailed metrics and statistics.
Provides formatted text reports for documentation and performance review.

Report sections:
- Mission configuration and parameters
- Performance metrics and analysis
- Target tracking results with error statistics
- Control system performance (thruster usage, control effort)
- MPC timing statistics and computational performance
- Collision avoidance and safety events

Output formats:
- Console display with formatted text
- Text file export for archival
- Summary statistics for comparison
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, List, Optional

import numpy as np

from config import SatelliteConfig

logger = logging.getLogger(__name__)


class MissionReportGenerator:
    """
    Generates comprehensive mission summary reports with configuration and metrics.

    Creates detailed text reports including:
    - Mission type and configuration
    - Controller parameters
    - Physical parameters
    - Performance metrics (position, orientation, control effort)
    - MPC timing analysis
    - Target maintenance statistics
    """

    def __init__(self, config_obj: Any):
        """
        Initialize report generator.

        Args:
            config_obj: Configuration object (SatelliteConfig)
        """
        self.config = config_obj

    def generate_report(
        self,
        output_path: Path,
        state_history: List[np.ndarray],
        target_state: np.ndarray,
        control_time: float,
        mpc_solve_times: List[float],
        control_history: List[np.ndarray],
        target_reached_time: Optional[float],
        target_maintenance_time: float,
        times_lost_target: int,
        maintenance_position_errors: List[float],
        maintenance_angle_errors: List[float],
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        angle_difference_func: Callable[..., Any],
        check_target_reached_func: Callable[..., Any],
        motive_server_url: str = "N/A",
        rigid_body_name: str = "N/A",
        test_mode: str = "REAL HARDWARE TEST",
    ) -> None:
        """
        Generate comprehensive mission summary report.

        Args:
            output_path: Path to save the report
            state_history: List of state vectors [x, y, vx, vy, yaw, omega]
            target_state: Target state vector
            control_time: Total mission duration in seconds
            mpc_solve_times: List of MPC solve times
            control_history: List of control vectors (thruster commands)
            target_reached_time: Time when target was first reached (None if never)
            target_maintenance_time: Time spent maintaining target
            times_lost_target: Number of times target was lost
            maintenance_position_errors: Position errors during maintenance
            maintenance_angle_errors: Angle errors during maintenance
            position_tolerance: Position tolerance threshold
            angle_tolerance: Angle tolerance threshold
            control_update_interval: Control loop update interval
            angle_difference_func: Function to calculate angle difference
            check_target_reached_func: Function to check if target was reached
            motive_server_url: OptiTrack server URL
            rigid_body_name: Rigid body name in OptiTrack
            test_mode: Test mode description
        """
        if not state_history:
            logger.warning(
                "WARNING: Cannot generate report: No state history available"
            )
            return

        try:
            with open(output_path, "w") as f:
                # Header
                self._write_header(f, output_path, test_mode)

                # Mission Configuration
                self._write_mission_configuration(f, state_history, target_state)

                # Controller Configuration
                self._write_controller_configuration(f)

                # Physical Parameters
                self._write_physical_parameters(f)

                # Hardware Settings
                self._write_hardware_settings(f, motive_server_url, rigid_body_name)

                # All Configuration Parameters (for test comparison)
                self._write_all_config_parameters(f)

                # Performance Results
                self._write_performance_results(
                    f,
                    state_history,
                    target_state,
                    control_time,
                    mpc_solve_times,
                    control_history,
                    target_reached_time,
                    target_maintenance_time,
                    times_lost_target,
                    maintenance_position_errors,
                    maintenance_angle_errors,
                    position_tolerance,
                    angle_tolerance,
                    control_update_interval,
                    angle_difference_func,
                    check_target_reached_func,
                )

                # Footer
                f.write("\n" + "=" * 80 + "\n")
                f.write("END OF MISSION SUMMARY\n")
                f.write("=" * 80 + "\n")

            logger.info(f"Mission summary saved to: {output_path}")
            print(f" Mission summary saved: {output_path}")

        except Exception as e:
            logger.error(f"ERROR: Error generating mission report: {e}")

    def _write_header(self, f, output_path: Path, test_mode: str) -> None:
        """Write report header."""
        f.write("=" * 80 + "\n")
        f.write("SATELLITE CONTROL SYSTEM - MISSION SUMMARY & CONFIGURATION\n")
        f.write("=" * 80 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Data Directory: {output_path.parent}\n")
        f.write(f"Test Mode: {test_mode}\n")
        f.write("=" * 80 + "\n\n")

    def _write_mission_configuration(
        self, f, state_history: List[np.ndarray], target_state: np.ndarray
    ) -> None:
        """Write mission configuration section."""
        f.write("=" * 80 + "\n")
        f.write("MISSION CONFIGURATION - RECREATION DATA\n")
        f.write("=" * 80 + "\n")
        f.write(
            "This section contains all information needed to recreate this exact mission.\n\n"
        )

        initial_state = state_history[0]

        # Determine mission type
        if (
            hasattr(self.config, "DXF_SHAPE_MODE_ACTIVE")
            and self.config.DXF_SHAPE_MODE_ACTIVE
        ):
            self._write_profile_following_config(f, initial_state)
        elif (
            hasattr(self.config, "ENABLE_WAYPOINT_MODE")
            and self.config.ENABLE_WAYPOINT_MODE
        ):
            self._write_waypoint_config(f, initial_state)
        else:
            self._write_point_to_point_config(f, initial_state, target_state)

        # Obstacle configuration
        self._write_obstacle_configuration(f)

    def _write_profile_following_config(self, f, initial_state: np.ndarray) -> None:
        """Write profile following (Mode 3) configuration."""
        f.write("MISSION TYPE: PROFILE FOLLOWING (MODE 3)\n")
        f.write("-" * 50 + "\n")
        f.write(
            "Description: Satellite follows a moving target along a custom path profile\n"
        )
        f.write("Mission Phases:\n")
        f.write(
            "  Phase 1: Move to closest point on profile and stabilize (3 seconds)\n"
        )
        f.write("  Phase 2: Track moving target along the path profile\n")
        f.write("  Phase 3: Stabilize at completion point (5 seconds)\n\n")

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n")

        f.write("SHAPE CONFIGURATION:\n")
        if hasattr(self.config, "DXF_SHAPE_CENTER") and self.config.DXF_SHAPE_CENTER:
            f.write(
                f"  Shape Center X:          {self.config.DXF_SHAPE_CENTER[0]:.3f} m\n"
            )
            f.write(
                f"  Shape Center Y:          {self.config.DXF_SHAPE_CENTER[1]:.3f} m\n"
            )
        if hasattr(self.config, "DXF_SHAPE_ROTATION"):
            f.write(
                f"  Shape Rotation:          {np.degrees(self.config.DXF_SHAPE_ROTATION):.1f}°\n"
            )
        if hasattr(self.config, "DXF_OFFSET_DISTANCE"):
            f.write(
                f"  Offset Distance:         {self.config.DXF_OFFSET_DISTANCE:.3f} m\n"
            )
        if hasattr(self.config, "DXF_PATH_LENGTH"):
            f.write(f"  Path Length:             {self.config.DXF_PATH_LENGTH:.3f} m\n")
        if hasattr(self.config, "DXF_BASE_SHAPE") and self.config.DXF_BASE_SHAPE:
            path_len = (
                len(self.config.DXF_SHAPE_PATH)
                if hasattr(self.config, "DXF_SHAPE_PATH") and self.config.DXF_SHAPE_PATH
                else 0
            )
            f.write(f"  Number of Path Points:   {path_len}\n\n")

        f.write("MOVING TARGET CONFIGURATION:\n")
        if hasattr(self.config, "DXF_TARGET_SPEED"):
            f.write(
                f"  Target Speed:            {self.config.DXF_TARGET_SPEED:.3f} m/s\n"
            )
        if hasattr(self.config, "DXF_ESTIMATED_DURATION"):
            f.write(
                f"  Estimated Duration:      {self.config.DXF_ESTIMATED_DURATION:.1f} s\n\n"
            )

    def _write_waypoint_config(self, f, initial_state: np.ndarray) -> None:
        """Write waypoint (Mode 1) configuration."""
        f.write("MISSION TYPE: WAYPOINT NAVIGATION (MODE 1)\n")
        f.write("-" * 50 + "\n")
        f.write("Description: Satellite visits multiple waypoints in sequence\n\n")

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n")

        f.write("WAYPOINTS:\n")
        f.write(f"  Number of Waypoints:     {len(self.config.WAYPOINT_TARGETS)}\n")
        for i, ((tx, ty), ta) in enumerate(
            zip(self.config.WAYPOINT_TARGETS, self.config.WAYPOINT_ANGLES), 1
        ):
            f.write(
                f"  Waypoint {i}:              ({tx:.3f}, {ty:.3f}) m, {np.degrees(ta):.1f}°\n"
            )
        f.write(
            f"\n  Hold Time per Waypoint:  {self.config.TARGET_HOLD_TIME:.1f} s\n\n"
        )

    def _write_point_to_point_config(
        self, f, initial_state: np.ndarray, target_state: np.ndarray
    ) -> None:
        """Write point-to-point (Mode 1) configuration."""
        f.write("MISSION TYPE: POINT-TO-POINT (MODE 1)\n")
        f.write("-" * 50 + "\n")
        f.write(
            "Description: Satellite navigates to target position and orientation\n\n"
        )

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n")

        target_pos = target_state[:2]
        target_angle = target_state[4]
        f.write("TARGET CONFIGURATION:\n")
        f.write(f"  Target X position:       {target_pos[0]:.3f} m\n")
        f.write(f"  Target Y position:       {target_pos[1]:.3f} m\n")
        f.write(f"  Target orientation:      {np.degrees(target_angle):.1f}°\n\n")

    def _write_obstacle_configuration(self, f) -> None:
        """Write obstacle configuration."""
        if self.config.OBSTACLES_ENABLED and self.config.OBSTACLES:
            f.write("OBSTACLE CONFIGURATION:\n")
            f.write("  Obstacle Avoidance:      ENABLED\n")
            f.write(f"  Number of Obstacles:     {len(self.config.OBSTACLES)}\n")
            for i, (ox, oy, orad) in enumerate(self.config.OBSTACLES, 1):
                f.write(
                    f"  Obstacle {i}:              ({ox:.3f}, {oy:.3f}) m, radius {orad:.3f} m\n"
                )
            f.write("\n")
        else:
            f.write("OBSTACLE CONFIGURATION:\n")
            f.write("  Obstacle Avoidance:      DISABLED\n\n")

    def _write_controller_configuration(self, f) -> None:
        """Write controller configuration section."""
        f.write("=" * 80 + "\n")
        f.write("CONTROLLER CONFIGURATION\n")
        f.write("=" * 80 + "\n")
        f.write("These parameters affect mission performance and control behavior.\n\n")

        f.write("MPC CONTROLLER PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write("  Controller Type:         Linearized MPC (A*x[k] + B*u[k])\n")
        f.write(f"  Solver:                  {self.config.MPC_SOLVER_TYPE}\n")
        f.write(
            f"  Prediction Horizon:      {self.config.MPC_PREDICTION_HORIZON} steps\n"
        )
        f.write(f"  Control Horizon:         {self.config.MPC_CONTROL_HORIZON} steps\n")
        f.write(f"  Simulation Timestep:     {self.config.SIMULATION_DT:.3f} s\n")
        f.write(f"  Control Timestep:        {self.config.CONTROL_DT:.3f} s\n")
        f.write(
            f"  Solver Time Limit:       {self.config.MPC_SOLVER_TIME_LIMIT:.3f} s\n\n"
        )

        f.write("COST FUNCTION WEIGHTS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Position Weight (Q):     {self.config.Q_POSITION:.1f}\n")
        f.write(f"  Velocity Weight (Q):     {self.config.Q_VELOCITY:.1f}\n")
        f.write(f"  Angle Weight (Q):        {self.config.Q_ANGLE:.1f}\n")
        f.write(f"  Angular Vel Weight (Q):  {self.config.Q_ANGULAR_VELOCITY:.1f}\n")
        f.write(f"  Thrust Penalty (R):      {self.config.R_THRUST:.3f}\n")
        f.write(f"  Switch Penalty (R):      {self.config.R_SWITCH:.3f}\n\n")

        f.write("CONTROL CONSTRAINTS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Max Velocity:            {self.config.MAX_VELOCITY:.3f} m/s\n")
        f.write(
            f"  Max Angular Velocity:    {np.degrees(self.config.MAX_ANGULAR_VELOCITY):.1f}°/s\n"
        )
        f.write(f"  Position Bounds:         ±{self.config.POSITION_BOUNDS:.1f} m\n\n")

        f.write("TOLERANCE THRESHOLDS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Position Tolerance:      {self.config.POSITION_TOLERANCE:.3f} m\n")
        f.write(
            f"  Angle Tolerance:         <{np.degrees(self.config.ANGLE_TOLERANCE):.1f}°\n"
        )
        f.write(
            f"  Velocity Tolerance:      {self.config.VELOCITY_TOLERANCE:.3f} m/s\n"
        )
        f.write(
            f"  Angular Vel Tolerance:   <{np.degrees(self.config.ANGULAR_VELOCITY_TOLERANCE):.1f}°/s\n\n"
        )

    def _write_physical_parameters(self, f) -> None:
        """Write physical parameters section."""
        f.write("PHYSICAL PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Total Mass:              {self.config.TOTAL_MASS:.3f} kg\n")
        f.write(
            f"  Moment of Inertia:       {self.config.MOMENT_OF_INERTIA:.6f} kg·m²\n"
        )
        f.write(f"  Satellite Size:          {self.config.SATELLITE_SIZE:.3f} m\n")
        f.write(
            f"  COM Offset:              ({self.config.COM_OFFSET[0]:.6f}, {self.config.COM_OFFSET[1]:.6f}) m\n\n"
        )

        f.write("THRUSTER FORCES:\n")
        for tid in range(1, 9):
            f.write(
                f"  Thruster {tid}:             {self.config.THRUSTER_FORCES[tid]:.6f} N\n"
            )
        f.write("\n")

    def _write_hardware_settings(
        self, f, motive_server_url: str, rigid_body_name: str
    ) -> None:
        """Write hardware interface settings."""
        f.write("HARDWARE INTERFACE SETTINGS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Serial Port:             {self.config.SERIAL_PORT}\n")
        f.write(f"  Baud Rate:               {self.config.SERIAL_BAUDRATE}\n")
        f.write(f"  Motive Server URL:       {motive_server_url}\n")
        f.write(f"  Rigid Body Name:         {rigid_body_name}\n\n")

    def _write_all_config_parameters(self, f) -> None:
        """Write comprehensive all configuration parameters section."""
        f.write("=" * 80 + "\n")
        f.write("ALL CONFIGURATION PARAMETERS (FOR TEST COMPARISON)\n")
        f.write("=" * 80 + "\n")
        f.write(
            "Complete listing of all system parameters for easy test-to-test comparison.\n\n"
        )

        # MPC Parameters
        f.write("MPC PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  MPC_PREDICTION_HORIZON:        {self.config.MPC_PREDICTION_HORIZON}\n"
        )
        f.write(f"  MPC_CONTROL_HORIZON:           {self.config.MPC_CONTROL_HORIZON}\n")
        f.write(
            f"  MPC_SOLVER_TIME_LIMIT:         {self.config.MPC_SOLVER_TIME_LIMIT:.3f} s\n"
        )
        f.write(f"  MPC_SOLVER_TYPE:               {self.config.MPC_SOLVER_TYPE}\n")
        f.write(f"  Q_POSITION:                    {self.config.Q_POSITION:.1f}\n")
        f.write(f"  Q_VELOCITY:                    {self.config.Q_VELOCITY:.1f}\n")
        f.write(f"  Q_ANGLE:                       {self.config.Q_ANGLE:.1f}\n")
        f.write(
            f"  Q_ANGULAR_VELOCITY:            {self.config.Q_ANGULAR_VELOCITY:.1f}\n"
        )
        f.write(f"  R_THRUST:                      {self.config.R_THRUST:.3f}\n")
        f.write(f"  R_SWITCH:                      {self.config.R_SWITCH:.3f}\n")
        f.write(
            f"  MAX_VELOCITY:                  {self.config.MAX_VELOCITY:.3f} m/s\n"
        )
        f.write(
            f"  MAX_ANGULAR_VELOCITY:          {np.degrees(self.config.MAX_ANGULAR_VELOCITY):.1f}°/s\n"
        )
        f.write(
            f"  POSITION_BOUNDS:               ±{self.config.POSITION_BOUNDS:.1f} m\n"
        )
        f.write(f"  DAMPING_ZONE:                  {self.config.DAMPING_ZONE:.3f} m\n")
        f.write(
            f"  VELOCITY_THRESHOLD:            {self.config.VELOCITY_THRESHOLD:.3f} m/s\n"
        )
        f.write(
            f"  MAX_VELOCITY_WEIGHT:           {self.config.MAX_VELOCITY_WEIGHT:.1f}\n"
        )
        f.write(
            f"  ADAPTIVE_HORIZONS_ENABLED:     {self.config.ADAPTIVE_HORIZONS_ENABLED}\n"
        )
        f.write(
            f"  POSITION_TOLERANCE:            {self.config.POSITION_TOLERANCE:.3f} m\n"
        )
        f.write(
            f"  ANGLE_TOLERANCE:               {np.degrees(self.config.ANGLE_TOLERANCE):.1f}°\n"
        )
        f.write(
            f"  VELOCITY_TOLERANCE:            {self.config.VELOCITY_TOLERANCE:.3f} m/s\n"
        )
        f.write(
            f"  ANGULAR_VELOCITY_TOLERANCE:    {np.degrees(self.config.ANGULAR_VELOCITY_TOLERANCE):.1f}°/s\n\n"
        )

        # Timing Parameters
        f.write("TIMING PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  SIMULATION_DT:                 {self.config.SIMULATION_DT:.3f} s\n")
        f.write(f"  CONTROL_DT:                    {self.config.CONTROL_DT:.3f} s\n")
        f.write(
            f"  MAX_SIMULATION_TIME:           {self.config.MAX_SIMULATION_TIME:.1f} s\n"
        )
        f.write(
            f"  TARGET_HOLD_TIME:              {self.config.TARGET_HOLD_TIME:.1f} s\n"
        )
        # Use WAYPOINT_FINAL_STABILIZATION_TIME as fallback for mission-specific stabilization times
        waypoint_final_stab = self.config.WAYPOINT_FINAL_STABILIZATION_TIME
        f.write(
            f"  POINT_TO_POINT_FINAL_STAB:    {getattr(self.config, 'POINT_TO_POINT_FINAL_STABILIZATION_TIME', waypoint_final_stab):.1f} s\n"
        )
        f.write(
            f"  MULTI_POINT_FINAL_STAB:       {getattr(self.config, 'MULTI_POINT_FINAL_STABILIZATION_TIME', waypoint_final_stab):.1f} s\n"
        )
        f.write(
            f"  DXF_FINAL_STAB:                {getattr(self.config, 'DXF_FINAL_STABILIZATION_TIME', waypoint_final_stab):.1f} s\n\n"
        )

        # Physics Parameters
        f.write("PHYSICS PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  TOTAL_MASS:                    {self.config.TOTAL_MASS:.3f} kg\n")
        f.write(
            f"  MOMENT_OF_INERTIA:             {self.config.MOMENT_OF_INERTIA:.6f} kg·m²\n"
        )
        f.write(
            f"  SATELLITE_SIZE:                {self.config.SATELLITE_SIZE:.3f} m\n"
        )
        f.write(
            f"  LINEAR_DAMPING:                {self.config.LINEAR_DAMPING:.3f} N/(m/s)\n"
        )
        f.write(
            f"  ROTATIONAL_DAMPING:            {self.config.ROTATIONAL_DAMPING:.4f} N·m/(rad/s)\n"
        )
        f.write(
            f"  THRUSTER_VALVE_DELAY:          {self.config.THRUSTER_VALVE_DELAY * 1000:.1f} ms\n"
        )
        f.write(
            f"  THRUSTER_RAMPUP_TIME:          {self.config.THRUSTER_RAMPUP_TIME * 1000:.1f} ms\n"
        )
        f.write(
            f"  THRUST_FORCE_NOISE_PERCENT:    {self.config.THRUST_FORCE_NOISE_PERCENT:.1f}%\n\n"
        )

        # Sensor Noise Parameters
        f.write("SENSOR NOISE PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  POSITION_NOISE_STD:            {self.config.POSITION_NOISE_STD * 1000:.2f} mm\n"
        )
        f.write(
            f"  ANGLE_NOISE_STD:               {np.degrees(self.config.ANGLE_NOISE_STD):.2f}°\n"
        )
        f.write(
            f"  VELOCITY_NOISE_STD:            {self.config.VELOCITY_NOISE_STD * 1000:.2f} mm/s\n"
        )
        f.write(
            f"  ANGULAR_VEL_NOISE_STD:         {np.degrees(self.config.ANGULAR_VEL_NOISE_STD):.2f}°/s\n\n"
        )

        # Disturbance Parameters
        f.write("DISTURBANCE PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  RANDOM_DISTURBANCES_ENABLED:   {self.config.RANDOM_DISTURBANCES_ENABLED}\n"
        )
        f.write(
            f"  DISTURBANCE_FORCE_STD:         {self.config.DISTURBANCE_FORCE_STD:.3f} N\n"
        )
        f.write(
            f"  DISTURBANCE_TORQUE_STD:        {self.config.DISTURBANCE_TORQUE_STD:.4f} N·m\n\n"
        )

        # Communication Parameters
        f.write("COMMUNICATION PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  SERIAL_PORT:                   {self.config.SERIAL_PORT}\n")
        f.write(f"  SERIAL_BAUDRATE:               {self.config.SERIAL_BAUDRATE}\n")
        f.write(
            f"  SERIAL_TIMEOUT:                {self.config.SERIAL_TIMEOUT:.1f} s\n"
        )
        f.write(f"  SATELLITE_UDP_PORT:            {self.config.SATELLITE_UDP_PORT}\n")
        f.write(f"  MOTIVE_SERVER_URL:             {self.config.MOTIVE_SERVER_URL}\n")
        f.write(
            f"  MOTIVE_TIMEOUT:                {self.config.MOTIVE_TIMEOUT:.1f} s\n\n"
        )

        # Optional/Feature Flags
        f.write("FEATURE FLAGS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  REALISTIC_PHYSICS_ENABLED:     {self.config.REALISTIC_PHYSICS_ENABLED}\n"
        )
        f.write(
            f"  USE_FINAL_STABILIZATION:       {self.config.USE_FINAL_STABILIZATION_IN_SIMULATION}\n"
        )
        f.write(f"  HEADLESS_MODE:                 {self.config.HEADLESS_MODE}\n\n")

    def _write_performance_results(
        self,
        f,
        state_history: List[np.ndarray],
        target_state: np.ndarray,
        control_time: float,
        mpc_solve_times: List[float],
        control_history: List[np.ndarray],
        target_reached_time: Optional[float],
        target_maintenance_time: float,
        times_lost_target: int,
        maintenance_position_errors: List[float],
        maintenance_angle_errors: List[float],
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        angle_difference_func: Callable[..., Any],
        check_target_reached_func: Callable[..., Any],
    ) -> None:
        """Write performance results section."""
        f.write("=" * 80 + "\n")
        f.write("MISSION PERFORMANCE RESULTS\n")
        f.write("=" * 80 + "\n\n")

        # Calculate metrics
        initial_state = state_history[0]
        final_state = state_history[-1]
        initial_pos = initial_state[:2]
        final_pos = final_state[:2]
        target_pos = target_state[:2]

        pos_error_initial = np.linalg.norm(initial_pos - target_pos)
        pos_error_final = np.linalg.norm(final_pos - target_pos)
        ang_error_initial = abs(
            angle_difference_func(target_state[4], initial_state[4])
        )
        ang_error_final = abs(angle_difference_func(target_state[4], final_state[4]))

        trajectory_distance = sum(
            np.linalg.norm(state_history[i][:2] - state_history[i - 1][:2])
            for i in range(1, len(state_history))
        )

        mpc_convergence_times = (
            np.array(mpc_solve_times) if mpc_solve_times else np.array([])
        )

        total_thrust_activations = (
            sum(np.sum(control) for control in control_history)
            if control_history
            else 0
        )
        total_thrust_magnitude = (
            sum(np.linalg.norm(control) for control in control_history)
            if control_history
            else 0
        )

        switching_events = 0
        if len(control_history) > 1:
            for i in range(1, len(control_history)):
                curr_control = control_history[i]
                prev_control = control_history[i - 1]
                if len(curr_control) < 8:
                    curr_control = np.pad(
                        curr_control, (0, 8 - len(curr_control)), "constant"
                    )
                if len(prev_control) < 8:
                    prev_control = np.pad(
                        prev_control, (0, 8 - len(prev_control)), "constant"
                    )
                switching_events += np.sum(np.abs(curr_control - prev_control))

        success = check_target_reached_func()
        vel_magnitude_final = np.linalg.norm(final_state[2:4])

        # Position & Trajectory Analysis
        f.write("[POSITION] POSITION & TRAJECTORY ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"Initial Position:          ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}) m\n"
        )
        f.write(
            f"Final Position:            ({final_pos[0]:.3f}, {final_pos[1]:.3f}) m\n"
        )
        f.write(
            f"Target Position:           ({target_pos[0]:.3f}, {target_pos[1]:.3f}) m\n"
        )
        f.write(f"Initial Position Error:    {pos_error_initial:.4f} m\n")
        f.write(f"Final Position Error:      {pos_error_final:.4f} m\n")
        if pos_error_initial > 0:
            f.write(
                f"Position Improvement:      {((pos_error_initial - pos_error_final) / pos_error_initial * 100):.1f}%\n"
            )
        f.write(f"Total Distance Traveled:   {trajectory_distance:.3f} m\n")
        if trajectory_distance > 0:
            f.write(
                f"Trajectory Efficiency:     {(np.linalg.norm(target_pos - initial_pos) / trajectory_distance * 100):.1f}%\n"
            )
        f.write("\n")

        # Orientation Analysis
        f.write("ORIENTATION ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Initial Angle Error:       {np.degrees(ang_error_initial):.2f}°\n")
        f.write(
            f"Final Angle Error:         {np.degrees(ang_error_final):.2f}° (target: <{np.degrees(angle_tolerance):.1f}°)\n"
        )
        if ang_error_initial > 0:
            f.write(
                f"Angle Improvement:         {((ang_error_initial - ang_error_final) / ang_error_initial * 100):.1f}%\n"
            )
        f.write(f"Final Velocity Magnitude:  {vel_magnitude_final:.4f} m/s\n")
        f.write(f"Final Angular Velocity:    {np.degrees(final_state[5]):.2f}°/s\n\n")

        # MPC Performance
        f.write("LINEARIZED MPC CONTROLLER PERFORMANCE\n")
        f.write("-" * 50 + "\n")
        f.write(f"Total Test Time:           {control_time:.1f} s\n")
        f.write(f"MPC Updates:               {len(mpc_convergence_times)} cycles\n")
        if control_time > 0:
            f.write(
                f"MPC Update Rate:           {len(mpc_convergence_times) / control_time:.1f} Hz\n"
            )

        if len(mpc_convergence_times) > 0:
            f.write(
                f"Fastest MPC Solve:         {np.min(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"Slowest MPC Solve:         {np.max(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"Average MPC Solve:         {np.mean(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"MPC Solve Std Dev:         {np.std(mpc_convergence_times):.3f} s\n"
            )
            timing_violations = sum(
                1 for t in mpc_convergence_times if t > (control_update_interval - 0.02)
            )
            f.write(
                f"Timing Violations:         {timing_violations}/{len(mpc_convergence_times)} ({100 * timing_violations / len(mpc_convergence_times):.1f}%)\n"
            )
            f.write(
                f"Real-time Performance:     {(np.mean(mpc_convergence_times) / control_update_interval * 100):.1f}% of available time\n"
            )
        f.write("\n")

        # Control Effort Analysis
        if control_history:
            f.write("CONTROL EFFORT & FUEL ANALYSIS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Total Thruster Activations: {total_thrust_activations:.0f}\n")
            f.write(f"Total Control Magnitude:    {total_thrust_magnitude:.2f} N·s\n")
            f.write(
                f"Average Control per Step:   {total_thrust_magnitude / len(control_history):.3f} N\n"
            )
            f.write(f"Thruster Switching Events:  {switching_events:.0f}\n")
            if len(control_history) > 0:
                f.write(
                    f"Control Smoothness:         {(1 - switching_events / (len(control_history) * 8)) * 100:.1f}%\n"
                )
            if trajectory_distance > 0:
                f.write(
                    f"Fuel Efficiency:            {total_thrust_magnitude / trajectory_distance:.3f} N·s/m\n"
                )
            f.write("\n")

        # Mission Success Analysis
        f.write("MISSION SUCCESS & TARGET MAINTENANCE ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Position Tolerance:        <{position_tolerance:.3f} m\n")
        f.write(f"Angle Tolerance:           <{np.degrees(angle_tolerance):.1f}°\n")
        f.write(
            f"Position Target Met:       {'YES' if pos_error_final < position_tolerance else 'NO'}\n"
        )
        f.write(
            f"Angle Target Met:          {'YES' if ang_error_final < angle_tolerance else 'NO'}\n"
        )
        f.write(
            f"Overall Mission Status:    {'SUCCESS' if success else 'INCOMPLETE'}\n\n"
        )

        # Precision Analysis
        precision_target_met = pos_error_final <= SatelliteConfig.PRECISION_TARGET_THRESHOLD
        if precision_target_met:
            f.write(
                f"PRECISION REQUIREMENT MET: Final position error {pos_error_final:.4f}m <= {SatelliteConfig.PRECISION_TARGET_THRESHOLD:.3f}m \n"
            )
        else:
            f.write(
                f"PRECISION REQUIREMENT NOT MET: Final position error {pos_error_final:.4f}m > {SatelliteConfig.PRECISION_TARGET_THRESHOLD:.3f}m \n"
            )

        f.write("\nPRECISION ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Target Precision:          {SatelliteConfig.PRECISION_TARGET_THRESHOLD:.3f}m ({SatelliteConfig.PRECISION_TARGET_THRESHOLD*100:.1f}cm)\n")
        f.write(f"Achieved Precision:        {pos_error_final:.4f}m\n")
        precision_ratio = pos_error_final / SatelliteConfig.PRECISION_TARGET_THRESHOLD
        if precision_ratio <= 1.0:
            f.write(f"Precision Ratio:           {precision_ratio:.2f} (PASSED)\n")
        else:
            f.write(
                f"Precision Ratio:           {precision_ratio:.2f} (FAILED - {((precision_ratio - 1) * 100):.1f}% over target)\n"
            )
        f.write("\n")

        # Target Maintenance Analysis
        if target_reached_time is not None:
            f.write("TARGET MAINTENANCE ANALYSIS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Target First Reached:      {target_reached_time:.1f} s\n")
            f.write(
                f"Target Maintenance Time:   {target_maintenance_time:.1f} s ({(target_maintenance_time / control_time * 100):.1f}% of total)\n"
            )
            f.write(f"Times Lost Target:         {times_lost_target}\n")

            if maintenance_position_errors:
                avg_maintenance_pos_err = np.mean(maintenance_position_errors)
                max_maintenance_pos_err = np.max(maintenance_position_errors)
                avg_maintenance_ang_err = np.mean(maintenance_angle_errors)
                max_maintenance_ang_err = np.max(maintenance_angle_errors)

                f.write(f"Avg Maintenance Pos Error: {avg_maintenance_pos_err:.4f} m\n")
                f.write(f"Max Maintenance Pos Error: {max_maintenance_pos_err:.4f} m\n")
                f.write(
                    f"Avg Maintenance Ang Error: {np.degrees(avg_maintenance_ang_err):.2f}°\n"
                )
                f.write(
                    f"Max Maintenance Ang Error: {np.degrees(max_maintenance_ang_err):.2f}°\n"
                )

                pos_stability = 100 * (1 - avg_maintenance_pos_err / position_tolerance)
                ang_stability = 100 * (1 - avg_maintenance_ang_err / angle_tolerance)
                f.write(
                    f"Position Stability:        {pos_stability:.1f}% (within tolerance)\n"
                )
                f.write(
                    f"Angle Stability:           {ang_stability:.1f}% (within tolerance)\n"
                )
        else:
            f.write("TARGET STATUS\n")
            f.write("-" * 50 + "\n")
            f.write("Target Never Reached:      Mission incomplete\n")
            remaining_pos_error = max(
                0.0, float(pos_error_final - position_tolerance)
            )
            remaining_ang_error = max(
                0.0, float(ang_error_final - angle_tolerance)
            )
            f.write(f"Remaining Position Error:  {remaining_pos_error:.4f} m\n")
            f.write(
                f"Remaining Angle Error:     {np.degrees(remaining_ang_error):.2f}°\n"
            )


def create_mission_report_generator(config_obj: Any) -> MissionReportGenerator:
    """
    Factory function to create a mission report generator.

    Args:
        config_obj: Configuration object (SatelliteConfig)

    Returns:
        Configured MissionReportGenerator instance
    """
    return MissionReportGenerator(config_obj)
