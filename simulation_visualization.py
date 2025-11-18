"""
Simulation Visualization Manager for Satellite Control System

Handles matplotlib-based visualization and animation for simulation results.
Provides real-time drawing and high-quality MP4 animation generation.

Visualization components:
- Satellite body with thruster indicators
- Target positions and waypoints
- Trajectory paths with color-coded segments
- Circular obstacles with safety margins
- Thruster activation indicators
- Real-time state information display

Animation capabilities:
- Frame-by-frame simulation playback
- High-quality MP4 video export
- Configurable frame rate and resolution
- Progress bar with time estimates
- Automatic legend and axis scaling

Post-processing visualization:
- Complete trajectory plots
- State history graphs
- Control effort analysis
- Integration with visualize.py for comprehensive reports

Key features:
- Modular design separated from simulation logic
- Efficient frame rendering with matplotlib
- Memory-optimized for long simulations
- Consistent styling with real hardware visualizations
"""

# Import logging
import logging
from datetime import datetime

import matplotlib.patches as patches
import numpy as np
from matplotlib.animation import writers

# Conditional import for visualization generator
try:
    from visualize import UnifiedVisualizationGenerator
except ImportError:
    UnifiedVisualizationGenerator = None  # type: ignore[assignment,misc]
logger = logging.getLogger(__name__)


class ProgressSuppressor:
    """
    Output stream wrapper that suppresses verbose output but allows progress bars.

    Filters out most matplotlib output while preserving progress-related output
    (lines with Encoding, Progress, etc.)
    """

    def __init__(self, stream):
        self.stream = stream
        self.progress_keywords = ["Encoding:", "Progress:", "█", "░", "%"]
        self.last_was_progress = False

    def write(self, text: str) -> None:
        """Write output, filtering out verbose messages."""
        # Check if this is progress bar output
        is_progress = any(keyword in text for keyword in self.progress_keywords)

        if is_progress:
            # Always output progress bar text
            self.stream.write(text)
            self.stream.flush()
            self.last_was_progress = True
        elif text == "\r":
            # Always allow carriage returns (used by progress bar)
            self.stream.write(text)
            self.stream.flush()
        elif text == "\n" and self.last_was_progress:
            # Only allow newlines that come after progress bar output
            self.stream.write(text)
            self.stream.flush()
            self.last_was_progress = False
        else:
            # Suppress other output
            self.last_was_progress = False

    def flush(self) -> None:
        """Flush the underlying stream."""
        self.stream.flush()

    def isatty(self) -> bool:
        """Check if stream is a TTY."""
        return hasattr(self.stream, "isatty") and self.stream.isatty()


class SimulationVisualizationManager:
    """
    Manages all visualization and animation for simulation mode.

    Handles:
    - Real-time matplotlib drawing
    - Satellite, target, and trajectory rendering
    - Animation export to MP4
    - Post-test visualization generation
    """

    def __init__(self, controller):
        """
        Initialize visualization manager.

        Args:
            controller: Reference to parent SatelliteMPCLinearizedSimulation instance
        """
        self.controller = controller
        # Convenient aliases to controller attributes
        self.satellite = controller.satellite
        self.target_state = None  # Will be updated via controller
        self.position_tolerance = None
        self.simulation_time = None
        self.state_history = None
        self.is_running = None
        self.mpc_solve_times = None
        self.max_simulation_time = None
        self.data_save_path = None

        # Import Config here to avoid circular imports
        from config import SatelliteConfig

        self.SatelliteConfig = SatelliteConfig

    def sync_from_controller(self):
        """Sync attributes from controller before each draw operation."""
        c = self.controller
        self.target_state = c.target_state
        self.position_tolerance = c.position_tolerance
        self.simulation_time = c.simulation_time
        self.state_history = c.state_history
        self.is_running = c.is_running
        self.mpc_solve_times = c.mpc_solve_times
        self.max_simulation_time = c.max_simulation_time
        self.data_save_path = c.data_save_path
        self.control_history = c.control_history
        self.mpc_info_history = c.mpc_info_history
        self.angle_tolerance = c.angle_tolerance
        self.control_update_interval = c.control_update_interval
        self.target_reached_time = c.target_reached_time
        self.target_maintenance_time = c.target_maintenance_time
        self.maintenance_position_errors = c.maintenance_position_errors
        self.maintenance_angle_errors = c.maintenance_angle_errors
        self.times_lost_target = c.times_lost_target

    def angle_difference(self, target_angle, current_angle):
        """Delegate to controller's angle_difference method."""
        return self.controller.angle_difference(target_angle, current_angle)

    def get_current_state(self):
        """Delegate to controller's get_current_state method."""
        return self.controller.get_current_state()

    def check_target_reached(self):
        """Delegate to controller's check_target_reached method."""
        return self.controller.check_target_reached()

    def save_animation_mp4(self, fig, ani):
        """Save the animation as MP4 file."""
        if not self.data_save_path:
            return

        mp4_path = self.data_save_path / "simulation_animation.mp4"

        try:
            # Set up the writer
            Writer = writers["ffmpeg"]
            writer = Writer(
                fps=10, metadata=dict(artist="Linearized MPC Simulation"), bitrate=1800
            )

            # Save animation
            ani.save(str(mp4_path), writer=writer)
            logger.info(f"Animation saved to: {mp4_path}")
        except Exception as e:
            logger.error(f"ERROR: Error saving animation: {e}")
            logger.error(
                "WARNING: Make sure ffmpeg is installed: brew install ffmpeg (Mac) or apt install ffmpeg (Linux)"
            )

    def draw_simulation(self):
        """Draw the simulation including satellite, target, and trajectory."""
        # Store current axis limits to restore them after clearing
        xlim = self.satellite.ax_main.get_xlim()
        ylim = self.satellite.ax_main.get_ylim()

        # Clear the axes
        self.satellite.ax_main.clear()

        # Immediately restore axis properties to prevent any reset issues
        self.satellite.ax_main.set_xlim(xlim)
        self.satellite.ax_main.set_ylim(ylim)
        self.satellite.ax_main.set_aspect("equal")
        self.satellite.ax_main.grid(True, alpha=0.3)
        self.satellite.ax_main.set_xlabel(
            "X Position (m)", fontsize=12, fontweight="bold"
        )
        self.satellite.ax_main.set_ylabel(
            "Y Position (m)", fontsize=12, fontweight="bold"
        )

        # Draw target elements FIRST with highest z-order
        target_x, target_y, target_angle = self.target_state[0], self.target_state[1], self.target_state[4]  # type: ignore[index]

        # Target tolerance circle
        target_circle = patches.Circle(
            (target_x, target_y),
            self.position_tolerance,  # type: ignore[arg-type]
            fill=False,
            color="green",
            linestyle="--",
            alpha=0.5,
            zorder=20,
        )
        target_circle.set_zorder(20)
        self.satellite.ax_main.add_patch(target_circle)

        # Target orientation arrow
        target_arrow_length = 0.2
        target_arrow_end = np.array(
            [target_x, target_y]
        ) + target_arrow_length * np.array([np.cos(target_angle), np.sin(target_angle)])
        arrow = self.satellite.ax_main.annotate(
            "",
            xy=target_arrow_end,
            xytext=[target_x, target_y],
            arrowprops=dict(arrowstyle="->", lw=3, color="darkgreen", alpha=0.8),
        )
        arrow.set_zorder(21)

        # Target position marker
        self.satellite.ax_main.plot(
            target_x,
            target_y,
            "ro",
            markersize=12,
            markerfacecolor="red",
            markeredgecolor="darkred",
            linewidth=2,
            zorder=22,
        )

        self._draw_satellite_elements()

        # Update title with current metrics
        current_state = self.get_current_state()
        pos_error = np.linalg.norm(current_state[:2] - self.target_state[:2])  # type: ignore[index]
        ang_error = abs(self.angle_difference(self.target_state[4], current_state[4]))  # type: ignore[index]

        # Create title
        title = "Linearized MPC Simulation\n"
        title += f"Time: {self.simulation_time:.1f}s | Pos Error: {pos_error:.3f}m | Ang Error: {np.degrees(ang_error):.1f}°"

        self.satellite.ax_main.set_title(title, fontsize=14, fontweight="bold")

    def _draw_obstacles(self):
        """Draw configured obstacles on the visualization."""
        if self.SatelliteConfig.OBSTACLES_ENABLED:
            obstacles = self.SatelliteConfig.get_obstacles()
            for i, (obs_x, obs_y, obs_radius) in enumerate(obstacles, 1):
                # Draw obstacle as red circle
                obstacle_circle = patches.Circle(
                    (obs_x, obs_y),
                    obs_radius,
                    fill=True,
                    color="red",
                    alpha=0.6,
                    zorder=8,
                )
                self.satellite.ax_main.add_patch(obstacle_circle)

                # Draw safety zone as red dashed circle
                safety_radius = obs_radius + 0.25
                safety_circle = patches.Circle(
                    (obs_x, obs_y),
                    safety_radius,
                    fill=False,
                    color="red",
                    linestyle="--",
                    alpha=0.4,
                    zorder=9,
                )
                self.satellite.ax_main.add_patch(safety_circle)

                # Add obstacle label  # type: ignore[attr-defined]
                self.satellite.ax_main.text(
                    obs_x,
                    obs_y,
                    f"O{i}",
                    fontsize=10,
                    color="white",  # type: ignore[attr-defined]
                    ha="center",
                    va="center",
                    fontweight="bold",
                    zorder=24,
                )

    def _draw_obstacle_avoidance_waypoints(self):
        """Draw obstacle avoidance waypoints for point-to-point and multi-point modes."""
        if hasattr(self, "obstacle_avoiding_waypoints") and self.obstacle_avoiding_waypoints:  # type: ignore[attr-defined]
            waypoints = self.obstacle_avoiding_waypoints  # type: ignore[attr-defined]
            current_idx = getattr(self, "current_obstacle_waypoint_idx", 0)

            # Draw waypoint path
            for i in range(len(waypoints) - 1):
                start_pos = waypoints[i]
                end_pos = waypoints[i + 1]

                # Color based on completion status
                if i < current_idx:
                    color = "green"  # Completed
                    alpha = 0.8
                elif i == current_idx:
                    color = "orange"  # Current
                    alpha = 1.0
                else:
                    color = "magenta"  # Future (obstacle avoidance path)
                    alpha = 0.6

                # Draw line segment
                self.satellite.ax_main.plot(
                    [start_pos[0], end_pos[0]],
                    [start_pos[1], end_pos[1]],
                    color=color,
                    linewidth=4,
                    alpha=alpha,
                    zorder=13,
                )

            # Draw waypoint markers
            for i, waypoint in enumerate(waypoints):
                if i < current_idx:
                    color = "green"
                    marker = "s"  # Square for completed
                    size = 10
                    alpha = 0.8
                elif i == current_idx:
                    color = "orange"
                    marker = "D"  # Diamond for current
                    size = 14
                    alpha = 1.0
                else:
                    color = "magenta"
                    marker = "s"  # Square for future
                    size = 10
                    alpha = 0.6
                # type: ignore[attr-defined]
                self.satellite.ax_main.plot(
                    waypoint[0],
                    waypoint[1],
                    marker,
                    color=color,  # type: ignore[attr-defined]
                    markersize=size,
                    alpha=alpha,
                    zorder=19,
                )

    def _draw_satellite_elements(self):
        """Draw satellite elements manually to avoid conflicts."""
        cos_angle = np.cos(self.satellite.angle)
        sin_angle = np.sin(self.satellite.angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

        # Draw trajectory trail
        if len(self.satellite.trajectory) > 1:
            trajectory_array = np.array(self.satellite.trajectory)
            alphas = np.linspace(0.1, 0.6, len(self.satellite.trajectory))

            for i in range(len(self.satellite.trajectory) - 1):
                self.satellite.ax_main.plot(
                    [trajectory_array[i, 0], trajectory_array[i + 1, 0]],
                    [trajectory_array[i, 1], trajectory_array[i + 1, 1]],
                    "b-",
                    alpha=alphas[i],
                    linewidth=1,
                    zorder=1,
                )

        square_corners = np.array(
            [
                [
                    -self.satellite.satellite_size / 2,
                    -self.satellite.satellite_size / 2,
                ],
                [self.satellite.satellite_size / 2, -self.satellite.satellite_size / 2],
                [self.satellite.satellite_size / 2, self.satellite.satellite_size / 2],
                [-self.satellite.satellite_size / 2, self.satellite.satellite_size / 2],
            ]
        )

        rotated_corners = np.array(
            [rotation_matrix @ corner for corner in square_corners]
        )
        translated_corners = rotated_corners + self.satellite.position

        satellite_patch = patches.Polygon(
            translated_corners,
            linewidth=3,
            edgecolor="black",
            facecolor="lightgray",
            alpha=0.7,
            zorder=5,
        )
        self.satellite.ax_main.add_patch(satellite_patch)

        # Draw thrusters with color coding
        for thruster_id, local_pos in self.satellite.thrusters.items():
            global_pos = rotation_matrix @ np.array(local_pos) + self.satellite.position
            color = self.satellite.thruster_colors[thruster_id]
            is_active = thruster_id in self.satellite.active_thrusters

            if is_active:
                thruster_size = 40
                alpha = 1.0
            else:
                thruster_size = 20
                alpha = 0.3

            self.satellite.ax_main.scatter(
                global_pos[0],
                global_pos[1],
                s=thruster_size,
                c=color,
                alpha=alpha,
                zorder=10,
            )

    def update_mpc_info_panel(self):
        """Update the information panel to match Visualize_Simulation_Linearized copy.py format."""
        self.satellite.ax_info.clear()
        self.satellite.ax_info.set_xlim(0, 1)
        self.satellite.ax_info.set_ylim(0, 1)
        self.satellite.ax_info.axis("off")
        self.satellite.ax_info.set_title("Simulation Info", fontsize=12, weight="bold")

        # Get current state and calculate values
        current_state = self.get_current_state()
        net_force, net_torque = self.satellite.calculate_forces_and_torques()

        # Calculate errors
        pos_error = np.linalg.norm(current_state[:2] - self.target_state[:2])  # type: ignore[index]
        angle_error = abs(np.degrees(self.angle_difference(self.target_state[4], current_state[4])))  # type: ignore[index]
        # type: ignore[index]
        # Get active thrusters
        active_thrusters = (
            list(sorted(self.satellite.active_thrusters))
            if self.satellite.active_thrusters
            else []
        )

        mpc_time = self.mpc_solve_times[-1] if self.mpc_solve_times else 0.0
        mpc_status = "Active" if self.is_running else "Stopped"

        # Current step estimation
        current_step = int(self.simulation_time / self.satellite.dt)
        max_steps = int(self.max_simulation_time / self.satellite.dt)

        # Information text matching the visualization format
        info_text = [
            "LINEARIZED MPC SIMULATION",
            f"{'=' * 25}",
            f"Step: {current_step}/{max_steps}",
            f"Time: {self.simulation_time:.1f}s",
            "",
            "CURRENT STATE:",
            f"X: {current_state[0]:.3f} m",
            f"Y: {current_state[1]:.3f} m",
            f"Yaw: {np.degrees(current_state[4]):.1f}°",
            "",
            "TARGET STATE:",
            f"X: {self.target_state[0]:.3f} m",  # type: ignore[index]
            f"Y: {self.target_state[1]:.3f} m",  # type: ignore[index]
            f"Yaw: {np.degrees(self.target_state[4]):.1f}°",  # type: ignore[index]
            "",
            "CONTROL ERRORS:",
            f"Position: {pos_error:.3f} m",
            f"Angle: {angle_error:.1f}°",
            "",
            "MPC CONTROLLER:",
            f"Status: {mpc_status}",
            f"Solve Time: {mpc_time:.3f}s",
            "",
            "THRUSTER CONTROL:",
            f"Active: {active_thrusters}",
            f"Count: {len(active_thrusters)}/8",
        ]

        # Display text with the same formatting as visualization
        y_pos = 0.95
        for line in info_text:
            if line.startswith("="):
                weight = "normal"
                size = 9
            elif line.isupper() and line.endswith(":"):
                weight = "bold"
                size = 10
            elif line.startswith(("LINEARIZED MPC", "Step:", "Time:")):
                weight = "bold"
                size = 11
            else:
                weight = "normal"
                size = 9

            self.satellite.ax_info.text(
                0.05,
                y_pos,
                line,
                fontsize=size,
                weight=weight,
                verticalalignment="top",
                fontfamily="monospace",
            )
            y_pos -= 0.035

    def save_mission_summary(self):
        """
        Save comprehensive mission summary and performance metrics to a text file.
        Creates a permanent record of:
        1. Mission configuration (user inputs + Config parameters)
        2. Initial conditions
        3. Performance metrics and results

        This file allows recreation of the exact same mission scenario.
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

        try:
            with open(summary_file_path, "w") as f:
                # Write header with timestamp
                f.write("=" * 80 + "\n")
                f.write("SATELLITE CONTROL SYSTEM - MISSION SUMMARY & CONFIGURATION\n")
                f.write("=" * 80 + "\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Data Directory: {self.data_save_path}\n")
                f.write("=" * 80 + "\n\n")

                # ========== MISSION CONFIGURATION (RECREATION DATA) ==========
                f.write("=" * 80 + "\n")
                f.write("MISSION CONFIGURATION - RECREATION DATA\n")
                f.write("=" * 80 + "\n")
                f.write(
                    "This section contains all information needed to recreate this exact mission.\n\n"
                )

                # Determine mission type and write type-specific configuration
                if (
                    hasattr(self.SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
                    and self.SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
                ):
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

                    # Starting configuration
                    initial_state = self.state_history[0]
                    f.write("STARTING CONFIGURATION:\n")
                    f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
                    f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
                    f.write(
                        f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n"
                    )

                    # Shape configuration
                    f.write("SHAPE CONFIGURATION:\n")
                    if (
                        hasattr(self.SatelliteConfig, "DXF_SHAPE_CENTER")
                        and self.SatelliteConfig.DXF_SHAPE_CENTER
                    ):
                        f.write(
                            f"  Shape Center X:          {self.SatelliteConfig.DXF_SHAPE_CENTER[0]:.3f} m\n"
                        )
                        f.write(
                            f"  Shape Center Y:          {self.SatelliteConfig.DXF_SHAPE_CENTER[1]:.3f} m\n"
                        )
                    if hasattr(self.SatelliteConfig, "DXF_SHAPE_ROTATION"):
                        f.write(
                            f"  Shape Rotation:          {np.degrees(self.SatelliteConfig.DXF_SHAPE_ROTATION):.1f}°\n"
                        )
                    if hasattr(self.SatelliteConfig, "DXF_OFFSET_DISTANCE"):
                        f.write(
                            f"  Offset Distance:         {self.SatelliteConfig.DXF_OFFSET_DISTANCE:.3f} m\n"
                        )
                    if hasattr(self.SatelliteConfig, "DXF_PATH_LENGTH"):
                        f.write(
                            f"  Path Length:             {self.SatelliteConfig.DXF_PATH_LENGTH:.3f} m\n"
                        )
                    if (
                        hasattr(self.SatelliteConfig, "DXF_BASE_SHAPE")
                        and self.SatelliteConfig.DXF_BASE_SHAPE
                    ):
                        f.write(
                            f"  Number of Path Points:   {len(self.SatelliteConfig.DXF_SHAPE_PATH) if hasattr(self.SatelliteConfig, 'DXF_SHAPE_PATH') and self.SatelliteConfig.DXF_SHAPE_PATH else 0}\n\n"
                        )

                    # Moving target configuration
                    f.write("MOVING TARGET CONFIGURATION:\n")
                    if hasattr(self.SatelliteConfig, "DXF_TARGET_SPEED"):
                        f.write(
                            f"  Target Speed:            {self.SatelliteConfig.DXF_TARGET_SPEED:.3f} m/s\n"
                        )
                    if hasattr(self.SatelliteConfig, "DXF_ESTIMATED_DURATION"):
                        f.write(
                            f"  Estimated Duration:      {self.SatelliteConfig.DXF_ESTIMATED_DURATION:.1f} s\n\n"
                        )

                elif (
                    hasattr(self.SatelliteConfig, "ENABLE_WAYPOINT_MODE")
                    and self.SatelliteConfig.ENABLE_WAYPOINT_MODE
                ):
                    f.write("MISSION TYPE: WAYPOINT NAVIGATION (MODE 1)\n")
                    f.write("-" * 50 + "\n")
                    f.write(
                        "Description: Satellite visits multiple waypoints in sequence\n\n"
                    )

                    # Starting configuration
                    initial_state = self.state_history[0]
                    f.write("STARTING CONFIGURATION:\n")
                    f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
                    f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
                    f.write(
                        f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n"
                    )

                    # Waypoints
                    f.write("WAYPOINTS:\n")
                    f.write(
                        f"  Number of Waypoints:     {len(self.SatelliteConfig.WAYPOINT_TARGETS)}\n"
                    )
                    for i, ((tx, ty), ta) in enumerate(
                        zip(
                            self.SatelliteConfig.WAYPOINT_TARGETS,
                            self.SatelliteConfig.WAYPOINT_ANGLES,
                        ),
                        1,
                    ):
                        f.write(
                            f"  Waypoint {i}:              ({tx:.3f}, {ty:.3f}) m, {np.degrees(ta):.1f}°\n"
                        )
                    f.write(
                        f"\n  Hold Time per Waypoint:  {self.SatelliteConfig.TARGET_HOLD_TIME:.1f} s\n\n"
                    )

                else:
                    f.write("MISSION TYPE: POINT-TO-POINT (MODE 1)\n")
                    f.write("-" * 50 + "\n")
                    f.write(
                        "Description: Satellite navigates to target position and orientation\n\n"
                    )

                    # Starting configuration
                    initial_state = self.state_history[0]
                    f.write("STARTING CONFIGURATION:\n")
                    f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
                    f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
                    f.write(
                        f"  Starting orientation:    {np.degrees(initial_state[4]):.1f}°\n\n"
                    )

                    # Target configuration
                    target_pos = self.target_state[:2]  # type: ignore[index]
                    target_angle = self.target_state[4]  # type: ignore[index]
                    f.write("TARGET CONFIGURATION:\n")
                    f.write(f"  Target X position:       {target_pos[0]:.3f} m\n")
                    f.write(f"  Target Y position:       {target_pos[1]:.3f} m\n")
                    f.write(
                        f"  Target orientation:      {np.degrees(target_angle):.1f}°\n\n"
                    )

                # Obstacle configuration (if enabled)
                if (
                    self.SatelliteConfig.OBSTACLES_ENABLED
                    and self.SatelliteConfig.OBSTACLES
                ):
                    f.write("OBSTACLE CONFIGURATION:\n")
                    f.write("  Obstacle Avoidance:      ENABLED\n")
                    f.write(
                        f"  Number of Obstacles:     {len(self.SatelliteConfig.OBSTACLES)}\n"
                    )
                    for i, (ox, oy, orad) in enumerate(
                        self.SatelliteConfig.OBSTACLES, 1
                    ):
                        f.write(
                            f"  Obstacle {i}:              ({ox:.3f}, {oy:.3f}) m, radius {orad:.3f} m\n"
                        )
                    f.write("\n")
                else:
                    f.write("OBSTACLE CONFIGURATION:\n")
                    f.write("  Obstacle Avoidance:      DISABLED\n\n")

                # ========== CONTROLLER CONFIGURATION ==========
                f.write("=" * 80 + "\n")
                f.write("CONTROLLER CONFIGURATION\n")
                f.write("=" * 80 + "\n")
                f.write(
                    "These parameters affect mission performance and control behavior.\n\n"
                )

                f.write("MPC CONTROLLER PARAMETERS:\n")
                f.write("-" * 50 + "\n")
                f.write("  Controller Type:         Linearized MPC (A*x[k] + B*u[k])\n")
                f.write(
                    f"  Solver:                  {self.SatelliteConfig.MPC_SOLVER_TYPE}\n"
                )
                f.write(
                    f"  Prediction Horizon:      {self.SatelliteConfig.MPC_PREDICTION_HORIZON} steps\n"
                )
                f.write(
                    f"  Control Horizon:         {self.SatelliteConfig.MPC_CONTROL_HORIZON} steps\n"
                )
                f.write(
                    f"  Simulation Timestep:     {self.SatelliteConfig.SIMULATION_DT:.3f} s\n"
                )
                f.write(
                    f"  Control Timestep:        {self.SatelliteConfig.CONTROL_DT:.3f} s\n"
                )
                f.write(
                    f"  Solver Time Limit:       {self.SatelliteConfig.MPC_SOLVER_TIME_LIMIT:.3f} s\n\n"
                )

                f.write("COST FUNCTION WEIGHTS:\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"  Position Weight (Q):     {self.SatelliteConfig.Q_POSITION:.1f}\n"
                )
                f.write(
                    f"  Velocity Weight (Q):     {self.SatelliteConfig.Q_VELOCITY:.1f}\n"
                )
                f.write(
                    f"  Angle Weight (Q):        {self.SatelliteConfig.Q_ANGLE:.1f}\n"
                )
                f.write(
                    f"  Angular Vel Weight (Q):  {self.SatelliteConfig.Q_ANGULAR_VELOCITY:.1f}\n"
                )
                f.write(
                    f"  Thrust Penalty (R):      {self.SatelliteConfig.R_THRUST:.3f}\n"
                )
                f.write(
                    f"  Switch Penalty (R):      {self.SatelliteConfig.R_SWITCH:.3f}\n\n"
                )

                f.write("CONTROL CONSTRAINTS:\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"  Max Velocity:            {self.SatelliteConfig.MAX_VELOCITY:.3f} m/s\n"
                )
                f.write(
                    f"  Max Angular Velocity:    {np.degrees(self.SatelliteConfig.MAX_ANGULAR_VELOCITY):.1f}°/s\n"
                )
                f.write(
                    f"  Position Bounds:         ±{self.SatelliteConfig.POSITION_BOUNDS:.1f} m\n\n"
                )

                f.write("TOLERANCE THRESHOLDS:\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"  Position Tolerance:      {self.SatelliteConfig.POSITION_TOLERANCE:.3f} m\n"
                )
                f.write(
                    f"  Angle Tolerance:         {np.degrees(self.SatelliteConfig.ANGLE_TOLERANCE):.1f}°\n"
                )
                f.write(
                    f"  Velocity Tolerance:      {self.SatelliteConfig.VELOCITY_TOLERANCE:.3f} m/s\n"
                )
                f.write(
                    f"  Angular Vel Tolerance:   {np.degrees(self.SatelliteConfig.ANGULAR_VELOCITY_TOLERANCE):.1f}°/s\n\n"
                )

                # ========== PHYSICAL PARAMETERS ==========
                f.write("PHYSICAL PARAMETERS:\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"  Total Mass:              {self.SatelliteConfig.TOTAL_MASS:.3f} kg\n"
                )
                f.write(
                    f"  Moment of Inertia:       {self.SatelliteConfig.MOMENT_OF_INERTIA:.6f} kg·m²\n"
                )
                f.write(
                    f"  Satellite Size:          {self.SatelliteConfig.SATELLITE_SIZE:.3f} m\n"
                )
                if self.SatelliteConfig.COM_OFFSET is not None:
                    f.write(
                        f"  COM Offset:              ({self.SatelliteConfig.COM_OFFSET[0]:.6f}, {self.SatelliteConfig.COM_OFFSET[1]:.6f}) m\n\n"
                    )
                else:
                    f.write("  COM Offset:              Not calculated\n\n")

                # Thruster forces
                f.write("THRUSTER FORCES:\n")
                for tid in range(1, 9):
                    f.write(
                        f"  Thruster {tid}:             {self.SatelliteConfig.THRUSTER_FORCES[tid]:.6f} N\n"
                    )
                f.write("\n")

                # ========== REALISTIC PHYSICS SETTINGS ==========
                f.write("REALISTIC PHYSICS SETTINGS:\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"  Realistic Physics:       {'ENABLED' if self.SatelliteConfig.USE_REALISTIC_PHYSICS else 'DISABLED'}\n"
                )
                if self.SatelliteConfig.USE_REALISTIC_PHYSICS:
                    f.write(
                        f"  Linear Damping Coeff:    {self.SatelliteConfig.LINEAR_DAMPING_COEFF:.3f} N/(m/s)\n"
                    )
                    f.write(
                        f"  Rotational Damping:      {self.SatelliteConfig.ROTATIONAL_DAMPING_COEFF:.4f} N·m/(rad/s)\n"
                    )
                    f.write(
                        f"  Position Noise Std:      {self.SatelliteConfig.POSITION_NOISE_STD * 1000:.2f} mm\n"
                    )
                    f.write(
                        f"  Angle Noise Std:         {np.degrees(self.SatelliteConfig.ANGLE_NOISE_STD):.2f}°\n"
                    )
                    f.write(
                        f"  Velocity Noise Std:      {self.SatelliteConfig.VELOCITY_NOISE_STD * 1000:.2f} mm/s\n"
                    )
                    f.write(
                        f"  Angular Vel Noise Std:   {np.degrees(self.SatelliteConfig.ANGULAR_VELOCITY_NOISE_STD):.2f}°/s\n"
                    )
                    f.write(
                        f"  Thruster Valve Delay:    {self.SatelliteConfig.THRUSTER_VALVE_DELAY * 1000:.1f} ms\n"
                    )
                    f.write(
                        f"  Thrust Rampup Time:      {self.SatelliteConfig.THRUSTER_RAMPUP_TIME * 1000:.1f} ms\n"
                    )
                    f.write(
                        f"  Thrust Force Noise:      {self.SatelliteConfig.THRUSTER_FORCE_NOISE_STD * 100:.1f}%\n"
                    )
                    f.write(
                        f"  Random Disturbances:     {'ENABLED' if self.SatelliteConfig.ENABLE_RANDOM_DISTURBANCES else 'DISABLED'}\n"
                    )
                    if self.SatelliteConfig.ENABLE_RANDOM_DISTURBANCES:
                        f.write(
                            f"  Disturbance Force Std:   {self.SatelliteConfig.DISTURBANCE_FORCE_STD:.3f} N\n"
                        )
                        f.write(
                            f"  Disturbance Torque Std:  {self.SatelliteConfig.DISTURBANCE_TORQUE_STD:.4f} N·m\n"
                        )
                f.write("\n")

                f.write("=" * 80 + "\n")
                f.write("MISSION PERFORMANCE RESULTS\n")
                f.write("=" * 80 + "\n\n")

                # Calculate all metrics (same as print_performance_summary)
                initial_state = self.state_history[0]
                final_state = self.get_current_state()
                initial_pos = initial_state[:2]
                final_pos = final_state[:2]
                target_pos = self.target_state[:2]  # type: ignore[index]

                pos_error_initial = np.linalg.norm(initial_pos - target_pos)
                pos_error_final = np.linalg.norm(final_pos - target_pos)
                ang_error_initial = abs(self.angle_difference(self.target_state[4], initial_state[4]))  # type: ignore[index]
                ang_error_final = abs(self.angle_difference(self.target_state[4], final_state[4]))  # type: ignore[index]
                # type: ignore[attr-defined]
                trajectory_distance = sum(
                    np.linalg.norm(self.state_history[i][:2] - self.state_history[i - 1][:2])  # type: ignore[attr-defined]
                    for i in range(1, len(self.state_history))
                )

                mpc_convergence_times = np.array(self.mpc_solve_times) if self.mpc_solve_times else np.array([])  # type: ignore[attr-defined]
                # type: ignore[attr-defined]
                total_thrust_activations = sum(
                    np.sum(control) for control in self.control_history
                )
                total_thrust_magnitude = sum(
                    np.linalg.norm(control) for control in self.control_history
                )

                switching_events = 0
                for i in range(1, len(self.control_history)):
                    curr_control = self.control_history[i]
                    prev_control = self.control_history[i - 1]
                    if len(curr_control) < 8:
                        curr_control = np.pad(
                            curr_control, (0, 8 - len(curr_control)), "constant"
                        )
                    if len(prev_control) < 8:
                        prev_control = np.pad(
                            prev_control, (0, 8 - len(prev_control)), "constant"
                        )
                    switching_events += np.sum(np.abs(curr_control - prev_control))

                success = self.check_target_reached()
                vel_magnitude_final = np.linalg.norm(final_state[2:4])

                avg_objective = np.mean(
                    [
                        info.get("objective_value", 0)
                        for info in self.mpc_info_history
                        if info.get("objective_value")
                    ]
                )
                avg_variables = np.mean(
                    [
                        info.get("num_variables", 0)
                        for info in self.mpc_info_history
                        if info.get("num_variables")
                    ]
                )
                avg_constraints = np.mean(
                    [
                        info.get("num_constraints", 0)
                        for info in self.mpc_info_history
                        if info.get("num_constraints")
                    ]
                )

                # Write all metrics to file
                f.write("[FORMULATION] LINEARIZED MPC FORMULATION ANALYSIS\n")
                f.write("-" * 50 + "\n")
                f.write(
                    "Formulation Type:          A*x[k] + B*u[k] (Linearized Dynamics)\n"
                )
                f.write(
                    f"Average Problem Size:      {avg_variables:.0f} variables, {avg_constraints:.0f} constraints\n"
                )
                f.write(f"Average Objective Value:   {avg_objective:.2f}\n\n")

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
                f.write(
                    f"Position Improvement:      {((pos_error_initial - pos_error_final) / pos_error_initial * 100):.1f}%\n"
                )
                f.write(f"Total Distance Traveled:   {trajectory_distance:.3f} m\n")  # type: ignore[attr-defined]
                f.write(
                    f"Trajectory Efficiency:     {(np.linalg.norm(target_pos - initial_pos) / trajectory_distance * 100):.1f}%\n\n"
                )

                f.write("ORIENTATION ANALYSIS\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"Initial Angle Error:       {np.degrees(ang_error_initial):.2f}°\n"
                )
                f.write(
                    f"Final Angle Error:         {np.degrees(ang_error_final):.2f}° (target: <{np.degrees(self.angle_tolerance):.1f}°)\n"
                )
                f.write(
                    f"Angle Improvement:         {((ang_error_initial - ang_error_final) / ang_error_initial * 100):.1f}%\n"
                )
                f.write(f"Final Velocity Magnitude:  {vel_magnitude_final:.4f} m/s\n")
                f.write(
                    f"Final Angular Velocity:    {np.degrees(final_state[5]):.2f}°/s\n\n"
                )

                f.write("LINEARIZED MPC CONTROLLER PERFORMANCE\n")
                f.write("-" * 50 + "\n")
                f.write(f"Total Simulation Time:     {self.simulation_time:.1f} s\n")
                f.write(
                    f"MPC Updates:               {len(mpc_convergence_times)} cycles\n"
                )
                if self.simulation_time > 0:  # type: ignore[operator]
                    f.write(f"MPC Update Rate:           {len(mpc_convergence_times) / self.simulation_time:.1f} Hz\n")  # type: ignore[operator]
                else:
                    f.write(
                        "MPC Update Rate:           N/A (insufficient simulation time)\n"
                    )
                # type: ignore[attr-defined]
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
                    f.write(
                        f"Real-time Performance:     {(np.mean(mpc_convergence_times) / self.control_update_interval * 100):.1f}% of available time\n"
                    )
                f.write("\n")

                f.write("CONTROL EFFORT & FUEL ANALYSIS\n")
                f.write("-" * 50 + "\n")
                f.write(f"Total Thruster Activations: {total_thrust_activations:.0f}\n")
                f.write(
                    f"Total Control Magnitude:    {total_thrust_magnitude:.2f} N·s\n"
                )
                if len(self.control_history) > 0:
                    f.write(
                        f"Average Control per Step:   {total_thrust_magnitude / len(self.control_history):.3f} N\n"
                    )
                    f.write(f"Thruster Switching Events:  {switching_events:.0f}\n")
                    f.write(
                        f"Control Smoothness:         {(1 - switching_events / (len(self.control_history) * 8)) * 100:.1f}%\n"
                    )
                else:
                    f.write("Average Control per Step:   N/A (no control history)\n")
                    f.write(f"Thruster Switching Events:  {switching_events:.0f}\n")
                    f.write("Control Smoothness:         N/A (no control history)\n")
                if trajectory_distance > 0:
                    f.write(
                        f"Fuel Efficiency:            {total_thrust_magnitude / trajectory_distance:.3f} N·s/m\n\n"
                    )
                else:  # type: ignore[attr-defined]
                    f.write(
                        "Fuel Efficiency:            N/A (zero distance traveled)\n\n"
                    )
                # type: ignore[attr-defined]
                f.write("MISSION SUCCESS & TARGET MAINTENANCE ANALYSIS\n")
                f.write("-" * 50 + "\n")
                f.write(
                    f"Position Tolerance:        <{self.position_tolerance:.3f} m\n"
                )
                f.write(
                    f"Angle Tolerance:           <{np.degrees(self.angle_tolerance):.1f}°\n"
                )
                f.write(
                    f"Position Target Met:       {'YES' if self.position_tolerance is not None and pos_error_final < self.position_tolerance else 'NO'}\n"
                )
                f.write(
                    f"Angle Target Met:          {'YES' if self.angle_tolerance is not None and ang_error_final < self.angle_tolerance else 'NO'}\n"
                )
                f.write(
                    f"Overall Mission Status:    {'SUCCESS' if success else 'INCOMPLETE'}\n\n"
                )

                # Precision analysis
                precision_target_met = pos_error_final <= 0.050
                if precision_target_met:
                    f.write(
                        f"PRECISION REQUIREMENT MET: Final position error {pos_error_final:.4f}m <= 0.050m \n"
                    )
                else:
                    f.write(
                        f"PRECISION REQUIREMENT NOT MET: Final position error {pos_error_final:.4f}m > 0.050m \n"
                    )

                f.write("\nPRECISION ANALYSIS\n")
                f.write("-" * 50 + "\n")
                f.write("Target Precision:          0.050m (5cm)\n")
                f.write(f"Achieved Precision:        {pos_error_final:.4f}m\n")
                precision_ratio = pos_error_final / 0.050
                if precision_ratio <= 1.0:
                    f.write(
                        f"Precision Ratio:           {precision_ratio:.2f} (PASSED)\n"
                    )
                else:
                    f.write(
                        f"Precision Ratio:           {precision_ratio:.2f} (FAILED - {((precision_ratio - 1) * 100):.1f}% over target)\n"
                    )
                f.write("\n")

                # Target maintenance analysis
                if self.target_reached_time is not None:
                    f.write("TARGET MAINTENANCE ANALYSIS\n")
                    f.write("-" * 50 + "\n")
                    f.write(
                        f"Target First Reached:      {self.target_reached_time:.1f} s\n"
                    )
                    if self.simulation_time > 0:  # type: ignore[attr-defined]
                        f.write(f"Target Maintenance Time:   {self.target_maintenance_time:.1f} s ({(self.target_maintenance_time / self.simulation_time * 100):.1f}% of total)\n")  # type: ignore[attr-defined]
                    else:  # type: ignore[attr-defined]
                        f.write(f"Target Maintenance Time:   {self.target_maintenance_time:.1f} s\n")  # type: ignore[attr-defined]
                    f.write(f"Times Lost Target:         {self.times_lost_target}\n")  # type: ignore[attr-defined]

                    if self.maintenance_position_errors:  # type: ignore[attr-defined]
                        avg_maintenance_pos_err = np.mean(self.maintenance_position_errors)  # type: ignore[attr-defined]
                        max_maintenance_pos_err = np.max(self.maintenance_position_errors)  # type: ignore[attr-defined]
                        avg_maintenance_ang_err = np.mean(self.maintenance_angle_errors)  # type: ignore[attr-defined]
                        max_maintenance_ang_err = np.max(self.maintenance_angle_errors)  # type: ignore[attr-defined]

                        f.write(f"Avg Maintenance Pos Error: {avg_maintenance_pos_err:.4f} m\n")  # type: ignore[attr-defined]
                        f.write(
                            f"Max Maintenance Pos Error: {max_maintenance_pos_err:.4f} m\n"
                        )
                        f.write(
                            f"Avg Maintenance Ang Error: {np.degrees(avg_maintenance_ang_err):.2f}°\n"
                        )
                        f.write(
                            f"Max Maintenance Ang Error: {np.degrees(max_maintenance_ang_err):.2f}°\n"
                        )

                        pos_stability = 100 * (1 - avg_maintenance_pos_err / self.position_tolerance)  # type: ignore[operator]
                        ang_stability = 100 * (1 - avg_maintenance_ang_err / self.angle_tolerance)  # type: ignore[attr-defined]
                        f.write(
                            f"Position Stability:        {pos_stability:.1f}% (within tolerance)\n"
                        )
                        f.write(f"Angle Stability:           {ang_stability:.1f}% (within tolerance)\n")  # type: ignore[attr-defined]
                else:
                    f.write("TARGET STATUS\n")
                    f.write("-" * 50 + "\n")
                    f.write("Target Never Reached:      Mission incomplete\n")
                    remaining_pos_error = max(0, pos_error_final - self.position_tolerance)  # type: ignore[operator]
                    remaining_ang_error = max(0, ang_error_final - self.angle_tolerance)  # type: ignore[attr-defined]
                    f.write(f"Remaining Position Error:  {remaining_pos_error:.4f} m\n")
                    f.write(
                        f"Remaining Angle Error:     {np.degrees(remaining_ang_error):.2f}°\n"
                    )

                f.write("\n" + "=" * 80 + "\n")
                f.write("END OF MISSION SUMMARY\n")
                f.write("=" * 80 + "\n")

            # Log and print success message
            logger.info(f"Mission summary saved to: {summary_file_path}")
            print(f" Mission summary saved: {summary_file_path}")

        except Exception as e:
            logger.error(f"ERROR: Error saving mission summary: {e}")

    def print_performance_summary(self):
        """Print comprehensive simulation performance summary for Linearized MPC."""
        if not self.state_history:
            return

        # Save mission summary to file (no terminal output)
        self.save_mission_summary()  # type: ignore[attr-defined]

        # Print brief success message
        final_state = self.get_current_state()
        pos_error_final = np.linalg.norm(final_state[:2] - self.target_state[:2])  # type: ignore[index]
        ang_error_final = abs(self.angle_difference(self.target_state[4], final_state[4]))  # type: ignore[index]
        success = self.check_target_reached()

        print(f"\n{'=' * 60}")
        print(" MISSION COMPLETE!")
        print(f"{'=' * 60}")
        print(f"Status:            {' SUCCESS' if success else '  INCOMPLETE'}")
        print(
            f"Final Pos Error:   {pos_error_final:.4f} m (target: <{self.position_tolerance:.3f} m)"
        )
        print(
            f"Final Ang Error:   {np.degrees(ang_error_final):.2f}° (target: <{np.degrees(self.angle_tolerance):.1f}°)"
        )
        print(f"Duration:          {self.simulation_time:.1f} s")
        print(f"{'=' * 60}\n")

    # type: ignore[attr-defined]
    def reset_simulation(self):
        """Reset simulation to initial state."""
        # Reset satellite state to initial values
        self.satellite.position = np.array(self.initial_start_pos)  # type: ignore[attr-defined]
        self.satellite.velocity = np.array([0.0, 0.0])
        self.satellite.angle = self.initial_start_angle  # type: ignore[attr-defined]
        self.satellite.angular_velocity = 0.0

        # Reset simulation variables
        self.simulation_time = 0.0
        self.is_running = True
        self.last_control_update = 0.0  # type: ignore[attr-defined]
        # type: ignore[attr-defined]
        # Reset target tracking
        self.target_reached_time = None
        self.target_maintenance_time = 0.0
        self.times_lost_target = 0  # type: ignore[attr-defined]
        self.maintenance_position_errors.clear()  # type: ignore[attr-defined]
        self.maintenance_angle_errors.clear()  # type: ignore[attr-defined]
        # type: ignore[attr-defined]
        # Reset data logging
        self.state_history.clear()  # type: ignore[union-attr]
        self.control_history.clear()
        self.target_history.clear()  # type: ignore[attr-defined]
        self.mpc_solve_times.clear()  # type: ignore[union-attr]
        self.mpc_info_history.clear()

        # Reset control
        self.current_thrusters = np.zeros(8, dtype=np.float64)
        self.previous_thrusters = np.zeros(8, dtype=np.float64)
        self.satellite.active_thrusters.clear()

        # Reset trajectory
        self.satellite.trajectory = [self.satellite.position.copy()]

    def auto_generate_visualizations(self):
        """
        Automatically generate all visualizations after simulation completion.
        This replaces the need for a separate visualization script.
        """
        if UnifiedVisualizationGenerator is None:
            print(
                "  Visualization components not available. Skipping auto-visualization."
            )
            return

        if self.data_save_path is None:
            print("  No data path available. Skipping auto-visualization.")
            return

        try:
            print("\n Animation, Plots and Summary will now be generated!")

            generator = UnifiedVisualizationGenerator(
                data_directory=str(self.data_save_path.parent),
                mode="simulation",
                prefer_pandas=False,
            )

            # Override paths to use current simulation data
            generator.csv_path = self.data_save_path / "simulation_data.csv"
            generator.output_dir = self.data_save_path

            # Load the data silently
            import io
            import sys

            old_stdout = sys.stdout
            sys.stdout = io.StringIO()  # Suppress output
            try:
                generator.load_csv_data()
            finally:
                sys.stdout = old_stdout

            # Generate performance plots FIRST (faster feedback for user)
            try:
                print("\nCreating Plots...")
                plots_path = self.data_save_path / "Plots"
                print(f"Saving Plots to: {plots_path}")

                sys.stdout = io.StringIO()  # Suppress output
                try:
                    generator.generate_performance_plots()
                finally:
                    sys.stdout = old_stdout

                print(" Plots saved successfully!")
                print(f" File location: {plots_path}")
            except Exception as plots_err:
                print(f"  Performance plots generation failed: {plots_err}")

            # Generate animation SECOND (takes longer)
            try:
                print("\nCreating animation...")
                animation_path = self.data_save_path / "Simulation_animation.mp4"
                print(f"Saving animation to: {animation_path}")

                # Allow progress output but suppress other verbose output
                old_stdout = sys.stdout
                sys.stdout = ProgressSuppressor(sys.stdout)
                try:
                    generator.generate_animation()
                finally:
                    sys.stdout = old_stdout

                print(" Animation saved successfully!")
                print(f" File location: {animation_path}")
            except Exception as anim_err:
                print(f"  Animation generation failed: {anim_err}")

        except Exception as e:
            print(f" Error during auto-visualization: {e}")
            print(" You can manually run visualizations later using Visualize.py")


def create_simulation_visualizer(controller):
    """Factory function to create a SimulationVisualizationManager instance."""
    return SimulationVisualizationManager(controller)
