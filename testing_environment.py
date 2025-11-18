"""
Satellite Thruster Testing Environment with Physics Simulation

Physics-based testing platform for satellite thruster systems.
Provides realistic dynamics simulation with visualization and interactive control.

Physics simulation:
- 2D planar motion with position (x, y) and orientation (θ)
- Eight-thruster configuration with individual force vectors
- Center of mass calculation from air bearing configuration
- Moment of inertia and rotational dynamics
- Thruster force and torque calculations

Testing capabilities:
- Keyboard-based manual control for testing
- Physics-based motion simulation with configurable time steps
- Real-time visualization with matplotlib
- State history recording and playback
- Animation generation for analysis

Key features:
- Accurate center of mass from three-point air bearing system
- Individual thruster calibration support
- Integration with MPC controllers for closed-loop testing
- Configurable physics parameters (mass, inertia, friction)
- Visualization of satellite body, thrusters, and trajectories
"""

import time

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from config import SatelliteConfig


class SatelliteThrusterTester:
    """
    Satellite thruster testing environment for virtual satellite simulation.

    Simulates satellite physics with center of mass effects,
    thruster forces, and rotational dynamics. Provides keyboard-based
    interactive control of the simulated satellite for testing.

    Functions:
    - Center of mass calculation from air bearing configuration
    - Physics-based motion simulation
    - Thruster control
    - Data display
    """

    def __init__(self):
        self.satellite_size = SatelliteConfig.SATELLITE_SIZE
        self.total_mass = SatelliteConfig.TOTAL_MASS

        self.thrusters = SatelliteConfig.THRUSTER_POSITIONS

        self.thruster_forces = SatelliteConfig.THRUSTER_FORCES.copy()

        self.moment_of_inertia = SatelliteConfig.MOMENT_OF_INERTIA

        self.com_offset = np.array(SatelliteConfig.COM_OFFSET)
        print(
            f"Center of Mass offset from geometric center: ({self.com_offset[0]:.6f}, {self.com_offset[1]:.6f}) m"
        )

        self.radii = [0.215, 0.192, 0.190]  # Radial distances from geometric center [m]
        self.angles_deg = [0, 120, -120]  # Angular positions [degrees]
        self.masses = [7.358, 8.074, 8.392]  # Masses at each bearing [kg]

        angles_rad = [np.radians(angle) for angle in self.angles_deg]
        self.bearing_positions = [
            (r * np.cos(theta), r * np.sin(theta))
            for r, theta in zip(self.radii, angles_rad)
        ]

        self.thruster_colors = {
            1: "#FF6B6B",  # Red
            2: "#4ECDC4",  # Teal
            3: "#45B7D1",  # Blue
            4: "#96CEB4",  # Green
            5: "#FFEAA7",  # Yellow
            6: "#DDA0DD",  # Plum
            7: "#98D8C8",  # Mint
            8: "#F7DC6F",  # Light Yellow
        }

        # Active thrusters tracking
        self.active_thrusters = set()
        self.thruster_activation_time = {}
        self.thruster_deactivation_time = {}  # Track when OFF command was sent

        # Satellite dynamics
        self.position = np.array([0.0, 0.0])  # [x, y] in meters
        self.velocity = np.array([0.0, 0.0])  # [vx, vy] in m/s
        self.angle = 0.0  # rotation in radians
        self.angular_velocity = 0.0  # rad/s

        self.dt = SatelliteConfig.SIMULATION_DT
        self.last_time = time.time()

        # Simulation time tracking (for external simulation mode)
        self.simulation_time = 0.0

        self.external_simulation_mode = False

        # Trajectory tracking
        self.trajectory = []
        self.max_trajectory_points = 200  # Keep last 200 positions

        # Setup plot
        self.setup_plot()

    def setup_plot(self):
        """Initialize matplotlib figure and setup event handlers."""
        self.fig, (self.ax_controls, self.ax_main, self.ax_info) = plt.subplots(
            1, 3, figsize=(20, 8), gridspec_kw={"width_ratios": [2, 3, 2]}
        )

        self.ax_controls.set_xlim(0, 1)
        self.ax_controls.set_ylim(0, 1)
        self.ax_controls.axis("off")

        self.ax_main.set_xlim(-3.0, 3.0)
        self.ax_main.set_ylim(-3.0, 3.0)
        self.ax_main.set_aspect("equal")
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_xlabel("X Position (m)", fontsize=12, fontweight="bold")
        self.ax_main.set_ylabel("Y Position (m)", fontsize=12, fontweight="bold")

        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")

        # Connect event handlers
        self.fig.canvas.mpl_connect("key_press_event", self.on_key_press)
        self.fig.canvas.mpl_connect("key_release_event", self.on_key_release)

        try:
            if (
                hasattr(self.fig.canvas, "manager")
                and self.fig.canvas.manager is not None
            ):
                self.fig.canvas.manager.set_window_title(
                    "Satellite Thruster Tester - Click here and use keys 1-8"
                )
        except AttributeError:
            pass  # Window title not supported on this backend

        # Initial draw
        self.draw_satellite()
        self.update_controls_panel()
        self.update_info_panel()

        # Start animation (no cache to avoid large memory + test warnings)
        ani = FuncAnimation(
            self.fig,
            self.update_simulation,
            interval=int(self.dt * 1000),
            blit=False,
            cache_frame_data=False,
        )
        self.animation = ani  # keep strong reference to avoid GC warnings

    def get_thrust_direction(self, thruster_id):
        """Calculate thrust direction for specified thruster."""
        x, y = self.thrusters[thruster_id]

        if abs(x) > abs(y):  # On left or right face
            if x > 0:  # Right face (thrusters 1, 2)
                return np.array([-1, 0])  # Push left
            else:  # Left face (thrusters 5, 6)
                return np.array([1, 0])  # Push right
        else:  # On top or bottom face
            if y > 0:  # Top face (thrusters 7, 8)
                return np.array([0, -1])  # Push down
            else:  # Bottom face (thrusters 3, 4)
                return np.array([0, 1])  # Push up

    def get_thrust_force(self, thruster_id):
        """
        Get thrust force for specified thruster with optional realistic delays and noise.

        When USE_REALISTIC_PHYSICS is enabled, implements:
        - ON command:
          * Valve delay (20ms): No thrust while valve opens
          * Ramp-up (10ms): Linear ramp from 0 to nominal force
        - OFF command:
          * Valve delay (20ms): Full thrust continues while valve closes
          * Ramp-down (10ms): Linear ramp from nominal to 0
        - Force noise: Random variations around nominal (pressure fluctuations, valve wear)
        """
        nominal_force = self.thruster_forces.get(thruster_id, 0.0)

        # If realistic physics is disabled, return nominal force immediately if active
        if not SatelliteConfig.USE_REALISTIC_PHYSICS:
            return nominal_force if thruster_id in self.active_thrusters else 0.0

        # Check if thruster has a deactivation command pending (OFF was commanded but delay not complete)
        if thruster_id in self.thruster_deactivation_time:
            deactivation_time = self.thruster_deactivation_time[thruster_id]
            time_since_deactivation = self.simulation_time - deactivation_time

            # Phase 1: Valve closing delay - maintains full thrust while valve closes
            if time_since_deactivation < SatelliteConfig.THRUSTER_VALVE_DELAY:
                force = nominal_force
            # Phase 2: Ramp-down - linear decrease from nominal to 0
            elif time_since_deactivation < (
                SatelliteConfig.THRUSTER_VALVE_DELAY
                + SatelliteConfig.THRUSTER_RAMPUP_TIME
            ):
                rampdown_progress = (
                    time_since_deactivation - SatelliteConfig.THRUSTER_VALVE_DELAY
                ) / SatelliteConfig.THRUSTER_RAMPUP_TIME
                force = nominal_force * (1.0 - rampdown_progress)
            else:
                # Ramp-down complete - thruster fully off
                force = 0.0

            # Add noise
            noise_factor = 1.0 + np.random.normal(
                0, SatelliteConfig.THRUSTER_FORCE_NOISE_STD
            )
            return force * noise_factor

        # Check if thruster is active (ON command)
        if thruster_id not in self.active_thrusters:
            return 0.0

        # Calculate time since activation
        activation_time = self.thruster_activation_time.get(
            thruster_id, self.simulation_time
        )
        time_since_activation = self.simulation_time - activation_time

        # Phase 1: Valve opening delay - no thrust while valve opens
        if time_since_activation < SatelliteConfig.THRUSTER_VALVE_DELAY:
            return 0.0

        # Phase 2: Ramp-up - linear increase to nominal force
        rampup_end = (
            SatelliteConfig.THRUSTER_VALVE_DELAY + SatelliteConfig.THRUSTER_RAMPUP_TIME
        )
        if time_since_activation < rampup_end:
            # Linear ramp from 0 to nominal over RAMPUP_TIME
            rampup_progress = (
                time_since_activation - SatelliteConfig.THRUSTER_VALVE_DELAY
            ) / SatelliteConfig.THRUSTER_RAMPUP_TIME
            force = nominal_force * rampup_progress
        else:
            # Phase 3: Full thrust with noise
            force = nominal_force

        # Add realistic force variations (pressure fluctuations, valve wear)
        noise_factor = 1.0 + np.random.normal(
            0, SatelliteConfig.THRUSTER_FORCE_NOISE_STD
        )
        return force * noise_factor

    def calculate_forces_and_torques(self):
        """Calculate net force and torque from active thrusters and gravity effects about center of mass."""
        net_force = np.array([0.0, 0.0])
        net_torque = 0.0

        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

        # Calculate thruster forces and torques
        for thruster_id in self.active_thrusters:
            # Get thruster position relative to geometric center
            thruster_pos_geometric = np.array(self.thrusters[thruster_id])
            thrust_dir = self.get_thrust_direction(thruster_id)
            thrust_force = self.get_thrust_force(
                thruster_id
            )  # Use individual thruster force

            # Transform thrust direction to global coordinates
            global_thrust_dir = rotation_matrix @ thrust_dir

            net_force += thrust_force * global_thrust_dir

            # Calculate torque about center of mass
            # First, get thruster position relative to COM in body coordinates
            thruster_pos_from_com = thruster_pos_geometric - self.com_offset

            # Rotate thruster position to global coordinates
            global_thruster_pos_from_com = rotation_matrix @ thruster_pos_from_com

            torque = np.cross(
                global_thruster_pos_from_com, thrust_force * global_thrust_dir
            )
            net_torque += torque

        # Add gravitational restoring torque due to air bearing support
        # In the testing environment, gravity acts downward but air bearings provide upward support
        gravitational_torque = self.calculate_gravitational_torque(rotation_matrix)
        net_torque += gravitational_torque

        return net_force, net_torque

    def calculate_gravitational_torque(self, rotation_matrix):
        """Calculate gravity restoring torque due to COM offset in test environment."""
        # In the real test setup, gravity acts downward and air bearings provide upward support

        # Get current COM position in global coordinates
        global_com_offset = rotation_matrix @ self.com_offset

        # The restoring torque acts to align the COM vertically below the support point

        gravity = 9.81  # m/s²

        # where h is the effective height of COM above the bearing plane

        effective_height = 0.03  # meters (conservative estimate for 290mm cube)


        # Calculate the current direction of the COM offset in global coordinates
        com_offset_magnitude = np.linalg.norm(global_com_offset)

        if com_offset_magnitude > 1e-6:  # Avoid division by zero
            pass

            # Calculate the cross product to get the magnitude and direction of restoring torque
            # The restoring force acts to pull the COM toward the vertical position

            # Calculate the torque using the cross product of position and restoring force
            # The restoring force is proportional to the horizontal component of the COM offset
            horizontal_offset = global_com_offset[
                0
            ]  # x-component (horizontal displacement)

            # The restoring torque opposes the angular displacement
            restoring_constant = self.total_mass * gravity * effective_height

            # For small angles, θ ≈ horizontal_offset / effective_height
            # But we want the torque to be proportional to the actual angular deviation
            angular_displacement = horizontal_offset / effective_height

            restoring_torque = -restoring_constant * angular_displacement

            # Apply a scaling factor to prevent numerical instability
            scaling_factor = 0.01  # Much smaller factor for stability
            restoring_torque *= scaling_factor
        else:
            restoring_torque = 0.0

        return restoring_torque

    def update_simulation(self, frame):
        """Update satellite physics and visualization"""
        current_time = time.time()
        self.last_time = current_time

        dt = self.dt

        net_force, net_torque = self.calculate_forces_and_torques()

        acceleration = net_force / self.total_mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        angular_acceleration = net_torque / self.moment_of_inertia
        self.angular_velocity += angular_acceleration * dt
        self.angle += self.angular_velocity * dt

        # Update trajectory
        self.trajectory.append(self.position.copy())
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)

        # Apply damping (air resistance/friction)
        if SatelliteConfig.USE_REALISTIC_PHYSICS:
            # Realistic damping: F_drag = -b*v, τ_drag = -c*ω
            drag_force = -SatelliteConfig.LINEAR_DAMPING_COEFF * self.velocity
            drag_accel = drag_force / self.total_mass
            self.velocity += drag_accel * dt

            drag_torque = (
                -SatelliteConfig.ROTATIONAL_DAMPING_COEFF * self.angular_velocity
            )
            drag_angular_accel = drag_torque / self.moment_of_inertia
            self.angular_velocity += drag_angular_accel * dt

            # Add random disturbances if enabled
            if SatelliteConfig.ENABLE_RANDOM_DISTURBANCES:
                disturbance_force = np.random.normal(
                    0, SatelliteConfig.DISTURBANCE_FORCE_STD, 2
                )
                self.velocity += (disturbance_force / self.total_mass) * dt

                disturbance_torque = np.random.normal(
                    0, SatelliteConfig.DISTURBANCE_TORQUE_STD
                )
                self.angular_velocity += (
                    disturbance_torque / self.moment_of_inertia
                ) * dt
        else:
            # Simple damping (idealized model)
            damping_factor = 0.995
            self.velocity *= damping_factor
            self.angular_velocity *= damping_factor

        if not self.external_simulation_mode:
            self.ax_main.clear()
            self.draw_satellite()
            self.update_controls_panel()
            self.update_info_panel()

        return []

    def set_title(self, title):
        """Set custom title for external simulation integration."""
        self.title_text = title
        self.ax_main.set_title(self.title_text, fontsize=14, fontweight="bold")

    def draw_satellite(self):
        """Draw satellite with current position and orientation."""
        if self.external_simulation_mode:
            return

        # Setup main plot
        self.ax_main.set_xlim(-3.0, 3.0)
        self.ax_main.set_ylim(-3.0, 3.0)
        self.ax_main.set_aspect("equal")
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_xlabel("X Position (m)", fontsize=12, fontweight="bold")
        self.ax_main.set_ylabel("Y Position (m)", fontsize=12, fontweight="bold")

        if hasattr(self, "title_text"):
            self.ax_main.set_title(self.title_text, fontsize=14, fontweight="bold")

        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

        bearing_colors = [
            "#8B0000",
            "#006400",
            "#00008B",
        ]  # Dark red, dark green, dark blue

        for _i, (local_pos, mass, color) in enumerate(
            zip(self.bearing_positions, self.masses, bearing_colors)
        ):
            global_pos = rotation_matrix @ np.array(local_pos) + self.position

            bearing_size = 30 + mass * 3
            self.ax_main.scatter(
                global_pos[0],
                global_pos[1],
                s=bearing_size,
                c=color,
                marker="s",
                alpha=0.6,
                edgecolors="black",
                linewidth=1,
            )

        # LAYER 2: Draw trajectory trail
        if len(self.trajectory) > 1:
            trajectory_array = np.array(self.trajectory)
            alphas = np.linspace(0.1, 0.6, len(self.trajectory))

            # Draw trajectory as individual line segments with varying alpha
            for i in range(len(self.trajectory) - 1):
                self.ax_main.plot(
                    [trajectory_array[i, 0], trajectory_array[i + 1, 0]],
                    [trajectory_array[i, 1], trajectory_array[i + 1, 1]],
                    "b-",
                    alpha=alphas[i],
                    linewidth=1,
                )

        com_global = rotation_matrix @ self.com_offset + self.position
        for _i, (local_pos, _mass, color) in enumerate(
            zip(self.bearing_positions, self.masses, bearing_colors)
        ):
            global_pos = rotation_matrix @ np.array(local_pos) + self.position
            self.ax_main.plot(
                [com_global[0], global_pos[0]],
                [com_global[1], global_pos[1]],
                ":",
                color=color,
                alpha=0.4,
                linewidth=1,
            )

        square_corners = np.array(
            [
                [-self.satellite_size / 2, -self.satellite_size / 2],
                [self.satellite_size / 2, -self.satellite_size / 2],
                [self.satellite_size / 2, self.satellite_size / 2],
                [-self.satellite_size / 2, self.satellite_size / 2],
            ]
        )

        rotated_corners = np.array(
            [rotation_matrix @ corner for corner in square_corners]
        )
        translated_corners = rotated_corners + self.position

        satellite_patch = patches.Polygon(
            translated_corners,
            linewidth=3,
            edgecolor="black",
            facecolor="lightgray",
            alpha=0.7,
        )
        self.ax_main.add_patch(satellite_patch)

        if np.linalg.norm(self.com_offset) > 0.001:  # Only if offset > 1mm
            self.ax_main.plot(
                [self.position[0], com_global[0]],
                [self.position[1], com_global[1]],
                "r--",
                linewidth=2,
                alpha=0.7,
            )

        for thruster_id, local_pos in self.thrusters.items():
            # Transform thruster position to global coordinates
            global_pos = rotation_matrix @ np.array(local_pos) + self.position

            # Get thruster color
            color = self.thruster_colors[thruster_id]

            is_active = thruster_id in self.active_thrusters

            # Draw thruster as colored circle
            if is_active:
                thruster_size = 20  # Smaller but bright when active
                alpha = 1.0
            else:
                thruster_size = 20  # Much smaller when inactive
                alpha = 0.3

            self.ax_main.scatter(
                global_pos[0], global_pos[1], s=thruster_size, c=color, alpha=alpha
            )

    def update_controls_panel(self):
        """Update control instructions panel (left side)."""
        if self.external_simulation_mode:
            return

        self.ax_controls.clear()
        self.ax_controls.set_xlim(0, 1)
        self.ax_controls.set_ylim(0, 1)
        self.ax_controls.axis("off")

        # Control instructions
        controls_text = """CONTROLS

THRUSTER MANUAL ACTIVATION:
Press & Hold Keys 1-8
Release to Deactivate

KEY MAPPING:
1 → Thruster 1 (Right Top)
2 → Thruster 2 (Right Bottom)
3 → Thruster 3 (Bottom Right)
4 → Thruster 4 (Bottom Left)
5 → Thruster 5 (Left Bottom)
6 → Thruster 6 (Left Top)
7 → Thruster 7 (Top Left)
8 → Thruster 8 (Top Right)

SIMULATION:
Space → Reset Position
Close Window / Q / ESC → Exit
"""

        self.ax_controls.text(
            0.05,
            0.95,
            controls_text,
            fontsize=9,
            fontfamily="monospace",
            verticalalignment="top",
            horizontalalignment="left",
            bbox=dict(
                boxstyle="round,pad=0.8",
                facecolor="lightgreen",
                alpha=0.9,
                edgecolor="darkgreen",
                linewidth=1,
            ),
        )

    def update_info_panel(self):
        """Update the information panel with status and color legend"""
        if self.external_simulation_mode:
            return

        self.ax_info.clear()
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")

        # Calculate current forces and torques
        net_force, net_torque = self.calculate_forces_and_torques()

        cos_angle = np.cos(self.angle)
        sin_angle = np.sin(self.angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])
        gravitational_torque = self.calculate_gravitational_torque(rotation_matrix)
        thruster_torque = net_torque - gravitational_torque

        # Create info text with COM information
        com_distance = np.linalg.norm(self.com_offset)
        speed = np.linalg.norm(self.velocity)
        kinetic_energy = (
            0.5 * self.total_mass * speed**2
            + 0.5 * self.moment_of_inertia * self.angular_velocity**2
        )

        info_text = f"""SATELLITE STATUS:

Position: ({self.position[0]:.3f}, {self.position[1]:.3f}) m
Velocity: ({self.velocity[0]:.3f}, {self.velocity[1]:.3f}) m/s
Speed: {speed:.3f} m/s
Angle: {np.degrees(self.angle):.1f}°
Angular Velocity: {np.degrees(self.angular_velocity):.1f}°/s

ENERGY & DYNAMICS:

Kinetic Energy: {kinetic_energy:.4f} J
Linear KE: {0.5 * self.total_mass * speed**2:.4f} J
Rotational KE: {0.5 * self.moment_of_inertia * self.angular_velocity**2:.4f} J

CENTER OF MASS:

COM Offset: ({self.com_offset[0]:.3f}, {self.com_offset[1]:.3f}) m
COM Distance: {com_distance:.3f} m from geometric center
Moment of Inertia: {self.moment_of_inertia:.3f} kg⋅m²

FORCES & TORQUES:

Net Force: ({net_force[0]:.3f}, {net_force[1]:.3f}) N
Force Magnitude: {np.linalg.norm(net_force):.3f} N
Thruster Torque: {thruster_torque:.4f} N⋅m
Gravitational Torque: {gravitational_torque:.4f} N⋅m
Total Torque: {net_torque:.4f} N⋅m (about COM)

ACTIVE THRUSTERS:

{', '.join([f'T{tid}' for tid in sorted(self.active_thrusters)]) if self.active_thrusters else 'None'}

CONTROLS:

Press 1-8: Fire thrusters
Space: Reset simulation"""

        self.ax_info.text(
            0.05,
            0.95,
            info_text,
            fontsize=10,
            fontfamily="monospace",
            verticalalignment="top",
            bbox=dict(
                boxstyle="round,pad=0.8",
                facecolor="lightblue",
                alpha=0.9,
                edgecolor="navy",
                linewidth=1,
            ),
        )

        legend_y_start = 0.35

        # Left side: Thruster Colors
        self.ax_info.text(
            0.05, legend_y_start, "THRUSTER COLORS:", fontsize=11, fontweight="bold"
        )

        for i, (thruster_id, color) in enumerate(self.thruster_colors.items()):
            row = i // 2  # Two columns
            col = i % 2
            x_pos = 0.05 + col * 0.2  # Closer spacing
            y_pos = legend_y_start - 0.05 - row * 0.04

            self.ax_info.scatter(
                x_pos, y_pos, s=80, c=color, edgecolors="black", linewidth=1
            )
            # Add thruster ID text
            self.ax_info.text(
                x_pos + 0.04,
                y_pos,
                f"T{thruster_id}",
                fontsize=10,
                verticalalignment="center",
                fontweight="bold",
            )

        legend_elements_x = 0.55  # Right side of the panel
        self.ax_info.text(
            legend_elements_x,
            legend_y_start,
            "KEY ELEMENTS:",
            fontsize=11,
            fontweight="bold",
        )

        elements = [
            ("O", "black", "Geometric Center"),
            ("O", "red", "Center of Mass"),
            ("[  ]", "#8B0000", "Air Bearings"),
        ]

        for i, (symbol, color, label) in enumerate(elements):
            y_pos = legend_y_start - 0.05 - i * 0.04
            self.ax_info.text(
                legend_elements_x,
                y_pos,
                symbol,
                fontsize=12,
                color=color,
                fontweight="bold",
            )
            self.ax_info.text(
                legend_elements_x + 0.04,
                y_pos,
                label,
                fontsize=10,
                verticalalignment="center",
            )

    def on_key_press(self, event):
        """Handle key press events"""
        if event.key in ["1", "2", "3", "4", "5", "6", "7", "8"]:
            thruster_id = int(event.key)
            self.active_thrusters.add(thruster_id)
            self.thruster_activation_time[thruster_id] = time.time()
            print(f"Thruster {thruster_id} ACTIVATED")

        elif event.key == " ":  # Space bar to reset
            self.reset_simulation()
            print("Simulation RESET")

    def on_key_release(self, event):
        """Handle key release events"""
        if event.key in ["1", "2", "3", "4", "5", "6", "7", "8"]:
            thruster_id = int(event.key)
            if thruster_id in self.active_thrusters:
                self.active_thrusters.remove(thruster_id)
                activation_duration = time.time() - self.thruster_activation_time.get(
                    thruster_id, 0
                )
                print(
                    f"Thruster {thruster_id} DEACTIVATED (fired for {activation_duration:.2f}s)"
                )

    def reset_simulation(self):
        """Reset satellite to initial state"""
        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.angle = 0.0
        self.angular_velocity = 0.0
        self.active_thrusters.clear()
        self.thruster_activation_time.clear()
        self.trajectory.clear()  # Clear trajectory trail

    def run(self):
        """Start the testing environment"""
        print("=" * 60)
        print("SATELLITE THRUSTER TESTING ENVIRONMENT")
        print("=" * 60)
        print("Controls:")
        print("  Press keys 1-8: Activate corresponding thruster")
        print("  Release keys 1-8: Deactivate thruster")
        print("  Press Space: Reset simulation")
        print("  Close window: Exit")
        print("\\nMake sure to click on the plot window to focus it!")
        print("=" * 60)

        plt.show()


def main():
    try:
        # Create and run the testing environment
        tester = SatelliteThrusterTester()
        tester.run()
    except KeyboardInterrupt:
        print("\\nTesting environment closed.")
    except Exception as e:
        print(f"\\nError: {e}")
        print("Testing environment failed to start.")


if __name__ == "__main__":
    main()
