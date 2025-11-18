"""
Simulation Test Mode Launchers for Satellite Control System

Interactive menu system and mode launchers for simulation testing.
Provides command-line interface for configuring and running simulation missions.

Supported test modes:
- Point-to-point navigation: Single target with position and orientation
- Multi-point navigation: Sequential waypoint following
- Custom initial conditions: User-specified start position, orientation, velocities

User input helpers:
- Position coordinates with validation
- Orientation angles (degrees, converted to radians)
- Initial velocities (linear and angular)
- Obstacle configuration (position and radius)
- Mission parameter validation

Key features:
- Interactive command-line prompts with defaults
- Input validation and error handling
- Integration with SatelliteMPCLinearizedSimulation
- Modular design for easy extension
- Consistent with real hardware test modes
"""

from typing import Optional, Tuple

import numpy as np

from config import SatelliteConfig
from simulation import SatelliteMPCLinearizedSimulation


def get_user_position(
    position_type: str, default_pos: Optional[Tuple[float, float]] = None
) -> Tuple[float, float]:
    """
    Get position coordinates from user input.

    Args:
        position_type: Description of position (e.g., "starting", "target")
        default_pos: Default position if provided

    Returns:
        tuple: (x, y) coordinates
    """
    if default_pos:
        print(f"\nEnter {position_type} position (default: {default_pos}):")
    else:
        print(f"\nEnter {position_type} position:")

    try:
        x_input = input("  X coordinate (m): ").strip()
        y_input = input("  Y coordinate (m): ").strip()

        if default_pos and not x_input and not y_input:
            return default_pos

        x = float(x_input) if x_input else (default_pos[0] if default_pos else 0.0)
        y = float(y_input) if y_input else (default_pos[1] if default_pos else 0.0)

        return (x, y)

    except ValueError:
        print("Invalid input. Using default or zero coordinates.")
        return default_pos if default_pos else (0.0, 0.0)


def get_user_orientation(
    orientation_type: str, default_angle: Optional[float] = None
) -> float:
    """
    Get orientation angle from user input.

    Args:
        orientation_type: Description of orientation (e.g., "starting", "target")
        default_angle: Default angle in radians if provided

    Returns:
        float: Angle in radians
    """
    try:
        default_deg = np.degrees(default_angle) if default_angle is not None else 0.0

        print(f"\nEnter {orientation_type} orientation:")
        angle_input = input(f"  Angle (degrees, default: {default_deg:.1f}°): ").strip()

        if default_angle is not None and not angle_input:
            return default_angle

        angle_deg = float(angle_input) if angle_input else default_deg
        return np.radians(angle_deg)

    except ValueError:
        print("Invalid angle input. Using default or zero.")
        return default_angle if default_angle is not None else 0.0


def get_user_velocities(
    default_vx: float = 0.0, default_vy: float = 0.0, default_omega: float = 0.0
) -> Tuple[float, float, float]:
    """
    Get initial velocity values from user input.

    Args:
        default_vx: Default X velocity in m/s
        default_vy: Default Y velocity in m/s
        default_omega: Default angular velocity in rad/s

    Returns:
        tuple: (vx, vy, omega) velocities
    """
    print("\n INITIAL VELOCITIES")
    print("Enter initial velocity state (default: all zeros for stationary start)")
    print(" Tip: Use Extract_Initial_Velocities.py to extract from real test data")

    try:
        vx_input = input(f"  X velocity (m/s, default: {default_vx:.3f}): ").strip()
        vy_input = input(f"  Y velocity (m/s, default: {default_vy:.3f}): ").strip()
        omega_input = input(
            f"  Angular velocity (rad/s, default: {default_omega:.3f}): "
        ).strip()

        vx = float(vx_input) if vx_input else default_vx
        vy = float(vy_input) if vy_input else default_vy
        omega = float(omega_input) if omega_input else default_omega

        return (vx, vy, omega)

    except ValueError:
        print("Invalid velocity input. Using default values (zero).")
        return (default_vx, default_vy, default_omega)


def get_user_obstacles() -> list:
    """
    Get obstacle configuration from user input.

    Returns:
        List of (x, y, radius) tuples defining obstacles
    """
    print("\n OBSTACLE CONFIGURATION")
    print("Add obstacles that the satellite should avoid (minimum 0.5m clearance)")

    obstacles = []

    while True:
        add_obstacle = input("\nAdd an obstacle? (y/n): ").strip().lower()
        if add_obstacle not in ["y", "yes"]:
            break

        try:
            print(f"\n--- Obstacle {len(obstacles) + 1} ---")
            x = float(input("  X position (m): ").strip())
            y = float(input("  Y position (m): ").strip())

            # Get radius with default
            radius_input = input(
                f"  Radius (m, default {SatelliteConfig.DEFAULT_OBSTACLE_RADIUS:.1f}): "
            ).strip()
            radius = (
                float(radius_input)
                if radius_input
                else SatelliteConfig.DEFAULT_OBSTACLE_RADIUS
            )

            # Validate radius
            if radius <= 0:
                print("   Radius must be positive. Using default.")
                radius = SatelliteConfig.DEFAULT_OBSTACLE_RADIUS
            elif radius < 0.1:
                print("    Warning: Very small obstacle radius. Using 0.1m minimum.")
                radius = 0.1
            elif radius > 2.0:
                print("    Warning: Very large obstacle radius. Using 2.0m maximum.")
                radius = 2.0

            obstacles.append((x, y, radius))
            print(f"   Added obstacle: ({x:.2f}, {y:.2f}) m, radius {radius:.2f} m")

        except ValueError:
            print("   Invalid input. Skipping this obstacle.")
            continue

        if len(obstacles) >= 10:  # Safety limit
            print("    Maximum 10 obstacles reached.")
            break

    if obstacles:
        print(f"\n Total obstacles configured: {len(obstacles)}")
        for i, (x, y, radius) in enumerate(obstacles, 1):
            print(f"  Obstacle {i}: ({x:.2f}, {y:.2f}) m, radius {radius:.2f} m")

        # Configure obstacles in SatelliteConfig
        SatelliteConfig.set_obstacles(obstacles)
    else:
        print("\n No obstacles configured. Satellite will use direct paths.")
        SatelliteConfig.clear_obstacles()

    return obstacles


def confirm_and_run_simulation(
    sim_type: str = "Point-to-Point",
    start_vx: float = 0.0,
    start_vy: float = 0.0,
    start_omega: float = 0.0,
) -> None:
    """
    Ask user to confirm parameters and run the simulation.
    Auto-generates all visualizations after simulation completion.

    Args:
        sim_type: Type of simulation for display purposes
        start_vx: Initial X velocity in m/s
        start_vy: Initial Y velocity in m/s
        start_omega: Initial angular velocity in rad/s
    """
    print(f"\n{'=' * 50}")
    print(f"  {sim_type.upper()} SIMULATION READY")
    print(f"{'=' * 50}")

    proceed = input("Proceed with simulation? (y/n): ").strip().lower()

    if proceed != "y" and proceed != "yes":
        print("Simulation cancelled.")
        return

    # Run simulation in data-only mode and auto-generate all visualizations
    print(f"\nRunning {sim_type} simulation with auto-visualization...")
    print("CSV data will be saved and all plots/animation will be auto-generated.")

    sim = SatelliteMPCLinearizedSimulation(
        start_vx=start_vx, start_vy=start_vy, start_omega=start_omega
    )
    sim.run_simulation(show_animation=False)


def get_simulation_type_choice() -> str:
    """Get user choice for simulation type (data only vs animation)."""
    print("\nChoose simulation type:")
    print("1. Data only (CSV file) - Recommended")
    print("2. With animation (slower)")

    choice = input("Enter choice (1-2): ")

    def poll_distance_sensors(self):
        """
        Polls distance sensors and updates internal state.
        Replace this with actual sensor polling if available.
        """
        pass  # No-op for now

    return choice


def run_point_to_point_mode():
    """
    Mode 1: Point-to-Point (Go to target and stay there)

    For simulation: User defines starting position, starting orientation,
                   target position, and target orientation
    For real: Current position is starting, user defines target position and orientation

    Satellite will navigate to target position and orientation.
    """
    print("\n Point-to-Point Navigation")
    print("The satellite will navigate to the target position and orientation.")

    # Get starting position
    start_pos = get_user_position("starting", SatelliteConfig.DEFAULT_START_POS)
    print(f"Starting position set: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m")

    # Get starting orientation
    start_angle = get_user_orientation("starting", SatelliteConfig.DEFAULT_START_ANGLE)
    print(f"Starting orientation set: {np.degrees(start_angle):.1f}°")

    # Get initial velocities
    start_vx, start_vy, start_omega = get_user_velocities()
    print(
        f"Initial velocities set: vx={start_vx:.6f} m/s, vy={start_vy:.6f} m/s, ω={start_omega:.6f} rad/s ({np.degrees(start_omega):.3f} deg/s)"
    )

    # Get target position
    target_pos = get_user_position("target", SatelliteConfig.DEFAULT_TARGET_POS)
    print(f"Target position set: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m")

    # Get target orientation
    target_angle = get_user_orientation("target", SatelliteConfig.DEFAULT_TARGET_ANGLE)
    print(f"Target orientation set: {np.degrees(target_angle):.1f}°")

    # Get obstacle configuration
    get_user_obstacles()

    # Update configuration
    SatelliteConfig.DEFAULT_START_POS = start_pos
    SatelliteConfig.DEFAULT_START_ANGLE = start_angle
    SatelliteConfig.DEFAULT_TARGET_POS = target_pos
    SatelliteConfig.DEFAULT_TARGET_ANGLE = target_angle

    # Display mission summary
    distance = np.linalg.norm(np.array(target_pos) - np.array(start_pos))
    angle_diff = np.degrees(abs(target_angle - start_angle))

    print("\nMission Summary:")
    print(f"  Distance to target: {distance:.2f} m")
    print(f"  Angular change required: {angle_diff:.1f}°")
    print("  Mission type: Navigate to position and orientation, then hold")

    confirm_and_run_simulation("Point-to-Point", start_vx, start_vy, start_omega)


def run_multi_point_mode():
    """
    Mode 2: Point-to-Point (multiple points)

    Satellite visits multiple targets in sequence. After stabilizing at each target
    for 3 seconds within tolerance limits, it moves to the next target.
    User defines starting position/orientation and each target position/orientation.
    """
    print("\n Multi-Point Navigation")
    print("The satellite will visit multiple targets in sequence.")
    print(
        "After stabilizing at each target for 3 seconds, it moves to the next target."
    )

    SatelliteConfig.set_multi_point_mode(True)  # Enable multi-point mode

    # Get starting position
    start_pos = get_user_position("starting", SatelliteConfig.DEFAULT_START_POS)
    print(f"Starting position set: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m")

    # Get starting orientation
    start_angle = get_user_orientation("starting", SatelliteConfig.DEFAULT_START_ANGLE)
    print(f"Starting orientation set: {np.degrees(start_angle):.1f}°")

    # Get initial velocities
    start_vx, start_vy, start_omega = get_user_velocities()
    print(
        f"Initial velocities set: vx={start_vx:.6f} m/s, vy={start_vy:.6f} m/s, ω={start_omega:.6f} rad/s ({np.degrees(start_omega):.3f} deg/s)"
    )

    # Collect target points and orientations
    target_points = []
    target_angles = []
    target_count = 1

    while True:
        print(f"\n--- Target Point {target_count} ---")
        target_pos = get_user_position(f"target {target_count}")
        target_angle = get_user_orientation(f"target {target_count}")

        target_points.append(target_pos)
        target_angles.append(target_angle)

        print(
            f"Target {target_count} set: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
        )

        add_another = input("\nAdd another target point? (y/n): ").strip().lower()
        if add_another != "y" and add_another != "yes":
            break

        target_count += 1

        if target_count > 10:  # Safety limit
            print("Maximum of 10 target points reached.")
            break

    # Get obstacle configuration
    get_user_obstacles()

    # Display mission summary
    print(f"\n{'=' * 50}")
    print("  MULTI-POINT MISSION SUMMARY")
    print(f"{'=' * 50}")
    print(
        f"Starting: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f}°"
    )
    print(f"Number of targets: {len(target_points)}")
    print("Stabilization time at each target: 3 seconds")

    total_distance = 0
    current_pos = np.array(start_pos)

    for i, (target_pos, target_angle) in enumerate(
        zip(target_points, target_angles), 1
    ):
        target_array = np.array(target_pos)
        segment_distance = np.linalg.norm(target_array - current_pos)
        total_distance += segment_distance

        print(
            f"  Target {i}: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}° - "
            f"Distance: {segment_distance:.2f} m"
        )
        current_pos = target_array

    print(f"Total mission distance: {total_distance:.2f} m")
    print("Mission behavior: Navigate → Stabilize 3s → Next target")

    SatelliteConfig.DEFAULT_START_POS = start_pos
    SatelliteConfig.DEFAULT_START_ANGLE = start_angle

    if target_points:
        SatelliteConfig.DEFAULT_TARGET_POS = target_points[0]
        SatelliteConfig.DEFAULT_TARGET_ANGLE = target_angles[0]

        # Configure waypoint mode
        SatelliteConfig.ENABLE_WAYPOINT_MODE = True
        SatelliteConfig.WAYPOINT_TARGETS = target_points.copy()
        SatelliteConfig.WAYPOINT_ANGLES = target_angles.copy()
        SatelliteConfig.CURRENT_TARGET_INDEX = 0
        SatelliteConfig.TARGET_STABILIZATION_START_TIME = None
        SatelliteConfig.TARGET_HOLD_TIME = 3.0

    confirm_and_run_simulation("Waypoint Navigation", start_vx, start_vy, start_omega)


def main():
    """
    Enhanced menu system for satellite control simulation using unified Mission module.

    Modes:
    1. Waypoint Navigation (single or multiple waypoints)
    2. Shape Following (circles, rectangles, triangles, hexagons, custom DXF)

    Also supports non-interactive runs via CLI flags.
    """
    import argparse

    # Import simulation class and mission manager (inside function to avoid circular imports)
    from simulation import SatelliteMPCLinearizedSimulation

    try:
        from mission import MissionManager
    except ImportError:
        MissionManager = None

    parser = argparse.ArgumentParser(description="Linearized MPC Simulation")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Run a non-interactive simulation using Config defaults and auto-generate outputs",
    )
    parser.add_argument(
        "--no-anim",
        action="store_true",
        help="Do not show live animation window during the run",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override max simulation time in seconds (defaults from Config)",
    )
    args, unknown = parser.parse_known_args()

    # Non-interactive quick run
    if args.auto:
        try:
            if args.duration is not None and args.duration > 0:
                SatelliteConfig.MAX_SIMULATION_TIME = args.duration
                print(
                    f"⏱  Overriding max simulation time to {args.duration:.2f}s for this run"
                )

            print("\n  Starting non-interactive simulation using Config defaults...")
            sim = SatelliteMPCLinearizedSimulation()
            # argparse converts '--no-anim' to 'no_anim'
            sim.run_simulation(show_animation=not args.no_anim)
        except KeyboardInterrupt:
            print("\n\nSimulation cancelled by user.")
        except Exception as e:
            print(f"\n Error during simulation: {e}")
            print("Exiting simulation.")
        return

    # Interactive mission-driven flow
    try:
        if MissionManager is None:
            print(" Mission module not available. Please check Mission.py file.")
            return

        manager = MissionManager("simulation")

        # Show menu and get user choice
        mode_choice = manager.show_mission_menu()

        # Run the selected mission
        config = manager.run_selected_mission(mode_choice)

        if config:
            print("\n Mission configured successfully!")
            print(f"Mission type: {config['mission_type']}")
            print(
                f"\nRunning {config['mission_type'].replace('_', '-').title()} simulation with auto-visualization..."
            )
            print(
                "CSV data will be saved and all plots/animation will be auto-generated."
            )

            # Extract all simulation parameters from config
            start_pos = config.get("start_pos", None)
            start_angle = config.get("start_angle", None)
            start_vx = config.get("start_vx", 0.0)
            start_vy = config.get("start_vy", 0.0)
            start_omega = config.get("start_omega", 0.0)

            sim = SatelliteMPCLinearizedSimulation(
                start_pos=start_pos,
                start_angle=start_angle,
                start_vx=start_vx,
                start_vy=start_vy,
                start_omega=start_omega,
            )
            sim.run_simulation(show_animation=False)
        else:
            print("\n Mission configuration cancelled.")

    except KeyboardInterrupt:
        print("\n\nSimulation cancelled by user.")
        return
    except Exception as e:
        print(f"\n Error: {e}")
        import traceback

        traceback.print_exc()
        print("Exiting simulation.")
        return


if __name__ == "__main__":
    main()
