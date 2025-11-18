"""
Real Hardware Test Mode Launchers

Entry points for different mission modes in real hardware testing.
Handles user input collection, system configuration, and test execution.

Supported mission modes:
- Point-to-Point: Navigate to single target position with orientation
- Waypoint Navigation: Sequential navigation through multiple waypoints
- Shape Following: Follow moving targets along geometric paths
  - Predefined shapes: Circle, Rectangle, Triangle, Hexagon
  - Custom shapes: DXF file import

Key features:
- Interactive command-line input for mission parameters
- Configuration validation before test execution
- Integration with real hardware controller
- Obstacle configuration support
"""

import numpy as np

from config import SatelliteConfig


def run_point_to_point_mode():
    """Mode 1: Point-to-Point (Real hardware testing)"""
    # Import here to avoid circular dependency
    from real import RealSatelliteMPCLinearized

    print("\n POINT-TO-POINT MODE")

    print("\nEnter target position and orientation:")
    try:
        target_x = float(input("Target X position (meters): "))
        target_y = float(input("Target Y position (meters): "))
        target_yaw_deg = float(input("Target Yaw angle (degrees): "))

        # Convert degrees to radians
        target_yaw_rad = np.radians(target_yaw_deg)

        print("\nTarget set to:")
        print(f"  Position: ({target_x:.3f}, {target_y:.3f}) meters")
        print(f"  Yaw: {target_yaw_deg:.1f}° ({target_yaw_rad:.3f} rad)")

    except ValueError:
        print("Invalid input. Using default target position.")
        target_x, target_y = 0.8, 0.5
        target_yaw_deg = 30.0
        target_yaw_rad = np.radians(target_yaw_deg)
        print(f"Default target: ({target_x}, {target_y}) m, {target_yaw_deg}°")
    except KeyboardInterrupt:
        print("\nTest cancelled by user.")
        return

    # Offer to add obstacles
    SatelliteConfig.clear_obstacles()
    add_obs = input("\nAdd obstacle? (y/n): ").strip().lower()
    while add_obs == "y":
        try:
            obs_x = float(input("  Obstacle X position (meters): "))
            obs_y = float(input("  Obstacle Y position (meters): "))
            obs_r = float(input("  Obstacle radius (meters): "))
            SatelliteConfig.add_obstacle(obs_x, obs_y, obs_r)
            print(f"  Obstacle added: ({obs_x:.2f}, {obs_y:.2f}), r={obs_r:.2f}")
        except Exception:
            print("  Invalid input, skipping.")
        add_obs = input("Add another obstacle? (y/n): ").strip().lower()
    if SatelliteConfig.get_obstacles():
        SatelliteConfig.OBSTACLES_ENABLED = True
        print(
            f"Obstacles enabled: {len(SatelliteConfig.get_obstacles())} obstacles configured."
        )
    else:
        SatelliteConfig.OBSTACLES_ENABLED = False

    # Confirm before starting
    confirm = input("\nProceed with POINT-TO-POINT test? (y/n): ").lower()
    if confirm != "y":
        print("Test cancelled.")
        return

    print("\nUsing Serial communication (COM20, 115200 baud)")

    # Create and run test with Serial interface
    real_test = RealSatelliteMPCLinearized(
        target_pos=(target_x, target_y),
        target_angle=target_yaw_rad,
        hardware_type="serial",
        port="COM20",
        baudrate=115200,
    )

    real_test.run_real_test(show_visualization=True)


def run_multi_point_mode():
    """Mode 2: Multi-Point Navigation (Real hardware testing)"""
    from real import RealSatelliteMPCLinearized

    print("\n MULTI-POINT NAVIGATION MODE")
    print("The satellite will visit multiple targets in sequence.")
    print(
        "After stabilizing at each target for 3 seconds, it moves to the next target."
    )

    # Collect target points and orientations
    target_points = []
    target_angles = []
    target_count = 1

    while True:
        print(f"\n--- Target Point {target_count} ---")
        try:
            target_x = float(input(f"Target {target_count} X position (meters): "))
            target_y = float(input(f"Target {target_count} Y position (meters): "))
            target_yaw_deg = float(
                input(f"Target {target_count} Yaw angle (degrees): ")
            )

            target_yaw_rad = np.radians(target_yaw_deg)
            target_points.append((target_x, target_y))
            target_angles.append(target_yaw_rad)

            print(
                f"Target {target_count} set: ({target_x:.2f}, {target_y:.2f}) m, {target_yaw_deg:.1f}°"
            )

        except ValueError:
            print("Invalid input. Skipping this target.")
            continue
        except KeyboardInterrupt:
            print("\nTest cancelled by user.")
            return

        add_another = input("\nAdd another target point? (y/n): ").strip().lower()
        if add_another != "y" and add_another != "yes":
            break

        target_count += 1
        if target_count > 10:  # Safety limit
            print("Maximum of 10 target points reached.")
            break

    if not target_points:
        print("No targets specified. Returning to main menu.")
        return

    # Display mission summary
    print(f"\n{'=' * 50}")
    print("  MULTI-POINT MISSION SUMMARY")
    print(f"{'=' * 50}")
    print(f"Number of targets: {len(target_points)}")
    print("Stabilization time at each target: 3 seconds")

    for i, (target_pos, target_angle) in enumerate(
        zip(target_points, target_angles), 1
    ):
        print(
            f"  Target {i}: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
        )

    print("Mission behavior: Navigate → Stabilize 3s → Next target")

    # Offer to add obstacles
    SatelliteConfig.clear_obstacles()
    add_obs = input("\nAdd obstacle? (y/n): ").strip().lower()
    while add_obs == "y":
        try:
            obs_x = float(input("  Obstacle X position (meters): "))
            obs_y = float(input("  Obstacle Y position (meters): "))
            obs_r = float(input("  Obstacle radius (meters): "))
            SatelliteConfig.add_obstacle(obs_x, obs_y, obs_r)
            print(f"  Obstacle added: ({obs_x:.2f}, {obs_y:.2f}), r={obs_r:.2f}")
        except Exception:
            print("  Invalid input, skipping.")
        add_obs = input("Add another obstacle? (y/n): ").strip().lower()
    if SatelliteConfig.get_obstacles():
        SatelliteConfig.OBSTACLES_ENABLED = True
        print(
            f"Obstacles enabled: {len(SatelliteConfig.get_obstacles())} obstacles configured."
        )
    else:
        SatelliteConfig.OBSTACLES_ENABLED = False

    # Confirm before starting
    confirm = input("\nProceed with MULTI-POINT test? (y/n): ").lower()
    if confirm != "y":
        print("Test cancelled.")
        return

    print("\nUsing Serial communication (COM20, 115200 baud)")

    # Configure waypoint mode
    SatelliteConfig.ENABLE_WAYPOINT_MODE = True
    SatelliteConfig.WAYPOINT_TARGETS = target_points.copy()
    SatelliteConfig.WAYPOINT_ANGLES = target_angles.copy()
    SatelliteConfig.CURRENT_TARGET_INDEX = 0
    SatelliteConfig.TARGET_STABILIZATION_START_TIME = None
    SatelliteConfig.TARGET_HOLD_TIME = 3.0

    real_test = RealSatelliteMPCLinearized(
        target_pos=target_points[0],
        target_angle=target_angles[0],
        hardware_type="serial",
        port="COM20",
        baudrate=115200,
    )

    real_test.run_real_test(show_visualization=True)


def main():
    """
    Enhanced menu system for real satellite hardware testing using unified Mission module.

    Modes:
    1. Waypoint Navigation (single or multiple waypoints)
    2. Shape Following (circles, rectangles, triangles, hexagons, custom DXF)
    """
    try:
        from mission import MissionManager

        manager = MissionManager("real")

        # Show menu and get user choice
        mode_choice = manager.show_mission_menu()

        # Run the selected mission
        config = manager.run_selected_mission(mode_choice)

        if config:
            print("\n Mission configured successfully!")
            print(f"Mission type: {config['mission_type']}")

            # Execute the real hardware test based on configuration
            execute_real_hardware_test(config)
        else:
            print("\n Mission configuration cancelled.")

    except KeyboardInterrupt:
        print("\n\nReal hardware test cancelled by user.")
        return
    except Exception as e:
        print(f"\n Error: {e}")
        print("Exiting real hardware test.")
        return


def execute_real_hardware_test(config):
    """Execute real hardware test based on mission configuration."""
    from real import RealSatelliteMPCLinearized

    try:
        mission_type = config["mission_type"]

        if mission_type == "waypoint":
            # Waypoint Navigation: configuration already set in Mission module
            real_test = RealSatelliteMPCLinearized(
                target_pos=config["target_points"][0] if "target_points" in config else config["target_pos"],
                target_angle=config["target_angles"][0] if "target_angles" in config else config["target_angle"],
                hardware_type="serial",
                port=config.get("port", "COM20"),
                baudrate=config.get("baudrate", 115200),
            )
            real_test.run_real_test(show_visualization=True)

        elif mission_type == "shape_following":
            # Shape Following: configuration already set in Mission module
            real_test = RealSatelliteMPCLinearized(
                target_pos=config.get("shape_center", (0.0, 0.0)),
                target_angle=0.0,
                hardware_type="serial",
                port=config.get("port", "COM20"),
                baudrate=config.get("baudrate", 115200),
            )
            real_test.run_real_test(show_visualization=True)

        else:
            print(f" Unknown mission type: {mission_type}")

    except KeyboardInterrupt:
        # Let KeyboardInterrupt propagate up to main()
        raise
    except Exception as e:
        print(f" Error executing real hardware test: {e}")
        import traceback

        traceback.print_exc()


def start_point_to_point_mode(
    target_x,
    target_y,
    target_yaw_deg,
    port="COM20",
    baudrate=115200,
    show_visualization=True,
):
    """Programmatic entry for Point-to-Point mode (for GUI integration)."""
    from real import RealSatelliteMPCLinearized

    target_yaw_rad = np.radians(target_yaw_deg)
    real_test = RealSatelliteMPCLinearized(
        target_pos=(target_x, target_y),
        target_angle=target_yaw_rad,
        hardware_type="serial",
        port=port,
        baudrate=baudrate,
    )
    real_test.run_real_test(show_visualization=show_visualization)


def start_multi_point_mode(
    target_points,
    target_angles_deg,
    port="COM20",
    baudrate=115200,
    show_visualization=True,
):
    """Programmatic entry for Multi-Point mode (for GUI integration)."""
    from real import RealSatelliteMPCLinearized

    target_angles = [np.radians(a) for a in target_angles_deg]
    SatelliteConfig.set_waypoint_mode(True)
    SatelliteConfig.set_waypoint_targets(target_points, target_angles)
    real_test = RealSatelliteMPCLinearized(
        target_pos=target_points[0],
        target_angle=target_angles[0],
        hardware_type="serial",
        port=port,
        baudrate=baudrate,
    )
    real_test.run_real_test(show_visualization=show_visualization)


if __name__ == "__main__":
    main()
