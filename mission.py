#!/usr/bin/env python3
"""
Unified Mission Module for MPC Satellite Control

Provides unified mission system for both simulation and real testing systems.
Defines common mission types and their configurations.

Mission Types:
1. Waypoint Navigation: Navigate to single or multiple waypoints in sequence
2. Shape Following: Follow moving target along custom shape paths
   - Demo shapes: Circle, Rectangle, Triangle, Hexagon
   - Custom shapes: Load from DXF CAD files

Features:
- Unified interface for both simulation and real hardware
- Automatic mode detection and configuration
- Interactive user input for mission parameters
- Obstacle configuration support
- Mission validation and confirmation
"""

import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from config import SatelliteConfig

# Add path for optional DXF_Viewer module
sys.path.insert(0, str(Path(__file__).parent / "DXF"))


class MissionManager:
    """Unified mission manager for both simulation and real test systems."""

    def __init__(self, mode: str = "simulation"):
        """Initialize mission manager.

        Args:
            mode: Either "simulation" or "real" to determine behavior
        """
        self.mode = mode.lower()
        if self.mode not in ["simulation", "real"]:
            raise ValueError("Mode must be either 'simulation' or 'real'")

        # Setup mode-specific labels and defaults
        self._setup_mode_labels()

    def _setup_mode_labels(self) -> None:
        """Setup mode-specific labels and defaults."""
        if self.mode == "simulation":
            self.system_name = "SIMULATION"
            self.system_title = "Satellite Control Simulation"
            self.hardware_config = None
        else:  # real
            self.system_name = "REAL HARDWARE TEST"
            self.system_title = "Real Satellite MPC Hardware Test"
            self.hardware_config = {"port": "COM20", "baudrate": 115200}

    def get_user_position(
        self, position_type: str, default_pos: Optional[Tuple[float, float]] = None
    ) -> Tuple[float, float]:
        """Get position input from user with validation.

        Args:
            position_type: Description of position (e.g., "starting", "target")
            default_pos: Default position to use if input fails

        Returns:
            Tuple of (x, y) coordinates
        """
        while True:
            try:
                x_input = input(
                    f"{position_type.title()} X position (meters): "
                ).strip()
                if x_input == "" and default_pos is not None:
                    return default_pos
                x = float(x_input)

                y_input = input(
                    f"{position_type.title()} Y position (meters): "
                ).strip()
                if y_input == "" and default_pos is not None:
                    return default_pos
                y = float(y_input)

                return (x, y)

            except ValueError:
                print("Invalid input. Please enter numeric values.")
                if default_pos is not None:
                    use_default = (
                        input(
                            f"Use default ({default_pos[0]:.2f}, {default_pos[1]:.2f})? (y/n): "
                        )
                        .strip()
                        .lower()
                    )
                    if use_default == "y":
                        return default_pos
            except KeyboardInterrupt:
                print(f"\n{self.system_name} cancelled by user.")
                raise

    def get_user_orientation(
        self, orientation_type: str, default_angle: Optional[float] = None
    ) -> float:
        """Get orientation input from user with validation.

        Args:
            orientation_type: Description of orientation (e.g., "starting", "target")
            default_angle: Default angle in radians

        Returns:
            Angle in radians
        """
        while True:
            try:
                angle_input = input(
                    f"{orientation_type.title()} orientation (degrees): "
                ).strip()
                if angle_input == "" and default_angle is not None:
                    return default_angle
                angle_deg = float(angle_input)
                return np.radians(angle_deg)

            except ValueError:
                print("Invalid input. Please enter a numeric value.")
                if default_angle is not None:
                    default_deg = np.degrees(default_angle)
                    use_default = (
                        input(f"Use default ({default_deg:.1f}°)? (y/n): ")
                        .strip()
                        .lower()
                    )
                    if use_default == "y":
                        return default_angle
            except KeyboardInterrupt:
                print(f"\n{self.system_name} cancelled by user.")
                raise

    def get_user_velocities(
        self,
        default_vx: float = 0.0,
        default_vy: float = 0.0,
        default_omega: float = 0.0,
    ) -> Tuple[float, float, float]:
        """Get initial velocity values from user input.

        Args:
            default_vx: Default X velocity in m/s
            default_vy: Default Y velocity in m/s
            default_omega: Default angular velocity in rad/s

        Returns:
            Tuple of (vx, vy, omega) velocities
        """
        while True:
            try:
                vx_input = input(
                    f"X velocity (m/s, default: {default_vx:.3f}): "
                ).strip()
                vy_input = input(
                    f"Y velocity (m/s, default: {default_vy:.3f}): "
                ).strip()
                omega_input = input(
                    f"Angular velocity (rad/s, default: {default_omega:.3f}): "
                ).strip()

                vx = float(vx_input) if vx_input else default_vx
                vy = float(vy_input) if vy_input else default_vy
                omega = float(omega_input) if omega_input else default_omega

                return (vx, vy, omega)

            except ValueError:
                print("Invalid velocity input. Please enter numeric values.")
            except KeyboardInterrupt:
                print(f"\n{self.system_name} cancelled by user.")
                raise

    def configure_obstacles(self) -> None:
        """Configure obstacles with user input."""
        SatelliteConfig.clear_obstacles()

        add_obs = input("\nAdd obstacle? (y/n): ").strip().lower()
        while add_obs == "y":
            try:
                obs_x = float(input("  Obstacle X position (meters): "))
                obs_y = float(input("  Obstacle Y position (meters): "))
                obs_r_input = input("  Obstacle radius (meters, default 0.5): ").strip()
                obs_r = float(obs_r_input) if obs_r_input else 0.5
                SatelliteConfig.add_obstacle(obs_x, obs_y, obs_r)
                print(f"  Obstacle added: ({obs_x:.2f}, {obs_y:.2f}), r={obs_r:.2f}")
            except ValueError:
                print("  Invalid input, skipping obstacle.")
            except KeyboardInterrupt:
                print(f"\n{self.system_name} cancelled by user.")
                raise

            add_obs = input("Add another obstacle? (y/n): ").strip().lower()

        if SatelliteConfig.get_obstacles():
            SatelliteConfig.OBSTACLES_ENABLED = True
            print(
                f"Obstacles enabled: {len(SatelliteConfig.get_obstacles())} obstacles configured."
            )
        else:
            SatelliteConfig.OBSTACLES_ENABLED = False
            print("No obstacles configured.")

    def confirm_mission(self, mission_type: str) -> bool:
        """Ask user to confirm mission start.

        Args:
            mission_type: Type of mission for confirmation message

        Returns:
            True if user confirms, False otherwise
        """
        confirm = (
            input(f"\nProceed with {mission_type} {self.mode}? (y/n): ").strip().lower()
        )
        if confirm != "y":
            print(f"{self.system_name} cancelled.")
            return False
        return True

    def run_multi_point_mode(self) -> Dict[str, Any]:
        """Mode 1: Waypoint Mission."""
        print("\n WAYPOINT MISSION")
        print(
            "The satellite will visit multiple waypoints in sequence or a single one based on selection."
        )
        print(
            f"After stabilizing at each waypoint for {SatelliteConfig.TARGET_HOLD_TIME:.1f} seconds, it moves to the next waypoint."
        )

        if self.mode == "simulation":
            # Simulation mode: get starting position and orientation
            print("\nSimulation starting configuration:")
            start_pos = self.get_user_position(
                "starting", SatelliteConfig.DEFAULT_START_POS
            )
            start_angle = self.get_user_orientation(
                "starting", SatelliteConfig.DEFAULT_START_ANGLE
            )
            start_vx, start_vy, start_omega = self.get_user_velocities()

            # Print summary
            # v_mag = np.sqrt(start_vx**2 + start_vy**2)  # Calculated but not displayed
            omega_deg_per_s = np.degrees(start_omega)
            print(
                f"Starting: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f} °, ({start_vx:.2f}, {start_vy:.2f}) m/s, {omega_deg_per_s:.1f} °/s"
            )
        else:
            # Real mode: current position is starting point (use defaults for type checking)
            print("\nReal hardware will use current position as starting point.")
            start_pos = (0.0, 0.0)  # Placeholder - will be overridden by real hardware
            start_angle = 0.0  # Placeholder - will be overridden by real hardware
            start_vx = start_vy = start_omega = 0.0

        # Collect target points and orientations
        target_points = []
        target_angles = []
        target_count = 1

        print("\nTarget configuration:")
        while True:
            print(f"\n--- Target Point {target_count} ---")

            if self.mode == "simulation":
                target_pos = self.get_user_position(f"target {target_count}")
                target_angle = self.get_user_orientation(f"target {target_count}")
            else:
                try:
                    target_x = float(
                        input(f"Target {target_count} X position (meters): ")
                    )
                    target_y = float(
                        input(f"Target {target_count} Y position (meters): ")
                    )
                    target_yaw_deg = float(
                        input(f"Target {target_count} Yaw angle (degrees): ")
                    )
                    target_pos = (target_x, target_y)
                    target_angle = np.radians(target_yaw_deg)
                except ValueError:
                    print("Invalid input. Skipping this target.")
                    continue

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

        if not target_points:
            print("No targets specified. Returning to main menu.")
            return {}  # Return empty dict instead of None

        # Display mission summary
        print(f"\n{'=' * 50}")
        print("  WAYPOINT MISSION SUMMARY")
        print(f"{'=' * 50}")
        if self.mode == "simulation":
            print(
                f"Starting: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f}°"
            )
        print(f"Number of targets: {len(target_points)}")
        print(f"Stabilization time at each target: {SatelliteConfig.TARGET_HOLD_TIME:.1f} seconds")

        if self.mode == "simulation":
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
        else:
            for i, (target_pos, target_angle) in enumerate(
                zip(target_points, target_angles), 1
            ):
                print(
                    f"  Target {i}: ({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
                )

        print("Mission behavior: Navigate → Stabilize 3s → Next target")

        # Configure obstacles
        self.configure_obstacles()

        config = {
            "mission_type": "waypoint",
            "target_points": target_points,
            "target_angles": target_angles,
        }

        if self.mode == "simulation":
            config.update(
                {
                    "start_pos": start_pos,
                    "start_angle": start_angle,
                    "start_vx": start_vx,
                    "start_vy": start_vy,
                    "start_omega": start_omega,
                }
            )
            # Update SatelliteConfig
            SatelliteConfig.DEFAULT_START_POS = start_pos
            SatelliteConfig.DEFAULT_START_ANGLE = start_angle
            SatelliteConfig.DEFAULT_TARGET_POS = target_points[0]
            SatelliteConfig.DEFAULT_TARGET_ANGLE = target_angles[0]

        else:
            if self.hardware_config:
                config.update(self.hardware_config)

        # Set waypoint mission configuration
        SatelliteConfig.ENABLE_WAYPOINT_MODE = True
        SatelliteConfig.WAYPOINT_TARGETS = target_points.copy()
        SatelliteConfig.WAYPOINT_ANGLES = target_angles.copy()
        SatelliteConfig.CURRENT_TARGET_INDEX = 0
        SatelliteConfig.TARGET_STABILIZATION_START_TIME = None
        if self.mode == "real":
            SatelliteConfig.TARGET_HOLD_TIME = 3.0

        return config

    def run_dxf_shape_mode(self) -> Dict[str, Any]:
        """Mission 2: Shape Following."""
        print("\n SHAPE FOLLOWING MISSION")
        print("Satellite will follow a moving target along a shape path.")
        print(
            "Available shapes: Circle, Rectangle, Triangle, Hexagon, or custom DXF files."
        )
        print("Mission has three phases:")
        print(f"  Phase 1: Move to closest point on shape and stabilize ({SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds)")
        print("  Phase 2: Track moving target along the shape path")
        print(f"  Phase 3: Stabilize at completion point ({SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:.1f} seconds)")

        if self.mode == "simulation":
            print("\nSimulation starting configuration:")
            start_pos = self.get_user_position(
                "starting", SatelliteConfig.DEFAULT_START_POS
            )
            start_angle = self.get_user_orientation(
                "starting", SatelliteConfig.DEFAULT_START_ANGLE
            )
            start_vx, start_vy, start_omega = self.get_user_velocities()

            # Print summary
            # v_mag = np.sqrt(start_vx**2 + start_vy**2)  # Calculated but not displayed
            omega_deg_per_s = np.degrees(start_omega)
            print(
                f"Starting: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f} °, ({start_vx:.2f}, {start_vy:.2f}) m/s, {omega_deg_per_s:.1f} °/s"
            )
        else:
            print("\nReal hardware will use current position as starting point.")
            start_pos = (0.0, 0.0)  # Placeholder - will be overridden by real hardware
            start_angle = 0.0  # Placeholder - will be overridden by real hardware
            start_vx = start_vy = start_omega = 0.0

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
            print("2. Circle")
            print("3. Rectangle")
            print("4. Triangle")
            print("5. Hexagon")
            choice = input("Select option (1-5): ").strip()
            if choice == "1":
                dxf_path = input("Enter DXF file path: ").strip()
                try:
                    shape_points = self.load_dxf_shape(dxf_path)
                    print(f" Loaded shape with {len(shape_points)} points from DXF")
                except Exception as e:
                    print(f" Failed to load DXF: {e}")
                    print("Falling back to demo circle")
                    shape_points = self.get_demo_shape("circle")
            elif choice == "2":
                shape_points = self.get_demo_shape("circle")
            elif choice == "3":
                shape_points = self.get_demo_shape("rectangle")
            elif choice == "4":
                shape_points = self.get_demo_shape("triangle")
            elif choice == "5":
                shape_points = self.get_demo_shape("hexagon")
            else:
                print("Invalid choice. Using demo circle.")
                shape_points = self.get_demo_shape("circle")
        else:
            print("\nDemo shapes available:")
            print("1. Circle")
            print("2. Rectangle")
            print("3. Triangle")
            print("4. Hexagon")
            choice = input("Select demo shape (1-4): ").strip()
            if choice == "1":
                shape_points = self.get_demo_shape("circle")
            elif choice == "2":
                shape_points = self.get_demo_shape("rectangle")
            elif choice == "3":
                shape_points = self.get_demo_shape("triangle")
            elif choice == "4":
                shape_points = self.get_demo_shape("hexagon")
            else:
                print("Invalid choice. Using circle.")
                shape_points = self.get_demo_shape("circle")

        try:
            if self.mode == "simulation":
                shape_center = self.get_user_position("shape center", (0.0, 0.0))
            else:
                center_x = float(input("Shape center X position (meters): "))
                center_y = float(input("Shape center Y position (meters): "))
                shape_center = (center_x, center_y)
            shape_rotation_input = input(
                "Shape rotation angle (degrees, default 0): "
            ).strip()
            shape_rotation_deg = (
                float(shape_rotation_input) if shape_rotation_input else 0.0
            )
            shape_rotation_rad = np.radians(shape_rotation_deg)
        except ValueError:
            print("Invalid input. Using default shape parameters.")
            shape_center = (0.0, 0.0)
            shape_rotation_deg = 0.0
            shape_rotation_rad = 0.0

        print(f"Shape center: ({shape_center[0]:.2f}, {shape_center[1]:.2f}) m")
        print(f"Shape rotation: {shape_rotation_deg:.1f}°")

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

        transformed_shape = self.transform_shape(
            shape_points, shape_center, shape_rotation_rad
        )
        upscaled_path = self.make_offset_path(transformed_shape, offset_distance)
        print(f" Created upscaled path with {len(upscaled_path)} points")

        print("\nMoving target configuration:")
        try:
            path_length = self.calculate_path_length(upscaled_path)
            print(f"Path length: {path_length:.2f} m")
            target_speed_input = input(
                "Target speed (meters/second, default 0.04): "
            ).strip()
            target_speed_mps = float(target_speed_input) if target_speed_input else 0.04
            if target_speed_mps <= 0:
                print("Speed must be positive. Using 0.04 m/s.")
                target_speed_mps = 0.04
            elif target_speed_mps > 0.5:
                print("Maximum speed 0.5 m/s. Using 0.5 m/s.")
                target_speed_mps = 0.5
        except ValueError:
            print("Invalid input. Using default speed.")
            target_speed_mps = 0.04

        print(f"Target speed: {target_speed_mps:.2f} m/s")

        # Get return position and orientation
        print("\nReturn position configuration:")
        use_return = (
            input("Return to specific position after profile? (y/n, default n): ")
            .strip()
            .lower()
        )
        if use_return == "y" or use_return == "yes":
            if self.mode == "simulation":
                return_pos = self.get_user_position("return", start_pos)
                return_angle = self.get_user_orientation("return", start_angle)
            else:
                try:
                    return_x = float(input("Return X position (meters): "))
                    return_y = float(input("Return Y position (meters): "))
                    return_yaw_deg = float(input("Return orientation (degrees): "))
                    return_pos = (return_x, return_y)
                    return_angle = np.radians(return_yaw_deg)
                except ValueError:
                    print("Invalid input. Using starting position as return position.")
                    return_pos = start_pos
                    return_angle = start_angle
            if return_pos is not None and return_angle is not None:
                print(
                    f"Return position: ({return_pos[0]:.2f}, {return_pos[1]:.2f}) m, {np.degrees(return_angle):.1f}°"
                )
            has_return = True
        else:
            return_pos = None
            return_angle = None
            has_return = False
            print("No return position specified. Mission will end after profile.")

        path_length = self.calculate_path_length(upscaled_path)
        # Estimate duration: 3s initial positioning + profile time + 10s final stabilization
        estimated_duration = 3.0 + (path_length / target_speed_mps) + 10.0
        print(f"Estimated mission duration: {estimated_duration:.1f} seconds")

        print(f"\n{'=' * 50}")
        print("  SHAPE FOLLOWING MISSION SUMMARY")
        print(f"{'=' * 50}")
        if self.mode == "simulation":
            print(
                f"Starting: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) m, {np.degrees(start_angle):.1f}°"
            )
        print(f"Shape center: ({shape_center[0]:.2f}, {shape_center[1]:.2f}) m")
        print(f"Shape rotation: {shape_rotation_deg:.1f}°")
        print(f"Offset distance: {offset_distance:.2f} m")
        print(f"Path points: {len(upscaled_path)}")
        print(f"Path length: {path_length:.2f} m")
        print(f"Target speed: {target_speed_mps:.2f} m/s")
        if has_return and return_pos is not None and return_angle is not None:
            print(
                f"Return position: ({return_pos[0]:.2f}, {return_pos[1]:.2f}) m, {np.degrees(return_angle):.1f}°"
            )
        print("Mission phases:")
        print("  Phase 1: Position at closest point and stabilize (3s)")
        print("  Phase 2: Track moving target along shape path")
        if has_return:
            print("  Phase 3: Return to specified position and stabilize (10s)")
        else:
            print("  Phase 3: Stabilize at final shape position (10s)")
        print(f"Mission duration: ~{estimated_duration:.1f}s estimated")

        self.configure_obstacles()

        config = {
            "mission_type": "shape_following",
            "shape_center": shape_center,
            "shape_rotation_rad": shape_rotation_rad,
            "shape_rotation_deg": shape_rotation_deg,
            "offset_distance": offset_distance,
            "target_speed_mps": target_speed_mps,
            "estimated_duration": estimated_duration,
            "shape_points": transformed_shape,
            "upscaled_path": upscaled_path,
            "path_length": path_length,
            "has_return": has_return,
            "return_pos": return_pos,
            "return_angle": return_angle,
        }

        if self.mode == "simulation":
            config.update(
                {
                    "start_pos": start_pos,
                    "start_angle": start_angle,
                    "start_vx": start_vx,
                    "start_vy": start_vy,
                    "start_omega": start_omega,
                }
            )
            SatelliteConfig.DEFAULT_START_POS = start_pos
            SatelliteConfig.DEFAULT_START_ANGLE = start_angle
        else:
            if self.hardware_config:
                config.update(self.hardware_config)

        SatelliteConfig.DXF_SHAPE_MODE_ACTIVE = True  # type: ignore[assignment]  # type: ignore[misc]
        SatelliteConfig.DXF_SHAPE_CENTER = shape_center  # type: ignore[assignment]
        SatelliteConfig.DXF_BASE_SHAPE = transformed_shape
        SatelliteConfig.DXF_SHAPE_PATH = upscaled_path
        SatelliteConfig.DXF_TARGET_SPEED = target_speed_mps
        SatelliteConfig.DXF_ESTIMATED_DURATION = estimated_duration  # type: ignore[assignment]
        SatelliteConfig.DXF_MISSION_START_TIME = None
        SatelliteConfig.DXF_SHAPE_PHASE = "POSITIONING"
        SatelliteConfig.DXF_PATH_LENGTH = path_length  # type: ignore[assignment]
        SatelliteConfig.DXF_HAS_RETURN = has_return  # type: ignore[misc]
        SatelliteConfig.DXF_RETURN_POSITION = return_pos  # type: ignore[misc]
        SatelliteConfig.DXF_RETURN_ANGLE = return_angle  # type: ignore[misc]

        for attr in [
            "DXF_TRACKING_START_TIME",
            "DXF_TARGET_START_DISTANCE",
            "DXF_STABILIZATION_START_TIME",
            "DXF_FINAL_POSITION",
            "DXF_RETURN_START_TIME",
        ]:
            if hasattr(SatelliteConfig, attr):
                delattr(SatelliteConfig, attr)

        return config

    def load_dxf_shape(self, file_path: str) -> List[Tuple[float, float]]:
        """
        Load shape points from DXF using the same pipeline as DXF_Viewer.

        Args:
            file_path: Path to DXF file

        Returns:
            List of (x, y) points in meters
        """
        import ezdxf  # type: ignore[import]

        try:
            from dxf_viewer import (  # type: ignore[import-not-found,import-untyped]
                extract_boundary_polygon,
                sanitize_boundary,
                units_code_to_name_and_scale,
            )
        except Exception as e:
            raise ImportError(f"dxf_viewer utilities unavailable: {e}")

        # Read DXF and determine units
        doc = ezdxf.readfile(file_path)  # type: ignore[attr-defined]
        msp = doc.modelspace()
        insunits = int(doc.header.get("$INSUNITS", 0))
        units_name, to_m = units_code_to_name_and_scale(insunits)

        # Extract and sanitize boundary in native units, then scale to meters
        boundary = extract_boundary_polygon(msp)
        boundary = sanitize_boundary(boundary, to_m)
        boundary_m = (
            [(float(x) * to_m, float(y) * to_m) for (x, y) in boundary]
            if boundary
            else []
        )

        if not boundary_m:
            raise ValueError("No usable DXF boundary could be constructed.")

        print(f" DXF Loaded (viewer parity): {len(boundary_m)} points")
        print(
            f"   Units: {units_name} (INSUNITS={insunits}), scaled → meters (x{to_m})"
        )
        return boundary_m

    def get_demo_shape(self, shape_type: str) -> List[Tuple[float, float]]:
        """
        Get predefined demo shape points.

        Args:
            shape_type: Type of shape ("circle", "rectangle", "triangle", or "hexagon")

        Returns:
            List of (x, y) points defining the shape
        """
        if shape_type == "circle":
            # Circle with 0.25m radius, 36 points for smooth path
            points = []
            num_points = 36
            for i in range(num_points + 1):  # +1 to close the shape
                angle = (i / num_points) * 2 * np.pi
                x = 0.25 * np.cos(angle)
                y = 0.25 * np.sin(angle)
                points.append((x, y))
            return points
        elif shape_type == "rectangle":
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
            # Default to circle
            return self.get_demo_shape("circle")

    def transform_shape(
        self,
        points: List[Tuple[float, float]],
        center: Tuple[float, float],
        rotation: float,
    ) -> List[Tuple[float, float]]:
        """
        Transform shape points to specified center and rotation.

        Args:
            points: List of (x, y) points
            center: Center point (x, y) for translation
            rotation: Rotation angle in radians

        Returns:
            List of transformed (x, y) points
        """
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

    def upscale_shape(
        self, points: List[Tuple[float, float]], offset_distance: float
    ) -> List[Tuple[float, float]]:
        """
        Create an upscaled path at fixed offset from shape boundary.

        Args:
            points: List of (x, y) points defining the shape
            offset_distance: Distance to offset from the boundary

        Returns:
            List of upscaled (x, y) points
        """
        if len(points) < 3:
            return points

        # Calculate shape centroid
        centroid_x = np.mean([p[0] for p in points])
        centroid_y = np.mean([p[1] for p in points])
        centroid = np.array([centroid_x, centroid_y])

        upscaled = []

        for i in range(len(points) - 1):  # Skip last point if it duplicates first
            current = np.array(points[i])
            next_point = np.array(points[i + 1])

            edge_vec = next_point - current
            edge_normal = np.array([-edge_vec[1], edge_vec[0]])
            if np.linalg.norm(edge_normal) > 0:
                edge_normal = edge_normal / np.linalg.norm(edge_normal)

            mid_point = (current + next_point) / 2
            to_mid = mid_point - centroid
            if np.dot(edge_normal, to_mid) < 0:
                edge_normal = -edge_normal

            # Offset the edge vertices
            offset_current = current + edge_normal * offset_distance
            upscaled.append(tuple(offset_current))

        if np.linalg.norm(np.array(points[0]) - np.array(points[-1])) < 1e-6:
            upscaled.append(upscaled[0])

        return upscaled

    def make_offset_path(
        self, points: List[Tuple[float, float]], offset_distance: float
    ) -> List[Tuple[float, float]]:
        """
        Create an outward offset path in meters, mirroring DXF_Viewer behavior.

        Delegates to DXF_Viewer.make_offset_path with units_to_m=1.0 for parity.
        Falls back to the previous implementation if the viewer utility isn't available.

        Args:
            points: List of (x, y) points defining the path
            offset_distance: Distance to offset from the path

        Returns:
            List of offset (x, y) points
        """
        if len(points) < 3:
            return points

        try:
            from DXF.dxf_viewer import (
                make_offset_path as viewer_make_offset_path,  # type: ignore[import-not-found,import-untyped]
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
            pts = points[:]
            if np.linalg.norm(np.array(pts[0]) - np.array(pts[-1])) > 1e-8:
                pts = pts + [pts[0]]
            try:
                import importlib.util  # type: ignore[import]

                geom_spec = importlib.util.find_spec("shapely.geometry")  # type: ignore[attr-defined]
                if geom_spec is None:
                    raise ImportError("shapely not installed")
                shapely_geometry = importlib.import_module("shapely.geometry")
                Polygon = shapely_geometry.Polygon
                poly = Polygon(pts)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                if poly.is_empty:
                    raise ValueError("Empty polygon after validity fix")
                buffered = poly.buffer(offset_distance, join_style=2, mitre_limit=5.0)
                exterior = list(buffered.exterior.coords)
                return [(float(x), float(y)) for x, y in exterior]
            except Exception:
                centroid = np.mean(np.array(pts[:-1]), axis=0)
                radii = [np.linalg.norm(np.array(p) - centroid) for p in pts[:-1]]
                avg_r = max(1e-6, float(np.mean(radii)))
                scale = 1.0 + (float(offset_distance) / avg_r)
                out = []
                for x, y in pts:
                    v = np.array([x, y]) - centroid
                    p = centroid + scale * v
                    out.append((float(p[0]), float(p[1])))
                if np.linalg.norm(np.array(out[0]) - np.array(out[-1])) > 1e-8:
                    out.append(out[0])
                return out

    def calculate_path_length(self, points: List[Tuple[float, float]]) -> float:
        """
        Calculate total length of path.

        Args:
            points: List of (x, y) points defining the path

        Returns:
            Total path length in meters
        """
        length = 0.0
        for i in range(len(points) - 1):
            p1 = np.array(points[i])
            p2 = np.array(points[i + 1])
            length += np.linalg.norm(p2 - p1)
        return float(length)

    def show_mission_menu(self) -> str:
        """
        Display mission selection menu and get user choice.

        Returns:
            User's mission mode choice as a string
        """
        print("=" * 60)
        print(f"    {self.system_title.upper()}")
        print("=" * 60)
        print("\nSelect mission mode:")
        print("1. Waypoint Navigation (single or multiple waypoints)")
        print(
            "2. Shape Following (circles, rectangles, triangles, hexagons, or custom DXF)"
        )

        return input("\nEnter mode (1-2): ").strip()

    def run_selected_mission(self, mode_choice: str) -> Optional[Dict[str, Any]]:
        """Run the mission based on user selection.

        Args:
            mode_choice: User's menu selection (1-2)

        Returns:
            Mission configuration dictionary or None if cancelled
        """
        if mode_choice == "1":
            print("\n MISSION 1: WAYPOINT NAVIGATION SELECTED")
            return self.run_multi_point_mode()
        elif mode_choice == "2":
            print("\n MISSION 2: SHAPE FOLLOWING SELECTED")
            return self.run_dxf_shape_mode()
        else:
            print("Invalid mode selected. Defaulting to Waypoint Navigation.")
            return self.run_multi_point_mode()


def handle_dxf_shape_update(simulation_obj):
    """
    Handle shape following mission updates for simulation.
    Returns True if mission is complete, False otherwise.
    """
    import numpy as np

    from config import SatelliteConfig

    if SatelliteConfig.DXF_MISSION_START_TIME is None:
        SatelliteConfig.DXF_MISSION_START_TIME = simulation_obj.simulation_time

        # Find closest point on path to starting position
        path = SatelliteConfig.DXF_SHAPE_PATH
        start_pos = np.array(
            [simulation_obj.satellite.position[0], simulation_obj.satellite.position[1]]
        )

        min_dist = float("inf")
        closest_idx = 0
        closest_point = path[0]

        for i, point in enumerate(path):
            dist = np.linalg.norm(start_pos - np.array(point))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                closest_point = point

        SatelliteConfig.DXF_CLOSEST_POINT_INDEX = closest_idx  # type: ignore[misc]
        SatelliteConfig.DXF_CURRENT_TARGET_POSITION = closest_point  # type: ignore[misc]
        total_length = 0.0
        for i in range(len(path)):
            idx = (SatelliteConfig.DXF_CLOSEST_POINT_INDEX + i) % len(path)
            next_idx = (SatelliteConfig.DXF_CLOSEST_POINT_INDEX + i + 1) % len(path)
            total_length += np.linalg.norm(
                np.array(path[next_idx]) - np.array(path[idx])
            )
        SatelliteConfig.DXF_PATH_LENGTH = total_length  # type: ignore[assignment]

        print(
            f" SHAPE FOLLOWING MISSION STARTED at t={simulation_obj.simulation_time:.2f}s"
        )
        print(
            f"   Phase 1: Moving to closest point on path ({closest_point[0]:.3f}, {closest_point[1]:.3f})"
        )
        print("   Target will start moving once satellite is stable")
        print(
            f" Shape path length: {SatelliteConfig.DXF_PATH_LENGTH:.3f} m (from index {SatelliteConfig.DXF_CLOSEST_POINT_INDEX})"
        )

    path = SatelliteConfig.DXF_SHAPE_PATH

    # Check current phase
    if getattr(SatelliteConfig, "DXF_SHAPE_PHASE", "POSITIONING") == "POSITIONING":
        # Phase 1: Move to closest point and stabilize
        target_pos = SatelliteConfig.DXF_CURRENT_TARGET_POSITION

        if target_pos is None:
            # Shouldn't happen, but handle it gracefully
            return False

        target_orientation = get_path_tangent_orientation(
            path, 0.0, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
        )

        simulation_obj.target_state = np.array(
            [target_pos[0], target_pos[1], 0.0, 0.0, target_orientation, 0.0]
        )

        current_pos = np.array(
            [simulation_obj.satellite.position[0], simulation_obj.satellite.position[1]]
        )
        pos_error = np.linalg.norm(current_pos - np.array(target_pos))
        ang_error = abs(
            simulation_obj.normalize_angle(
                simulation_obj.satellite.angle - target_orientation
            )
        )

        if (
            pos_error < simulation_obj.position_tolerance
            and ang_error < simulation_obj.angle_tolerance
        ):
            if not hasattr(simulation_obj, "shape_stabilization_start_time"):
                simulation_obj.shape_stabilization_start_time = (
                    simulation_obj.simulation_time
                )
                print(f" Reached starting position, stabilizing for {SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds...")
            else:
                stabilization_time = (
                    simulation_obj.simulation_time
                    - simulation_obj.shape_stabilization_start_time
                )
                if stabilization_time >= SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:
                    SatelliteConfig.DXF_SHAPE_PHASE = "TRACKING"  # type: ignore[misc]
                    SatelliteConfig.DXF_TRACKING_START_TIME = (
                        simulation_obj.simulation_time
                    )
                    SatelliteConfig.DXF_TARGET_START_DISTANCE = 0.0
                    print(" Satellite stable! Starting profile tracking...")
                    print(
                        f"   Target speed: {SatelliteConfig.DXF_TARGET_SPEED:.2f} m/s"
                    )
        else:
            if hasattr(simulation_obj, "shape_stabilization_start_time"):
                delattr(simulation_obj, "shape_stabilization_start_time")

    elif SatelliteConfig.DXF_SHAPE_PHASE == "TRACKING":
        # Phase 2: Track moving target along shape
        tracking_time = (
            simulation_obj.simulation_time - SatelliteConfig.DXF_TRACKING_START_TIME
        )
        distance_traveled = SatelliteConfig.DXF_TARGET_SPEED * tracking_time
        path_len = max(getattr(SatelliteConfig, "DXF_PATH_LENGTH", 0.0), 1e-9)

        # Distance-based completion
        if distance_traveled >= path_len:
            current_path_position, _ = get_position_on_path(
                path, path_len, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )

            # Print completion message only once
            if not getattr(
                SatelliteConfig, "DXF_PATH_COMPLETED_MESSAGE_PRINTED", False
            ):
                print(
                    f" Shape Following: Full path completed! Distance: {distance_traveled:.2f}m, Path length: {path_len:.2f}m"
                )
                SatelliteConfig.DXF_PATH_COMPLETED_MESSAGE_PRINTED = True  # type: ignore[misc]

            # Check if we have a return position
            has_return = getattr(SatelliteConfig, "DXF_HAS_RETURN", False)

            if has_return:
                # If return position specified, go directly to RETURNING phase
                SatelliteConfig.DXF_SHAPE_PHASE = "RETURNING"  # type: ignore[misc]
                SatelliteConfig.DXF_RETURN_START_TIME = simulation_obj.simulation_time  # type: ignore[misc]
                return_pos = SatelliteConfig.DXF_RETURN_POSITION  # type: ignore[misc]
                return_angle = SatelliteConfig.DXF_RETURN_ANGLE  # type: ignore[misc]
                print(
                    f" Profile traversal completed! Total tracking time: {tracking_time:.1f}s"
                )
                if return_pos and return_angle is not None:
                    print(
                        f" Starting return to position ({return_pos[0]:.2f}, {return_pos[1]:.2f}) m, {np.degrees(return_angle):.1f}°"
                    )
            else:
                # No return position, stabilize at the last point on the path
                if SatelliteConfig.DXF_SHAPE_PHASE != "STABILIZING":
                    # First time entering STABILIZING phase
                    SatelliteConfig.DXF_SHAPE_PHASE = "STABILIZING"  # type: ignore[misc]
                    SatelliteConfig.DXF_STABILIZATION_START_TIME = simulation_obj.simulation_time  # type: ignore[misc]
                    SatelliteConfig.DXF_FINAL_POSITION = current_path_position  # type: ignore[misc]
                    print(
                        f" Profile traversal completed! Stabilizing at final path position for {SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:.1f} seconds..."
                    )
                    print(f"   Total tracking time: {tracking_time:.1f}s")
                    print(
                        f"⏱  Starting final stabilization timer at t={simulation_obj.simulation_time:.1f}s"
                    )
        else:
            # Update target position
            wrapped_s = distance_traveled % path_len
            current_path_position, _ = get_position_on_path(
                path, wrapped_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )
            SatelliteConfig.DXF_CURRENT_TARGET_POSITION = current_path_position  # type: ignore[misc]

            target_orientation = get_path_tangent_orientation(
                path, wrapped_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )

            # Set the target state
            simulation_obj.target_state = np.array(
                [
                    current_path_position[0],
                    current_path_position[1],
                    0.0,
                    0.0,
                    target_orientation,
                    0.0,
                ]
            )

    elif SatelliteConfig.DXF_SHAPE_PHASE == "STABILIZING":
        # Phase 3: Stabilize at final position (only used when NO return position)
        final_pos = SatelliteConfig.DXF_FINAL_POSITION
        if final_pos is None:
            return False

        end_s = getattr(SatelliteConfig, "DXF_PATH_LENGTH", 0.0)
        target_orientation = get_path_tangent_orientation(
            path, end_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
        )

        simulation_obj.target_state = np.array(
            [final_pos[0], final_pos[1], 0.0, 0.0, target_orientation, 0.0]
        )

        # Timer should already be set when we entered this phase
        stabilization_start = getattr(
            SatelliteConfig,
            "DXF_STABILIZATION_START_TIME",
            simulation_obj.simulation_time,
        )
        stabilization_time = simulation_obj.simulation_time - stabilization_start

        # Print progress every 2 seconds
        last_print = getattr(SatelliteConfig, "DXF_LAST_PROGRESS_PRINT", 0.0)
        if simulation_obj.simulation_time - last_print >= 2.0:
            print(f"⏱  Stabilizing... {stabilization_time:.1f}s / {SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:.1f}s")
            SatelliteConfig.DXF_LAST_PROGRESS_PRINT = simulation_obj.simulation_time  # type: ignore[misc]

        if stabilization_time >= SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:
            # Print completion message only once using a flag
            if not getattr(SatelliteConfig, "DXF_COMPLETION_MESSAGE_PRINTED", False):
                print(" SHAPE FOLLOWING MISSION COMPLETED!")
                print("   Shape successfully traversed and stabilized")
                SatelliteConfig.DXF_COMPLETION_MESSAGE_PRINTED = True  # type: ignore[misc]
            return True

    elif SatelliteConfig.DXF_SHAPE_PHASE == "RETURNING":
        # Phase 4: Return to specified position and stabilize
        return_pos = SatelliteConfig.DXF_RETURN_POSITION  # type: ignore[misc]
        return_angle = SatelliteConfig.DXF_RETURN_ANGLE  # type: ignore[misc]

        if return_pos is None or return_angle is None:
            return False

        # Type narrowing: we know return_pos and return_angle are not None here
        return_x, return_y = return_pos[0], return_pos[1]
        simulation_obj.target_state = np.array(
            [return_x, return_y, 0.0, 0.0, return_angle, 0.0]
        )

        # Check if we've reached the return position and stayed for 10 seconds
        current_pos = np.array(
            [simulation_obj.satellite.position[0], simulation_obj.satellite.position[1]]
        )
        pos_error = np.linalg.norm(current_pos - np.array(return_pos))
        ang_error = abs(
            simulation_obj.normalize_angle(
                simulation_obj.satellite.angle - return_angle
            )
        )

        if (
            pos_error < simulation_obj.position_tolerance
            and ang_error < simulation_obj.angle_tolerance
        ):
            if not hasattr(simulation_obj, "return_stabilization_start_time"):
                simulation_obj.return_stabilization_start_time = (
                    simulation_obj.simulation_time
                )
                print(f" Reached return position, stabilizing for {SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:.1f} seconds...")
            else:
                stabilization_time = (
                    simulation_obj.simulation_time
                    - simulation_obj.return_stabilization_start_time
                )
                if stabilization_time >= SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:
                    # Print completion message only once using a flag
                    if not getattr(
                        SatelliteConfig, "DXF_RETURN_COMPLETION_MESSAGE_PRINTED", False
                    ):
                        print(" MISSION COMPLETED!")
                        print("   Profile traversed and returned to position")
                        SatelliteConfig.DXF_RETURN_COMPLETION_MESSAGE_PRINTED = True  # type: ignore[misc]
                    return True
        else:
            if hasattr(simulation_obj, "return_stabilization_start_time"):
                delattr(simulation_obj, "return_stabilization_start_time")

    return False


def get_position_on_path(path, distance, start_idx):
    """
    Get position on path given distance traveled from start point.
    Returns (position, completed) where completed is True if full path traversed.
    """
    if not path or len(path) < 2:
        return path[0] if path else (0, 0), True

    remaining_distance = distance

    total_length = 0.0
    for i in range(len(path)):
        idx = (start_idx + i) % len(path)
        next_idx = (start_idx + i + 1) % len(path)
        segment_length = np.linalg.norm(np.array(path[next_idx]) - np.array(path[idx]))
        total_length += segment_length

    if distance >= total_length:
        # Path completed - but don't print here, let the caller handle it once
        return path[start_idx], True

    for i in range(len(path)):
        idx = (start_idx + i) % len(path)
        next_idx = (start_idx + i + 1) % len(path)

        p1 = np.array(path[idx])
        p2 = np.array(path[next_idx])
        segment_length = np.linalg.norm(p2 - p1)

        if remaining_distance <= segment_length:
            # Position is on this segment
            if segment_length > 0:
                t = remaining_distance / segment_length
                position = p1 + t * (p2 - p1)
                return tuple(position), False
            else:
                return tuple(p1), False

        remaining_distance -= segment_length

    return path[start_idx], True


def get_path_tangent_orientation(path, distance, start_idx):
    """
    Get the tangent orientation (direction of travel) along the path at a given distance.
    Returns the angle in radians that the satellite should face while following the path.
    """
    import numpy as np

    if not path or len(path) < 2:
        return 0.0

    remaining_distance = distance

    # Find the current segment and position along path
    for i in range(len(path)):
        idx = (start_idx + i) % len(path)
        next_idx = (start_idx + i + 1) % len(path)

        p1 = np.array(path[idx])
        p2 = np.array(path[next_idx])
        segment_length = np.linalg.norm(p2 - p1)

        if remaining_distance <= segment_length:
            # We're on this segment, calculate the tangent direction
            if segment_length > 0:
                direction_vector = (p2 - p1) / segment_length  # Normalize
                tangent_angle = np.arctan2(direction_vector[1], direction_vector[0])
                return tangent_angle
            else:
                if i + 1 < len(path):
                    next_next_idx = (start_idx + i + 2) % len(path)
                    p3 = np.array(path[next_next_idx])
                    next_direction = p3 - p2
                    if np.linalg.norm(next_direction) > 0:
                        next_direction = next_direction / np.linalg.norm(next_direction)
                        return np.arctan2(next_direction[1], next_direction[0])
                return 0.0

        remaining_distance -= segment_length

    # If we've gone past the end, use the direction of the last segment
    last_idx = (start_idx + len(path) - 1) % len(path)
    first_idx = start_idx
    p_last = np.array(path[last_idx])
    p_first = np.array(path[first_idx])
    final_direction = p_first - p_last  # Direction from last point back to start
    if np.linalg.norm(final_direction) > 0:
        final_direction = final_direction / np.linalg.norm(final_direction)
        return np.arctan2(final_direction[1], final_direction[0])

    return 0.0


def get_user_position(position_type: str, default_pos: tuple = None) -> tuple:  # type: ignore[assignment]
    """Backward compatibility function for position input."""
    manager = MissionManager("simulation")  # Default to simulation
    return manager.get_user_position(position_type, default_pos)


def get_user_orientation(orientation_type: str, default_angle: float = None) -> float:  # type: ignore[assignment]
    """Backward compatibility function for orientation input."""
    manager = MissionManager("simulation")  # Default to simulation
    return manager.get_user_orientation(orientation_type, default_angle)


def get_user_obstacles() -> list:
    """Backward compatibility function for obstacle configuration."""
    manager = MissionManager("simulation")  # Default to simulation
    manager.configure_obstacles()
    return SatelliteConfig.get_obstacles()


def confirm_and_run_simulation(
    sim_type: str = "Point-to-Point", config: Optional[dict] = None
) -> None:
    """
    Run simulation with mission configuration.

    Args:
        sim_type: Type of simulation for display purposes
        config: Mission configuration dictionary containing start positions and velocities
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

    # Import here to avoid circular imports
    try:
        from simulation import SatelliteMPCLinearizedSimulation

        # Extract velocities from config if available
        start_vx = 0.0
        start_vy = 0.0
        start_omega = 0.0

        if config:
            start_vx = config.get("start_vx", 0.0)
            start_vy = config.get("start_vy", 0.0)
            start_omega = config.get("start_omega", 0.0)

        sim = SatelliteMPCLinearizedSimulation(
            start_vx=start_vx, start_vy=start_vy, start_omega=start_omega
        )
        sim.run_simulation(show_animation=False)
    except ImportError as e:
        print(f" Could not import simulation module: {e}")
        print("Please run simulation manually from Simulation.py")


def run_multi_point_mode(mode: str = "simulation"):
    """Run waypoint mission mode."""
    manager = MissionManager(mode)
    config = manager.run_multi_point_mode()
    return config


def run_dxf_shape_mode(mode: str = "simulation"):
    """Run shape following mission mode."""
    manager = MissionManager(mode)
    config = manager.run_dxf_shape_mode()
    return config


def main():
    """Main function for standalone mission testing."""
    import argparse

    parser = argparse.ArgumentParser(description="Mission Configuration System")
    parser.add_argument(
        "--mode",
        choices=["simulation", "real"],
        default="simulation",
        help="Mission mode (simulation or real)",
    )

    args = parser.parse_args()

    try:
        manager = MissionManager(args.mode)
        mode_choice = manager.show_mission_menu()
        config = manager.run_selected_mission(mode_choice)

        if config:
            print("\n Mission configured successfully!")
            print(f"Mission type: {config['mission_type']}")
            if args.mode == "simulation":
                confirm_and_run_simulation(
                    config["mission_type"].replace("_", "-").title(), config
                )
        else:
            print("\n Mission configuration cancelled.")

    except KeyboardInterrupt:
        print("\n\nMission configuration cancelled by user.")
        return 1
    except Exception as e:
        print(f"\n Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
