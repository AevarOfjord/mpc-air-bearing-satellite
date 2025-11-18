"""
Mission State Manager for Satellite Control System

Centralized mission logic shared between simulation and real hardware testing.
Provides unified mission state transitions and target calculations for all mission types.

Mission types supported:
1. Waypoint Navigation: Navigate to single or multiple waypoints with rotation control
2. Shape Following: Follow geometric paths (circles, rectangles, triangles, hexagons, DXF imports)

Key features:
- Unified state machine for mission progression
- Position and orientation tolerance checking
- Target calculation and waypoint management
- DXF shape import and path generation
- Mission completion detection
- Eliminates ~800 lines of duplicate code between simulation and real modes
"""

from typing import Callable, List, Optional, Tuple

import numpy as np

from config import SatelliteConfig


class MissionStateManager:
    """
    Manages mission state transitions and target calculations for all mission types.

    This class provides a unified implementation of mission logic for both
    simulation and real hardware control systems.
    """

    def __init__(
        self,
        position_tolerance: float = 0.05,
        angle_tolerance: float = 0.05,
        normalize_angle_func: Optional[Callable[[float], float]] = None,
        angle_difference_func: Optional[Callable[[float, float], float]] = None,
        point_to_line_distance_func: Optional[
            Callable[[np.ndarray, np.ndarray, np.ndarray], float]
        ] = None,
        calculate_safe_path_func: Optional[
            Callable[[np.ndarray, np.ndarray, List, float], List]
        ] = None,
    ):
        """
        Initialize mission state manager.

        Args:
            position_tolerance: Position error tolerance in meters
            angle_tolerance: Angle error tolerance in radians
            normalize_angle_func: Function to normalize angles to [-pi, pi]
            angle_difference_func: Function to calculate angle difference
            point_to_line_distance_func: Function to calculate point-to-line distance
            calculate_safe_path_func: Function to calculate safe path avoiding obstacles
        """
        self.position_tolerance = position_tolerance
        self.angle_tolerance = angle_tolerance

        # Store helper functions
        self.normalize_angle = normalize_angle_func or self._default_normalize_angle
        self.angle_difference = angle_difference_func or self._default_angle_difference
        self.point_to_line_distance = (
            point_to_line_distance_func or self._default_point_to_line_distance
        )
        self.calculate_safe_path = calculate_safe_path_func

        # Mission state tracking
        self.current_nav_waypoint_idx: int = 0
        self.nav_target_reached_time: Optional[float] = None

        self.dxf_completed: bool = False
        self.multi_point_target_reached_time: Optional[float] = None

        self.shape_stabilization_start_time: Optional[float] = None
        self.return_stabilization_start_time: Optional[float] = None
        self.final_waypoint_stabilization_start_time: Optional[float] = None

    @staticmethod
    def _default_normalize_angle(angle: float) -> float:
        """Default angle normalization to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    @staticmethod
    def _default_angle_difference(angle1: float, angle2: float) -> float:
        """Default angle difference calculation."""
        diff = angle1 - angle2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    @staticmethod
    def _default_point_to_line_distance(
        point: np.ndarray, line_start: np.ndarray, line_end: np.ndarray
    ) -> float:
        """Default point-to-line distance calculation."""
        line_vec = line_end - line_start
        line_length_sq = np.dot(line_vec, line_vec)

        if line_length_sq == 0:
            return float(np.linalg.norm(point - line_start))

        point_vec = point - line_start
        t = np.dot(point_vec, line_vec) / line_length_sq
        t = max(0, min(1, t))

        closest_point = line_start + t * line_vec
        return float(np.linalg.norm(point - closest_point))

    def update_target_state(
        self,
        current_position: np.ndarray,
        current_angle: float,
        current_time: float,
        current_state: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """
        Update target state based on active mission mode.

        Args:
            current_position: Current satellite position [x, y]
            current_angle: Current satellite orientation in radians
            current_time: Current simulation/control time in seconds
            current_state: Full state vector [x, y, vx, vy, theta, omega]

        Returns:
            Target state vector [x, y, vx, vy, theta, omega] or None
        """
        # Waypoint mode
        if (
            hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
            and SatelliteConfig.ENABLE_WAYPOINT_MODE
        ):
            return self._handle_multi_point_mode(
                current_position, current_angle, current_time
            )

        # DXF shape mode
        elif (
            hasattr(SatelliteConfig, "DXF_SHAPE_MODE_ACTIVE")
            and SatelliteConfig.DXF_SHAPE_MODE_ACTIVE
        ):
            return self._handle_dxf_shape_mode(
                current_position, current_angle, current_time
            )

        # Point-to-point mode (no-op, handled by caller)
        return None

    def _handle_multi_point_mode(
        self, current_position: np.ndarray, current_angle: float, current_time: float
    ) -> Optional[np.ndarray]:
        """Handle waypoint sequential navigation mode."""
        target_pos, target_angle = SatelliteConfig.get_current_waypoint_target()
        if target_pos is None:
            return None

        target_state = np.array(
            [target_pos[0], target_pos[1], 0.0, 0.0, target_angle, 0.0]
        )

        pos_error = np.linalg.norm(current_position - np.array(target_pos))
        ang_error = abs(self.angle_difference(target_angle, current_angle))

        if pos_error < self.position_tolerance and ang_error < self.angle_tolerance:
            if self.multi_point_target_reached_time is None:
                self.multi_point_target_reached_time = current_time
                print(
                    f" TARGET {SatelliteConfig.CURRENT_TARGET_INDEX + 1} REACHED! Stabilizing..."
                )
            else:
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

                maintenance_time = current_time - self.multi_point_target_reached_time
                if maintenance_time >= required_hold_time:
                    # Advance to next target
                    next_available = SatelliteConfig.advance_to_next_target()
                    if next_available:
                        (
                            target_pos,
                            target_angle,
                        ) = SatelliteConfig.get_current_waypoint_target()
                        print(
                            f" MOVING TO NEXT TARGET {SatelliteConfig.CURRENT_TARGET_INDEX + 1}: "
                            f"({target_pos[0]:.2f}, {target_pos[1]:.2f}) m, {np.degrees(target_angle):.1f}°"
                        )
                        self.multi_point_target_reached_time = None
                    else:
                        # All targets completed
                        SatelliteConfig.MULTI_POINT_PHASE = "COMPLETE"  # type: ignore[assignment]
                        print(
                            " ALL WAYPOINTS REACHED! Final stabilization phase."
                        )
                        return None  # Signal mission complete
        else:
            self.multi_point_target_reached_time = None

        return target_state

    def _handle_dxf_shape_mode(
        self, current_position: np.ndarray, current_angle: float, current_time: float
    ) -> Optional[np.ndarray]:
        """Handle DXF shape following mode."""
        # Import Mission functions if available
        try:
            from mission import (  # noqa: F401
                get_path_tangent_orientation,
                get_position_on_path,
            )
        except ImportError:
            print(" Mission module not available for DXF shape mode")
            return None

        # Initialize mission start time
        if SatelliteConfig.DXF_MISSION_START_TIME is None:
            SatelliteConfig.DXF_MISSION_START_TIME = current_time  # type: ignore

            # Find closest point on path
            path = SatelliteConfig.DXF_SHAPE_PATH
            min_dist = float("inf")
            closest_idx = 0
            closest_point = path[0]

            for i, point in enumerate(path):
                dist = np.linalg.norm(current_position - np.array(point))
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
                    closest_point = point

            SatelliteConfig.DXF_CLOSEST_POINT_INDEX = closest_idx  # type: ignore
            SatelliteConfig.DXF_CURRENT_TARGET_POSITION = closest_point  # type: ignore

            # Calculate total path length
            total_length = 0.0
            for i in range(len(path)):
                idx = (closest_idx + i) % len(path)
                next_idx = (closest_idx + i + 1) % len(path)
                total_length += np.linalg.norm(
                    np.array(path[next_idx]) - np.array(path[idx])
                )

            SatelliteConfig.DXF_PATH_LENGTH = total_length  # type: ignore

            print(f" PROFILE FOLLOWING MISSION STARTED at t={current_time:.2f}s")
            print(
                f"   Phase 1: Moving to closest point on path ({closest_point[0]:.3f}, {closest_point[1]:.3f})"
            )
            print(f" Profile path length: {total_length:.3f} m")

        path = SatelliteConfig.DXF_SHAPE_PATH
        phase = getattr(SatelliteConfig, "DXF_SHAPE_PHASE", "POSITIONING")

        # Phase 1: POSITIONING
        if phase == "POSITIONING":
            return self._dxf_positioning_phase(
                current_position, current_angle, current_time, path
            )

        # Phase 2: TRACKING
        elif phase == "TRACKING":
            return self._dxf_tracking_phase(current_position, current_time, path)

        # Phase 3: PATH_STABILIZATION (at path waypoints)
        elif phase == "PATH_STABILIZATION":
            return self._dxf_path_stabilization_phase(
                current_position, current_angle, current_time, path
            )

        # Phase 4: STABILIZING
        elif phase == "STABILIZING":
            return self._dxf_stabilizing_phase(current_time, path)

        # Phase 5: RETURNING
        elif phase == "RETURNING":
            return self._dxf_returning_phase(
                current_position, current_angle, current_time
            )

        return None

    def _dxf_positioning_phase(
        self,
        current_position: np.ndarray,
        current_angle: float,
        current_time: float,
        path: List[Tuple[float, float]],
    ) -> Optional[np.ndarray]:
        """Handle DXF positioning phase."""
        from mission import get_path_tangent_orientation

        # Set phase start time on first entry
        if SatelliteConfig.DXF_POSITIONING_START_TIME is None:
            SatelliteConfig.DXF_POSITIONING_START_TIME = current_time  # type: ignore

        target_pos = SatelliteConfig.DXF_CURRENT_TARGET_POSITION
        target_orientation = get_path_tangent_orientation(
            path, 0.0, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
        )

        target_state = np.array([target_pos[0], target_pos[1], 0.0, 0.0, target_orientation, 0.0])  # type: ignore

        pos_error = np.linalg.norm(current_position - np.array(target_pos))
        ang_error = abs(self.angle_difference(target_orientation, current_angle))

        if pos_error < self.position_tolerance and ang_error < self.angle_tolerance:
            if self.shape_stabilization_start_time is None:
                self.shape_stabilization_start_time = current_time
                SatelliteConfig.DXF_SHAPE_PHASE = "PATH_STABILIZATION"  # type: ignore
                SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME = current_time  # type: ignore
                print(f" Reached starting position, stabilizing for {SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds...")
            else:
                stabilization_time = current_time - self.shape_stabilization_start_time
                if stabilization_time >= SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:
                    SatelliteConfig.DXF_SHAPE_PHASE = "TRACKING"  # type: ignore
                    SatelliteConfig.DXF_TRACKING_START_TIME = current_time  # type: ignore
                    SatelliteConfig.DXF_TARGET_START_DISTANCE = 0.0
                    print(" Satellite stable! Starting profile tracking...")
                    print(
                        f"   Target speed: {SatelliteConfig.DXF_TARGET_SPEED:.2f} m/s"
                    )
        else:
            self.shape_stabilization_start_time = None

        return target_state

    def _dxf_tracking_phase(
        self,
        current_position: np.ndarray,
        current_time: float,
        path: List[Tuple[float, float]],
    ) -> Optional[np.ndarray]:
        """Handle DXF tracking phase."""
        from mission import get_path_tangent_orientation, get_position_on_path

        tracking_time = current_time - SatelliteConfig.DXF_TRACKING_START_TIME  # type: ignore
        distance_traveled = SatelliteConfig.DXF_TARGET_SPEED * tracking_time
        path_len = max(getattr(SatelliteConfig, "DXF_PATH_LENGTH", 0.0), 1e-9)

        if distance_traveled >= path_len:
            # Path complete
            current_path_position, _ = get_position_on_path(
                path, path_len, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )
            has_return = getattr(SatelliteConfig, "DXF_HAS_RETURN", False)
            if has_return:
                # Start path stabilization phase at final waypoint before returning
                if self.final_waypoint_stabilization_start_time is None:
                    self.final_waypoint_stabilization_start_time = current_time
                    SatelliteConfig.DXF_SHAPE_PHASE = "PATH_STABILIZATION"  # type: ignore
                    SatelliteConfig.DXF_PATH_STABILIZATION_START_TIME = current_time  # type: ignore  # Reset for second PATH_STABILIZATION
                    SatelliteConfig.DXF_FINAL_POSITION = current_path_position  # type: ignore
                    print(" Profile traversal completed!")
                    print(
                        f" Stabilizing at final waypoint for {SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds before return..."
                    )
                # Return None to let next update handle PATH_STABILIZATION phase
                return None
            else:
                SatelliteConfig.DXF_STABILIZATION_START_TIME = current_time  # type: ignore
                SatelliteConfig.DXF_SHAPE_PHASE = "STABILIZING"  # type: ignore
                SatelliteConfig.DXF_FINAL_POSITION = current_path_position  # type: ignore
                print(" Profile traversal completed! Stabilizing at final position...")
                return None
        else:
            # Continue tracking
            wrapped_s = distance_traveled % path_len
            current_path_position, _ = get_position_on_path(
                path, wrapped_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )
            SatelliteConfig.DXF_CURRENT_TARGET_POSITION = current_path_position  # type: ignore

            target_orientation = get_path_tangent_orientation(
                path, wrapped_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )

            return np.array(
                [
                    current_path_position[0],
                    current_path_position[1],  # type: ignore
                    0.0,
                    0.0,
                    target_orientation,
                    0.0,
                ]
            )

        return None

    def _dxf_path_stabilization_phase(
        self,
        current_position: np.ndarray,
        current_angle: float,
        current_time: float,
        path: List[Tuple[float, float]],
    ) -> Optional[np.ndarray]:
        """Handle DXF path stabilization phase - stabilizing at path waypoints (start or end)."""
        from mission import get_path_tangent_orientation

        # Determine if we're stabilizing at start or end based on which timer is active
        # and whether DXF_FINAL_POSITION has been set
        is_end_stabilization = (
            hasattr(SatelliteConfig, 'DXF_FINAL_POSITION')
            and SatelliteConfig.DXF_FINAL_POSITION is not None
        )

        if is_end_stabilization:
            # Stabilizing at END of path (before returning)
            target_pos = SatelliteConfig.DXF_FINAL_POSITION
            path_s = getattr(SatelliteConfig, "DXF_PATH_LENGTH", 0.0)
        else:
            # Stabilizing at START of path (before tracking begins)
            target_pos = SatelliteConfig.DXF_CURRENT_TARGET_POSITION
            path_s = 0.0

        target_orientation = get_path_tangent_orientation(
            path, path_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
        )

        target_state = np.array([target_pos[0], target_pos[1], 0.0, 0.0, target_orientation, 0.0])  # type: ignore

        pos_error = np.linalg.norm(current_position - np.array(target_pos))
        ang_error = abs(self.angle_difference(target_orientation, current_angle))

        if pos_error < self.position_tolerance and ang_error < self.angle_tolerance:
            if is_end_stabilization:
                # END stabilization logic
                if self.final_waypoint_stabilization_start_time is None:
                    self.final_waypoint_stabilization_start_time = current_time
                    print(
                        f" Satellite reached final waypoint. Stabilizing for {SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds before return..."
                    )
                else:
                    stabilization_time = (
                        current_time - self.final_waypoint_stabilization_start_time
                    )
                    if stabilization_time >= SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:
                        SatelliteConfig.DXF_SHAPE_PHASE = "RETURNING"  # type: ignore
                        SatelliteConfig.DXF_RETURN_START_TIME = current_time  # type: ignore
                        return_pos = SatelliteConfig.DXF_RETURN_POSITION  # type: ignore
                        print(" Path stabilization complete!")
                        print(f" Starting return to position ({return_pos[0]:.2f}, {return_pos[1]:.2f}) m")  # type: ignore
                        self.final_waypoint_stabilization_start_time = None
                        return None
            else:
                # START stabilization logic
                if self.shape_stabilization_start_time is None:
                    self.shape_stabilization_start_time = current_time
                    print(
                        f" Satellite reached path start. Stabilizing for {SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:.1f} seconds before tracking..."
                    )
                else:
                    stabilization_time = current_time - self.shape_stabilization_start_time
                    if stabilization_time >= SatelliteConfig.SHAPE_POSITIONING_STABILIZATION_TIME:
                        SatelliteConfig.DXF_SHAPE_PHASE = "TRACKING"  # type: ignore
                        SatelliteConfig.DXF_TRACKING_START_TIME = current_time  # type: ignore
                        SatelliteConfig.DXF_TARGET_START_DISTANCE = 0.0
                        print(" Path stabilization complete! Starting profile tracking...")
                        print(
                            f"   Target speed: {SatelliteConfig.DXF_TARGET_SPEED:.2f} m/s"
                        )
                        self.shape_stabilization_start_time = None
        else:
            # Reset stabilization timer if satellite drifts away
            if is_end_stabilization:
                self.final_waypoint_stabilization_start_time = None
            else:
                self.shape_stabilization_start_time = None

        return target_state

    def _dxf_stabilizing_phase(
        self, current_time: float, path: List[Tuple[float, float]]
    ) -> Optional[np.ndarray]:
        """Handle DXF stabilizing phase."""
        from mission import get_path_tangent_orientation

        final_pos = SatelliteConfig.DXF_FINAL_POSITION

        # Determine target orientation: use return angle if at return position, otherwise use path tangent
        has_return = getattr(SatelliteConfig, "DXF_HAS_RETURN", False)
        return_pos = getattr(SatelliteConfig, "DXF_RETURN_POSITION", None)
        at_return_position = (has_return and return_pos is not None and final_pos is not None and
                             np.allclose(final_pos, return_pos, atol=0.001))

        if at_return_position:
            # Stabilizing at return position - use return angle
            target_orientation = getattr(SatelliteConfig, "DXF_RETURN_ANGLE", 0.0) or 0.0
        else:
            # Stabilizing at end of path - use path tangent
            end_s = getattr(SatelliteConfig, "DXF_PATH_LENGTH", 0.0)
            target_orientation = get_path_tangent_orientation(
                path, end_s, SatelliteConfig.DXF_CLOSEST_POINT_INDEX
            )

        target_state = np.array([final_pos[0], final_pos[1], 0.0, 0.0, target_orientation, 0.0])  # type: ignore

        if SatelliteConfig.DXF_STABILIZATION_START_TIME is None:
            SatelliteConfig.DXF_STABILIZATION_START_TIME = current_time  # type: ignore

        stabilization_time = current_time - SatelliteConfig.DXF_STABILIZATION_START_TIME  # type: ignore
        if stabilization_time >= SatelliteConfig.SHAPE_FINAL_STABILIZATION_TIME:
            print(" PROFILE FOLLOWING MISSION COMPLETED!")
            print("   Profile successfully traversed and stabilized")
            self.dxf_completed = True
            return None  # Signal mission complete

        return target_state

    def _dxf_returning_phase(
        self, current_position: np.ndarray, current_angle: float, current_time: float
    ) -> Optional[np.ndarray]:
        """Handle DXF returning phase."""
        return_pos = SatelliteConfig.DXF_RETURN_POSITION  # type: ignore
        return_angle = SatelliteConfig.DXF_RETURN_ANGLE or 0.0  # type: ignore

        target_state = np.array([return_pos[0], return_pos[1], 0.0, 0.0, return_angle, 0.0])  # type: ignore

        pos_error = np.linalg.norm(current_position - np.array(return_pos))  # type: ignore
        ang_error = abs(self.angle_difference(return_angle, current_angle))

        # Only check position error for transition, not angle error during movement
        if pos_error < self.position_tolerance:
            # Now enforce angle error once at position
            if ang_error < self.angle_tolerance:
                # Reached return position - transition to STABILIZING phase
                if self.return_stabilization_start_time is None:
                    self.return_stabilization_start_time = current_time
                    SatelliteConfig.DXF_SHAPE_PHASE = "STABILIZING"  # type: ignore
                    SatelliteConfig.DXF_STABILIZATION_START_TIME = current_time  # type: ignore
                    SatelliteConfig.DXF_FINAL_POSITION = return_pos  # type: ignore
                    print(f" Reached return position! Transitioning to final stabilization...")
                    return None  # Let next update handle STABILIZING phase
            else:
                # Still at position but not at correct angle, keep trying
                self.return_stabilization_start_time = None
        else:
            self.return_stabilization_start_time = None

        return target_state

    def reset(self) -> None:
        """Reset all mission state for a new mission."""
        self.current_nav_waypoint_idx = 0
        self.nav_target_reached_time = None

        self.multi_point_target_reached_time = None

        self.shape_stabilization_start_time = None
        self.return_stabilization_start_time = None
        self.final_waypoint_stabilization_start_time = None
