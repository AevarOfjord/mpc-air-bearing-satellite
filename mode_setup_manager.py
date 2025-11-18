"""
Mode Setup Manager for Satellite Control System

Handles operation mode detection and setup for different mission types.
Encapsulates mode-specific initialization logic for clean mission configuration.

Supported modes:
- Point-to-Point: Direct navigation to a single target position
- Waypoint Navigation: Sequential navigation through multiple waypoints

Key features:
- Automatic mode detection from configuration
- Mode-specific variable initialization
- Waypoint list management
- Clean separation of mode logic from main controllers
"""

from config import SatelliteConfig


class ModeSetupManager:
    """
    Manages operation mode detection and setup for satellite missions.

    Detects which mission mode is active and initializes mode-specific
    variables and waypoints accordingly.
    """

    @staticmethod
    def detect_operation_mode() -> str:
        """
        Detect which operation mode is active based on SatelliteConfig settings.

        Returns:
            str: One of 'point-to-point', 'waypoint'
        """
        if (
            hasattr(SatelliteConfig, "ENABLE_WAYPOINT_MODE")
            and SatelliteConfig.ENABLE_WAYPOINT_MODE
        ):
            return "waypoint"
        else:
            return "point-to-point"

    @staticmethod
    def setup_mode_specific_variables(controller):
        """
        Setup mode-specific variables based on detected operation mode.

        Args:
            controller: The satellite controller instance to configure
        """
        controller.operation_mode = ModeSetupManager.detect_operation_mode()

        if controller.operation_mode == "waypoint":
            # Waypoint mode variables
            controller.next_multi_point_target = None
            print(
                f" WAYPOINT MODE: {len(SatelliteConfig.WAYPOINT_TARGETS)} targets configured"
            )

        else:
            print(
                f" POINT-TO-POINT MODE: Target ({controller.target_state[0]:.2f}, {controller.target_state[1]:.2f})"
            )


def create_mode_setup_manager():
    """Factory function to create a ModeSetupManager instance."""
    return ModeSetupManager()
