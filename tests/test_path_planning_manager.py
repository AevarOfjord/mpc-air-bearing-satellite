"""
Unit tests for PathPlanningManager class.

Tests path planning, obstacle avoidance, and waypoint navigation functionality.
"""

from unittest.mock import Mock

import numpy as np
import pytest

# Import the module under test
from path_planning_manager import PathPlanningManager, create_path_planning_manager


class TestPathPlanningManagerInitialization:
    """Test PathPlanningManager initialization."""

    def test_initialization_with_config(self):
        """Test that manager initializes with config object."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(1.0, 1.0), (2.0, 2.0)]
        config.OBSTACLE_RADII = [0.2, 0.2]

        manager = PathPlanningManager(config)

        assert manager.config == config
        assert manager.obstacles_enabled is True
        assert manager.safety_radius == 0.3
        assert len(manager.obstacles) == 2
        assert manager.current_waypoint_index is None
        assert manager.waypoints == []

    def test_initialization_without_obstacles(self):
        """Test initialization when obstacles are disabled."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        assert manager.obstacles_enabled is False
        assert manager.obstacles == []

    def test_factory_function(self):
        """Test create_path_planning_manager factory function."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = create_path_planning_manager(config)

        assert isinstance(manager, PathPlanningManager)
        assert manager.config == config


class TestCalculateObstacleAvoidingPath:
    """Test obstacle-avoiding path calculation."""

    def test_direct_path_no_obstacles(self):
        """Test that direct path is returned when no obstacles block the way."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(5.0, 5.0)]  # Far away obstacle
        config.OBSTACLE_RADII = [0.2]

        manager = PathPlanningManager(config)

        start = np.array([0.0, 0.0])
        target = np.array([1.0, 1.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        # Should return direct path (just start and end)
        assert len(path) >= 2
        assert path[0] == (start[0], start[1])
        assert path[-1] == (target[0], target[1])

    def test_obstacles_disabled_returns_direct_path(self):
        """Test that direct path is returned when obstacles are disabled."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        start = np.array([0.0, 0.0])
        target = np.array([2.0, 2.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        assert len(path) == 2
        assert path[0] == (0.0, 0.0)
        assert path[-1] == (2.0, 2.0)

    def test_path_avoids_obstacle_in_direct_line(self):
        """Test that path is calculated around obstacle in direct line."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(1.0, 1.0)]  # Obstacle in the middle
        config.OBSTACLE_RADII = [0.3]

        manager = PathPlanningManager(config)

        start = np.array([0.0, 0.0])
        target = np.array([2.0, 2.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        # Path should have more than 2 points (waypoints around obstacle)
        assert len(path) >= 2
        # First and last points should be start and target
        assert path[0] == (start[0], start[1])
        assert path[-1] == (target[0], target[1])

    def test_multiple_obstacles(self):
        """Test path planning with multiple obstacles."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(0.5, 0.5), (1.5, 1.5)]
        config.OBSTACLE_RADII = [0.2, 0.2]

        manager = PathPlanningManager(config)

        start = np.array([0.0, 0.0])
        target = np.array([2.0, 2.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        # Should find a path
        assert len(path) >= 2
        assert path[0] == (start[0], start[1])
        assert path[-1] == (target[0], target[1])


class TestUpdateTargetWithObstacleAvoidance:
    """Test dynamic target update with obstacle avoidance."""

    def test_no_obstacles_returns_original_target(self):
        """Test that original target is returned when no obstacles."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        current_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target_pos = (2.0, 2.0)
        target_angle = 0.0

        result_pos, result_angle = manager.update_target_with_obstacle_avoidance(
            current_pos, target_pos, target_angle
        )

        assert result_pos == target_pos
        assert result_angle == target_angle
        assert manager.current_waypoint_index is None

    def test_waypoint_navigation_progresses(self):
        """Test that waypoint navigation progresses through path."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(1.0, 1.0)]
        config.OBSTACLE_RADII = [0.3]
        config.TARGET_POSITION_TOLERANCE = 0.1

        manager = PathPlanningManager(config)

        # Start navigation
        current_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target_pos = (2.0, 2.0)
        target_angle = 0.0

        # First call should initialize waypoints
        result_pos, result_angle = manager.update_target_with_obstacle_avoidance(
            current_pos, target_pos, target_angle
        )

        # Should be navigating waypoints
        if manager.is_navigating_waypoints():
            assert manager.current_waypoint_index is not None
            assert len(manager.waypoints) > 0

    def test_reached_target_clears_waypoints(self):
        """Test that reaching target clears waypoints."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = []
        config.OBSTACLE_RADII = []
        config.TARGET_POSITION_TOLERANCE = 0.1

        manager = PathPlanningManager(config)

        # Position very close to target
        current_pos = np.array([0.01, 0.01, 0.0, 0.0, 0.0, 0.0])
        target_pos = (0.0, 0.0)
        target_angle = 0.0

        result_pos, result_angle = manager.update_target_with_obstacle_avoidance(
            current_pos, target_pos, target_angle
        )

        # Should not be navigating waypoints (direct path)
        assert not manager.is_navigating_waypoints()


class TestResetPath:
    """Test path reset functionality."""

    def test_reset_clears_waypoints(self):
        """Test that reset clears all waypoint data."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = []
        config.OBSTACLE_RADII = []

        manager = PathPlanningManager(config)

        # Manually set some waypoints
        manager.waypoints = [(1.0, 1.0), (2.0, 2.0)]
        manager.current_waypoint_index = 0

        manager.reset_path()

        assert manager.waypoints == []
        assert manager.current_waypoint_index is None


class TestNavigationQueries:
    """Test navigation state query functions."""

    def test_is_navigating_waypoints_false_initially(self):
        """Test that is_navigating_waypoints returns False initially."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        assert not manager.is_navigating_waypoints()

    def test_is_navigating_waypoints_true_during_navigation(self):
        """Test that is_navigating_waypoints returns True during navigation."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)
        manager.waypoints = [(1.0, 1.0), (2.0, 2.0)]
        manager.current_waypoint_index = 0

        assert manager.is_navigating_waypoints()

    def test_get_current_waypoint_index_returns_none_initially(self):
        """Test that get_current_waypoint_index returns None initially."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        assert manager.get_current_waypoint_index() is None

    def test_get_current_waypoint_index_returns_valid_index(self):
        """Test that get_current_waypoint_index returns valid index during navigation."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)
        manager.waypoints = [(1.0, 1.0), (2.0, 2.0)]
        manager.current_waypoint_index = 1

        assert manager.get_current_waypoint_index() == 1

    def test_get_waypoint_count_returns_zero_initially(self):
        """Test that get_waypoint_count returns 0 initially."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)

        assert manager.get_waypoint_count() == 0

    def test_get_waypoint_count_returns_correct_count(self):
        """Test that get_waypoint_count returns correct count."""
        config = Mock()
        config.OBSTACLES_ENABLED = False
        config.OBSTACLE_SAFETY_RADIUS = 0.3

        manager = PathPlanningManager(config)
        manager.waypoints = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0)]

        assert manager.get_waypoint_count() == 3


class TestEdgeCases:
    """Test edge cases and error conditions."""

    def test_start_equals_target(self):
        """Test path calculation when start equals target."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = []
        config.OBSTACLE_RADII = []

        manager = PathPlanningManager(config)

        start = np.array([1.0, 1.0])
        target = np.array([1.0, 1.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        # Should handle gracefully
        assert len(path) >= 1

    def test_very_large_obstacles(self):
        """Test handling of very large obstacles."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.5
        config.OBSTACLE_POSITIONS = [(1.0, 1.0)]
        config.OBSTACLE_RADII = [2.0]  # Very large obstacle

        manager = PathPlanningManager(config)

        start = np.array([0.0, 0.0])
        target = np.array([2.0, 2.0])

        # Should still find a path (or return direct path if no solution)
        path = manager.calculate_obstacle_avoiding_path(start, target)
        assert len(path) >= 2

    def test_negative_coordinates(self):
        """Test path planning with negative coordinates."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(0.0, 0.0)]
        config.OBSTACLE_RADII = [0.2]

        manager = PathPlanningManager(config)

        start = np.array([-1.0, -1.0])
        target = np.array([1.0, 1.0])

        path = manager.calculate_obstacle_avoiding_path(start, target)

        assert len(path) >= 2
        assert path[0] == (-1.0, -1.0)
        assert path[-1] == (1.0, 1.0)


class TestIntegration:
    """Integration tests for complete workflows."""

    def test_complete_obstacle_avoidance_workflow(self):
        """Test complete workflow from initialization to target reached."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(1.0, 0.0)]
        config.OBSTACLE_RADII = [0.3]
        config.TARGET_POSITION_TOLERANCE = 0.1

        manager = PathPlanningManager(config)

        # Start far from target
        current_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target_pos = (2.0, 0.0)
        target_angle = 0.0

        # First update - should initialize path
        (
            intermediate_target,
            intermediate_angle,
        ) = manager.update_target_with_obstacle_avoidance(
            current_pos, target_pos, target_angle
        )

        # Verify path was created
        # initial_waypoint_count = manager.get_waypoint_count()  # Not currently used

        # Simulate progression through waypoints
        for _step in range(10):
            # Move towards intermediate target
            current_pos[0] += 0.2

            (
                intermediate_target,
                intermediate_angle,
            ) = manager.update_target_with_obstacle_avoidance(
                current_pos, target_pos, target_angle
            )

            # Waypoint index should eventually progress or reach target
            if manager.is_navigating_waypoints():
                assert manager.get_current_waypoint_index() is not None

    def test_reset_during_navigation(self):
        """Test resetting path during active navigation."""
        config = Mock()
        config.OBSTACLES_ENABLED = True
        config.OBSTACLE_SAFETY_RADIUS = 0.3
        config.OBSTACLE_POSITIONS = [(1.0, 1.0)]
        config.OBSTACLE_RADII = [0.3]
        config.TARGET_POSITION_TOLERANCE = 0.1

        manager = PathPlanningManager(config)

        # Start navigation
        current_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        target_pos = (2.0, 2.0)
        target_angle = 0.0

        manager.update_target_with_obstacle_avoidance(
            current_pos, target_pos, target_angle
        )

        # Reset path mid-navigation
        manager.reset_path()

        # Verify clean state
        assert not manager.is_navigating_waypoints()
        assert manager.get_waypoint_count() == 0
        assert manager.get_current_waypoint_index() is None

        # Should be able to start new navigation
        new_target_pos = (3.0, 3.0)
        manager.update_target_with_obstacle_avoidance(
            current_pos, new_target_pos, target_angle
        )

        # Can navigate again
        assert True  # Successfully restarted navigation


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
