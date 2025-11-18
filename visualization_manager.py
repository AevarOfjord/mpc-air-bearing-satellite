"""
Visualization Manager for Satellite Control System

Handles visualization and animation generation for both real and simulated tests.
Separates visualization logic from core controller logic for better modularity.

Visualization capabilities:
- Real-time matplotlib plotting during hardware tests
- Post-test animation generation with MP4 export
- Trajectory plots with waypoints and obstacles
- State history visualization
- Thruster activation indicators

Animation features:
- High-quality MP4 video export
- Configurable frame rate and resolution
- Progress bar with encoding status
- Automatic legend and axis configuration
- Consistent styling with simulation visualizations

Key features:
- Modular design for easy integration
- Progress suppression for clean console output
- Integration with visualize.py for comprehensive reports
- Memory-efficient frame rendering
- Support for both real hardware and simulation data
"""

import io
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

# Visualization manager for real and simulation tests


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


class RealTimeVisualizer:
    """
    Manages real-time matplotlib visualization during hardware tests.

    Displays:
    - Satellite trajectory
    - Current position and orientation
    - Target position and tolerance circle
    - Real-time updates during test execution
    """

    def __init__(self):
        """Initialize the real-time visualizer."""
        self.fig = None
        self.ax = None
        self.trajectory = []
        self.max_trajectory_points = 1000

    def setup(self):
        """Configure matplotlib display for real-time monitoring."""
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("X Position (m)", fontsize=12, fontweight="bold")
        self.ax.set_ylabel("Z Position (m)", fontsize=12, fontweight="bold")
        self.ax.set_title("Real Satellite MPC Control", fontsize=14, fontweight="bold")

        # Set initial display area
        self.ax.set_xlim(-1, 3)
        self.ax.set_ylim(-1, 3)

    def draw(
        self,
        current_position: np.ndarray,
        current_angle: float,
        target_state: np.ndarray,
        position_tolerance: float,
    ):
        """
        Draw the real-time visualization.

        Args:
            current_position: Current satellite position [x, y]
            current_angle: Current satellite orientation (radians)
            target_state: Target state [x, y, vx, vy, angle, angular_vel]
            position_tolerance: Position tolerance for target circle
        """
        if self.ax is None:
            return

        # Clear the axes
        self.ax.clear()

        # Reset axis properties
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("X Position (m)", fontsize=12, fontweight="bold")
        self.ax.set_ylabel("Z Position (m)", fontsize=12, fontweight="bold")
        self.ax.set_title("Real Satellite MPC Control", fontsize=14, fontweight="bold")

        # Draw target
        target_x = float(target_state[0])
        target_y = float(target_state[1])
        target_angle = float(target_state[4])

        # Point-to-point mode - normal target tolerance circle
        target_circle = Circle(
            (target_x, target_y),
            position_tolerance,  # type: ignore[arg-type]
            fill=False,
            color="green",
            linestyle="--",
            alpha=0.5,
        )
        self.ax.add_patch(target_circle)

        # Target orientation indicator
        target_arrow_length = 0.2
        target_arrow_end = np.array(
            [target_x, target_y]
        ) + target_arrow_length * np.array([np.cos(target_angle), np.sin(target_angle)])
        self.ax.annotate(
            "",
            xy=(float(target_arrow_end[0]), float(target_arrow_end[1])),
            xytext=(target_x, target_y),
            arrowprops=dict(arrowstyle="->", lw=3, color="darkgreen", alpha=0.8),
        )

        # Target position marker
        self.ax.plot(
            target_x,
            target_y,
            "ro",
            markersize=12,
            markerfacecolor="red",
            markeredgecolor="darkred",
            linewidth=2,
        )

        # Draw trajectory
        if len(self.trajectory) > 1:
            trajectory_array = np.array(self.trajectory)
            self.ax.plot(
                trajectory_array[:, 0],
                trajectory_array[:, 1],
                "b-",
                alpha=0.6,
                linewidth=2,
            )

        # Draw current satellite position
        if len(self.trajectory) > 0:
            sat_x, sat_y = current_position

            # Satellite body
            self.ax.plot(
                sat_x,
                sat_y,
                "bo",
                markersize=10,
                markerfacecolor="blue",
                markeredgecolor="darkblue",
                linewidth=2,
            )

            # Satellite orientation indicator
            sat_arrow_length = 0.15
            sat_arrow_end = np.array([sat_x, sat_y]) + sat_arrow_length * np.array(
                [np.cos(current_angle), np.sin(current_angle)]
            )
            self.ax.annotate(
                "",
                xy=(float(sat_arrow_end[0]), float(sat_arrow_end[1])),
                xytext=(float(sat_x), float(sat_y)),
                arrowprops=dict(arrowstyle="->", lw=2, color="darkblue", alpha=0.8),
            )

        # Update axis limits to keep satellite in view
        if len(self.trajectory) > 0:
            sat_x, sat_y = current_position
            margin = 1.0
            self.ax.set_xlim(
                min(sat_x - margin, target_x - margin),
                max(sat_x + margin, target_x + margin),
            )
            self.ax.set_ylim(
                min(sat_y - margin, target_y - margin),
                max(sat_y + margin, target_y + margin),
            )

        plt.draw()
        plt.pause(0.01)  # Small pause to update display

    def update_trajectory(self, position: np.ndarray):
        """
        Update the trajectory with a new position.

        Args:
            position: New position to add [x, y]
        """
        self.trajectory.append(tuple(position))
        if len(self.trajectory) > self.max_trajectory_points:
            self.trajectory.pop(0)


class PostTestVisualizer:
    """
    Generates post-test visualizations (animations and plots).

    Uses UnifiedVisualizationGenerator to create:
    - MP4 animation of test execution
    - Performance plots (position, velocity, control, etc.)
    """

    @staticmethod
    def generate_visualizations(data_save_path: Path, UnifiedVisualizationGenerator):
        """
        Automatically generate all visualizations after test completion.

        Args:
            data_save_path: Path to test data directory
            UnifiedVisualizationGenerator: Visualization generator class (from Visualize.py)
        """
        if UnifiedVisualizationGenerator is None:
            print(
                "  Visualization components not available. Skipping auto-visualization."
            )
            return

        if data_save_path is None:
            print("  No data path available. Skipping auto-visualization.")
            return

        try:
            print("\n Animation, Plots and Summary will now be generated!")

            generator = UnifiedVisualizationGenerator(
                data_directory=str(data_save_path.parent),
                mode="real",
                prefer_pandas=False,
            )

            # Override paths to use current test data
            generator.csv_path = data_save_path / "real_test_data.csv"
            generator.output_dir = data_save_path

            # Load the data silently
            old_stdout = sys.stdout
            sys.stdout = io.StringIO()  # Suppress output
            try:
                generator.load_csv_data()
            finally:
                sys.stdout = old_stdout

            # Generate animation
            try:
                print("\nCreating animation...")
                animation_path = data_save_path / "Real_Test_animation.mp4"
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

            # Generate performance plots
            try:
                print("\nCreating Plots...")
                plots_path = data_save_path / "Plots"
                print(f"Saving Plots to: {plots_path}")

                # Allow progress output but suppress other verbose output
                old_stdout = sys.stdout
                sys.stdout = ProgressSuppressor(sys.stdout)
                try:
                    generator.generate_performance_plots()
                finally:
                    sys.stdout = old_stdout

                print(" Plots saved successfully!")
                print(f" File location: {plots_path}")
            except Exception as plots_err:
                print(f"  Performance plots generation failed: {plots_err}")

        except Exception as e:
            print(f" Error during auto-visualization: {e}")
            print(" You can manually run visualizations later using Visualize.py")


def create_realtime_visualizer() -> RealTimeVisualizer:
    """
    Factory function to create a real-time visualizer.

    Returns:
        Configured RealTimeVisualizer instance
    """
    return RealTimeVisualizer()
