#!/usr/bin/env python3
"""
Real Data Simulated - Replay Real Thruster Commands Through Simulation

Takes thruster command sequence from a real hardware test CSV file and
replays them through the simulation physics engine to create an animation
showing what the simulated physics predicts.

This allows visual comparison between real hardware behavior and simulated physics.

Usage:
    python real_data_simulated.py
"""

import ast
import json
from pathlib import Path

import numpy as np
import pandas as pd

from config import SatelliteConfig
from testing_environment import SatelliteThrusterTester


def load_dxf_shape_config(test_directory: Path):
    """Load DXF shape configuration from test directory if it exists.

    Args:
        test_directory: Path to test directory

    Returns:
        bool: True if DXF config was loaded, False otherwise
    """
    dxf_config_file = test_directory / "dxf_shape_config.json"

    if not dxf_config_file.exists():
        return False

    try:
        with open(dxf_config_file, "r") as f:
            config = json.load(f)

        # Restore DXF configuration to SatelliteConfig
        SatelliteConfig.DXF_SHAPE_MODE_ACTIVE = config.get("active", False)
        if "base_shape" in config:
            SatelliteConfig.DXF_BASE_SHAPE = [tuple(p) for p in config["base_shape"]]
        if "shape_path" in config:
            SatelliteConfig.DXF_SHAPE_PATH = [tuple(p) for p in config["shape_path"]]
        if "shape_center" in config:
            SatelliteConfig.DXF_SHAPE_CENTER = tuple(config["shape_center"])  # type: ignore[assignment]
        if "shape_rotation" in config:
            SatelliteConfig.DXF_SHAPE_ROTATION = config["shape_rotation"]

        print(f" Loaded DXF shape configuration from {dxf_config_file}")
        return True

    except Exception as e:
        print(f"  Failed to load DXF config: {e}")
        return False


class RealDataSimulationReplay:
    """Replays real test thruster commands through simulation and creates animation."""

    def __init__(self, real_test_csv_path: str, overlay_dxf: bool = False):
        """Initialize the replay system.

        Args:
            real_test_csv_path: Path to real test CSV file
            overlay_dxf: If True, overlay DXF shape on animation
        """
        self.csv_path = Path(real_test_csv_path)
        self.commands = []
        self.initial_state = None
        self.satellite = None
        self.overlay_dxf = overlay_dxf

        # Animation data storage
        self.trajectory_data = []
        self.state_data = []

        print(f" Loading real test data from: {self.csv_path}")

    def load_thruster_sequence(self):
        """Load thruster command sequence from real test CSV."""
        print(" Extracting thruster command sequence...")

        # Try to load DXF shape config if it exists
        dxf_loaded = load_dxf_shape_config(self.csv_path.parent)
        if dxf_loaded:
            print(" DXF shape configuration will be overlaid on animation")
            self.overlay_dxf = True  # Auto-enable overlay if DXF config exists

        # Load CSV
        df = pd.read_csv(self.csv_path)
        print(f" Loaded {len(df)} data points")

        # Get initial state from first row
        first = df.iloc[0]
        self.initial_state = {
            "x": float(first["Current_X"]),  # type: ignore[arg-type]
            "y": float(first["Current_Y"]),  # type: ignore[arg-type]
            "yaw": float(first["Current_Yaw"]),  # type: ignore[arg-type]
            "vx": float(first["Current_VX"]),  # type: ignore[arg-type]
            "vy": float(first["Current_VY"]),  # type: ignore[arg-type]
            "omega": float(first["Current_Angular_Vel"]),  # type: ignore[arg-type]
        }

        print(
            f" Initial state: Pos({self.initial_state['x']:.3f}, {self.initial_state['y']:.3f}), "
            f"Yaw {np.degrees(self.initial_state['yaw']):.1f}°"
        )

        # Extract thruster commands
        for _idx, row in df.iterrows():
            cmd_str = str(row["Command_Vector"])
            cmd = ast.literal_eval(cmd_str)  # Parse "[0, 1, 0, 1, ...]"
            self.commands.append(
                {
                    "time": float(row["Control_Time"]),  # type: ignore[arg-type]
                    "command": cmd,
                }
            )

        print(f" Extracted {len(self.commands)} thruster commands")

    def run_simulation(self):
        """Run simulation with the thruster command sequence."""
        print("\n Running simulation with real thruster commands...")
        print("=" * 80)

        assert (
            self.initial_state is not None
        ), "Initial state not loaded. Call load_thruster_sequence() first."

        # Create satellite physics environment
        self.satellite = SatelliteThrusterTester()

        # Set initial state from real test
        self.satellite.position = np.array(
            [self.initial_state["x"], self.initial_state["y"]]
        )
        self.satellite.velocity = np.array(
            [self.initial_state["vx"], self.initial_state["vy"]]
        )
        self.satellite.angle = self.initial_state["yaw"]
        self.satellite.angular_velocity = self.initial_state["omega"]

        # Set title
        self.satellite.set_title("Real Test Commands → Simulated Physics")

        # Timing
        control_dt = SatelliteConfig.CONTROL_DT
        simulation_dt = SatelliteConfig.SIMULATION_DT
        steps_per_control = int(control_dt / simulation_dt)

        print(f"⏱  Control timestep: {control_dt * 1000:.0f} ms")
        print(f"⏱  Physics timestep: {simulation_dt * 1000:.0f} ms")
        print(f" Physics updates per control: {steps_per_control}")
        print()

        # Run through each command
        for cmd_idx, cmd_info in enumerate(self.commands):
            cmd = cmd_info["command"]

            # Apply thruster command
            self.satellite.active_thrusters.clear()
            for thruster_num, is_active in enumerate(cmd, start=1):
                if is_active:
                    self.satellite.active_thrusters.add(thruster_num)

            # Run physics for this control interval
            for _ in range(steps_per_control):
                self.satellite.update_simulation(None)

            # Store state for animation
            self.trajectory_data.append(self.satellite.position.copy())
            self.state_data.append(
                {
                    "time": cmd_info["time"],
                    "pos": self.satellite.position.copy(),
                    "vel": self.satellite.velocity.copy(),
                    "angle": self.satellite.angle,
                    "omega": self.satellite.angular_velocity,
                    "thrusters": cmd.copy(),
                }
            )

            # Progress
            if (cmd_idx + 1) % 100 == 0 or cmd_idx == len(self.commands) - 1:
                print(
                    f"Progress: {100 * (cmd_idx + 1) / len(self.commands):.1f}% "
                    f"({cmd_idx + 1}/{len(self.commands)}) - "
                    f"Pos: ({self.satellite.position[0]:.2f}, {self.satellite.position[1]:.2f})"
                )

        print()
        print(" Simulation complete!")
        print(
            f" Final position: ({self.satellite.position[0]:.3f}, {self.satellite.position[1]:.3f})"
        )
        print(f" Final angle: {np.degrees(self.satellite.angle):.1f}°")

    def save_to_csv(self):
        """Save simulation results to CSV for visualization."""
        print("\n Saving simulation data to CSV...")

        # Extract the test name from the real test directory
        # e.g., "22-10-2025_19-52-06-Test1 copy" -> "22-10-2025_19-52-06-Test1"
        real_test_dir_name = self.csv_path.parent.name
        # Remove " copy" if present and extract just the date-time part
        test_name = real_test_dir_name.replace(" copy", "").split("-Test")[0]

        # Create output directory: Data/Real_Data_Simulated/<test_name>
        base_output = Path("Data/Real_Data_Simulated")
        output_dir = base_output / test_name
        output_dir.mkdir(parents=True, exist_ok=True)

        # Prepare data for CSV (matching real test format)
        csv_data = []
        real_df = pd.read_csv(self.csv_path)
        control_dt = SatelliteConfig.CONTROL_DT

        for idx, state in enumerate(self.state_data):
            # Get target from real data
            real_row = real_df.iloc[idx]

            row = {
                "Step": idx,
                "Time": state["time"],
                "CONTROL_DT": control_dt,  # Add this so Visualize.py uses correct timestep
                "Current_X": state["pos"][0],
                "Current_Y": state["pos"][1],
                "Current_Yaw": state["angle"],
                "Current_VX": state["vel"][0],
                "Current_VY": state["vel"][1],
                "Current_Angular_Vel": state["omega"],
                "Target_X": float(real_row["Target_X"]),  # type: ignore[arg-type]
                "Target_Y": float(real_row["Target_Y"]),  # type: ignore[arg-type]
                "Target_Yaw": float(real_row["Target_Yaw"]),  # type: ignore[arg-type]
                "Error_X": state["pos"][0] - float(real_row["Target_X"]),  # type: ignore[arg-type]
                "Error_Y": state["pos"][1] - float(real_row["Target_Y"]),  # type: ignore[arg-type]
                "Error_Yaw": state["angle"] - float(real_row["Target_Yaw"]),  # type: ignore[arg-type]
                "Command_Vector": str(state["thrusters"]),
            }

            # Add individual thruster columns
            for i in range(8):
                row[f"Thruster_{i + 1}"] = state["thrusters"][i]

            csv_data.append(row)

        # Save to CSV
        df = pd.DataFrame(csv_data)
        csv_path = output_dir / "simulation_data.csv"
        df.to_csv(csv_path, index=False)

        print(f" Saved to: {csv_path}")
        print(f" Output directory: Data/Real_Data_Simulated/{test_name}")
        return output_dir, test_name

    def create_visualization(self):
        """Create animation and plots using Visualize.py (exact same format)."""
        print("\n Creating visualization using Visualize.py format...")
        print("=" * 80)

        # First save data to CSV
        output_dir, test_name = self.save_to_csv()

        # Use UnifiedVisualizationGenerator for exact same format
        from visualize import UnifiedVisualizationGenerator

        viz = UnifiedVisualizationGenerator(
            data_directory=str(output_dir),
            mode="simulation",
            interactive=False,
            load_data=True,
            overlay_dxf=self.overlay_dxf,
        )

        # Generate plots
        print(" Generating performance plots...")
        viz.generate_performance_plots()

        # Generate animation with test name
        animation_filename = f"{test_name}_Simulated.mp4"
        print(f" Generating animation: {animation_filename}")
        viz.generate_animation(output_filename=animation_filename)

        print(" Visualization complete!")
        return str(output_dir), test_name


def main():
    """Main function."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Replay real test thruster commands through simulated physics"
    )
    parser.add_argument(
        "--overlay-dxf",
        action="store_true",
        help="Overlay DXF shape on animation (if available in Config)",
    )
    parser.add_argument(
        "--real-test",
        type=str,
        default=None,
        help="Path to real test CSV file (default: Data/Real_Test/22-10-2025_19-52-06-Test1 copy/real_test_data.csv)",
    )

    args = parser.parse_args()

    print("=" * 80)
    print("REAL TEST COMMANDS → SIMULATED PHYSICS ANIMATION")
    print("=" * 80)
    print()
    print("This tool takes thruster commands from a real hardware test")
    print("and replays them through the simulation physics engine to")
    print("create an animation showing the simulated response.")
    if args.overlay_dxf:
        print()
        print(" DXF shape overlay: ENABLED")
    print()

    # Real test CSV path
    real_test_csv = (
        args.real_test
        or "Data/Real_Test/22-10-2025_19-52-06-Test1 copy/real_test_data.csv"
    )

    if not Path(real_test_csv).exists():
        print(f" Error: File not found: {real_test_csv}")
        return 1

    try:
        # Create replayer
        replayer = RealDataSimulationReplay(real_test_csv, overlay_dxf=args.overlay_dxf)

        # Load thruster sequence
        replayer.load_thruster_sequence()

        # Run simulation
        replayer.run_simulation()

        # Create visualization using Visualize.py format
        output_dir, test_name = replayer.create_visualization()

        print()
        print("=" * 80)
        print(" SUCCESS!")
        print("=" * 80)
        print(f" Output directory: {output_dir}")
        print(f" Animation name: {test_name}_Simulated.mp4")
        print()
        print("The animation and plots use the EXACT same format as Visualize.py")
        print("for direct visual comparison with real test results!")
        print("=" * 80)

        return 0

    except Exception as e:
        print(f"\n Error: {e}")
        import traceback

        traceback.print_exc()
        return 1


if __name__ == "__main__":
    import sys

    sys.exit(main())
