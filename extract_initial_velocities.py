"""
Extract Initial Velocities from Real Test Data

Extracts averaged initial velocities from a real test CSV file.
Reads the velocity columns (Current_VX, Current_VY, Current_Angular_Vel)
and averages them over all rows to provide initial conditions for simulation.

Usage:
    python extract_initial_velocities.py <csv_file>

Example:
    python extract_initial_velocities.py Data/Real_Test/Test1/initial_state.csv

The script will:
1. Read all rows from the CSV file
2. Extract velocity columns
3. Average over all rows to reduce measurement noise
4. Display the averaged initial velocities
5. Optionally save to a file for use in simulations
"""

import os
import sys

import numpy as np
import pandas as pd


def extract_initial_velocities(csv_path: str, verbose: bool = True):
    """
    Extract and average initial velocities from a CSV file.

    Args:
        csv_path: Path to the CSV file containing real test data
        verbose: If True, print detailed information

    Returns:
        dict: Dictionary containing averaged velocities:
              {'vx': float, 'vy': float, 'omega': float, 'num_samples': int}
    """
    # Check if file exists
    if not os.path.exists(csv_path):
        print(f"ERROR: File not found: {csv_path}")
        return None

    try:
        # Read CSV file
        if verbose:
            print(f"\nReading data from: {csv_path}")

        df = pd.read_csv(csv_path)

        # Check if required columns exist
        required_columns = ["Current_VX", "Current_VY", "Current_Angular_Vel"]
        missing_columns = [col for col in required_columns if col not in df.columns]

        if missing_columns:
            print(f"ERROR: Missing required columns: {missing_columns}")
            print(f"Available columns: {list(df.columns)}")
            return None

        # Get number of samples
        num_samples = len(df)

        if num_samples == 0:
            print("ERROR: CSV file is empty")
            return None

        if verbose:
            print(f"Found {num_samples} data samples")

        # Extract velocity columns and convert to numpy arrays
        vx_data = df["Current_VX"].to_numpy()
        vy_data = df["Current_VY"].to_numpy()
        omega_data = df["Current_Angular_Vel"].to_numpy()

        # Calculate averages
        vx_avg = np.mean(vx_data)
        vy_avg = np.mean(vy_data)
        omega_avg = np.mean(omega_data)

        # Calculate standard deviations (for noise assessment)
        vx_std = np.std(vx_data)
        vy_std = np.std(vy_data)
        omega_std = np.std(omega_data)

        # Also extract final position and orientation if available
        position_data = {}
        if (
            "Current_X" in df.columns
            and "Current_Y" in df.columns
            and "Current_Yaw" in df.columns
        ):
            position_data = {
                "x": df["Current_X"].iloc[-1],
                "y": df["Current_Y"].iloc[-1],
                "theta": df["Current_Yaw"].iloc[-1],
            }

        if verbose:
            print("\n" + "=" * 60)
            print("INITIAL VELOCITY EXTRACTION RESULTS")
            print("=" * 60)
            print(f"\nStatistics (averaged over {num_samples} samples):")
            print(
                f"  - Sampling duration:     {num_samples * 0.06:.2f} seconds (@ 60ms intervals)"
            )
            print("\nAVERAGED INITIAL VELOCITIES:")
            print(f"  - X velocity (vx):       {vx_avg:+.6f} m/s  (std: {vx_std:.6f})")
            print(f"  - Y velocity (vy):       {vy_avg:+.6f} m/s  (std: {vy_std:.6f})")
            print(
                f"  - Angular velocity (ω):  {omega_avg:+.6f} rad/s  (std: {omega_std:.6f})"
            )
            print(f"                           {np.degrees(omega_avg):+.3f} deg/s")

            # Compute magnitude
            v_magnitude = np.sqrt(vx_avg**2 + vy_avg**2)
            print(f"\nVelocity Magnitude:     {v_magnitude:.6f} m/s")

            if v_magnitude > 0.001:  # Only compute direction if significant motion
                v_direction = np.degrees(np.arctan2(vy_avg, vx_avg))
                print(f"   Velocity Direction:     {v_direction:.1f}°")

            if position_data:
                print("\nFinal Position (last sample):")
                print(f"  - X position:            {position_data['x']:.6f} m")
                print(f"  - Y position:            {position_data['y']:.6f} m")
                print(f"  - Orientation (θ):       {position_data['theta']:.6f} rad")
                print(
                    f"                           {np.degrees(position_data['theta']):.1f}°"
                )

            print("\n" + "=" * 60)
            print("USE IN SIMULATION:")
            print("=" * 60)
            print("When prompted in Simulation.py, enter:")
            print(f"  X velocity:              {vx_avg:.6f}")
            print(f"  Y velocity:              {vy_avg:.6f}")
            print(f"  Angular velocity:        {omega_avg:.6f}")
            print("=" * 60 + "\n")

        # Return results as dictionary
        results = {
            "vx": vx_avg,
            "vy": vy_avg,
            "omega": omega_avg,
            "vx_std": vx_std,
            "vy_std": vy_std,
            "omega_std": omega_std,
            "num_samples": num_samples,
            "duration": num_samples * 0.06,
            "position": position_data if position_data else None,
        }

        return results

    except Exception as e:
        print(f"ERROR: Error processing file: {e}")
        import traceback

        traceback.print_exc()
        return None


def save_initial_conditions(results: dict, output_path: str):
    """
    Save extracted initial conditions to a text file.

    Args:
        results: Dictionary from extract_initial_velocities()
        output_path: Path to save the output file
    """
    try:
        with open(output_path, "w") as f:
            f.write("Initial Conditions for Simulation\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Extracted from {results['num_samples']} samples ")
            f.write(f"({results['duration']:.2f} seconds)\n\n")
            f.write("INITIAL VELOCITIES:\n")
            f.write(f"  vx    = {results['vx']:.6f} m/s\n")
            f.write(f"  vy    = {results['vy']:.6f} m/s\n")
            f.write(
                f"  omega = {results['omega']:.6f} rad/s ({np.degrees(results['omega']):.3f} deg/s)\n\n"
            )
            f.write("VELOCITY STANDARD DEVIATIONS:\n")
            f.write(f"  vx_std    = {results['vx_std']:.6f} m/s\n")
            f.write(f"  vy_std    = {results['vy_std']:.6f} m/s\n")
            f.write(f"  omega_std = {results['omega_std']:.6f} rad/s\n\n")

            if results["position"]:
                f.write("FINAL POSITION (last sample):\n")
                f.write(f"  x     = {results['position']['x']:.6f} m\n")
                f.write(f"  y     = {results['position']['y']:.6f} m\n")
                f.write(
                    f"  theta = {results['position']['theta']:.6f} rad ({np.degrees(results['position']['theta']):.1f}°)\n"
                )

        print(f"Initial conditions saved to: {output_path}")

    except Exception as e:
        print(f"ERROR: Error saving file: {e}")


def main():
    """Main function for command-line usage."""
    print("\n" + "=" * 60)
    print("INITIAL VELOCITY EXTRACTOR")
    print("=" * 60)

    # Check if CSV file provided as command-line argument
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
    else:
        # Interactive mode - ask for file path
        print("\nEnter the path to the CSV file containing real test data.")
        print("Examples:")
        print("  - Data/Real_Test/Test1/real_test_data.csv")
        print("  - Data/Real_Test/Test2/real_test_data.csv")
        print()

        csv_path = input("CSV file path: ").strip()

        # Remove quotes if user copy-pasted a path with quotes
        csv_path = csv_path.strip('"').strip("'")

        if not csv_path:
            print("\nERROR: No file path provided")
            sys.exit(1)

    # Extract velocities
    results = extract_initial_velocities(csv_path, verbose=True)

    if results is None:
        print("\nFailed to extract velocities")
        sys.exit(1)

    # Ask if user wants to save results
    save_option = input("\nSave results to file? (y/n): ").strip().lower()

    if save_option in ["y", "yes"]:
        # Generate output filename
        base_name = os.path.splitext(csv_path)[0]
        output_path = f"{base_name}_initial_conditions.txt"

        custom_path = input(f"\nOutput file path (default: {output_path}): ").strip()
        if custom_path:
            output_path = custom_path

        save_initial_conditions(results, output_path)

    print("\nDone!")


if __name__ == "__main__":
    main()
