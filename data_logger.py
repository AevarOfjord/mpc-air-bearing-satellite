"""
Data Logger for Satellite Control System

Centralized data logging and CSV export for both simulation and real hardware testing.
Handles step-by-step data collection and exports to standardized CSV format.

Key features:
- Dual-mode operation: Simulation and real hardware test modes
- Detailed log data with full state history per timestep
- Terminal message logging for debugging and analysis
- Automatic CSV export with mode-specific headers
- Configurable save paths with automatic directory creation
"""

import csv
from pathlib import Path
from typing import Any, Dict, List, Optional


class DataLogger:
    """
    Centralized data logging and CSV export.

    Handles logging of simulation/real test data and exports to CSV format
    for analysis. Used by both RealSatelliteMPCLinearized and
    SatelliteMPCLinearizedSimulation.
    """

    def __init__(self, mode: str = "simulation"):
        """
        Initialize data logger.

        Args:
            mode: Either "simulation" or "real" to determine output filenames
        """
        self.mode = mode.lower()
        if self.mode not in ["simulation", "real"]:
            raise ValueError("Mode must be either 'simulation' or 'real'")

        self.detailed_log_data: List[Dict[str, Any]] = []
        self.terminal_log_data: List[Dict[str, Any]] = []
        self.data_save_path: Optional[Path] = None
        self.current_step = 0

    def set_save_path(self, path: Path) -> None:
        """
        Set the directory path where data will be saved.

        Args:
            path: Directory path for saving data files
        """
        self.data_save_path = path
        if not path.exists():
            path.mkdir(parents=True, exist_ok=True)

    def log_entry(self, entry: Dict[str, Any]) -> None:
        """
        Add a log entry to the detailed log data.

        Args:
            entry: Dictionary containing log data for one timestep
        """
        self.detailed_log_data.append(entry)
        self.current_step += 1

    def log_terminal_message(self, message_data: Dict[str, Any]) -> None:
        """
        Add a terminal output message to the terminal log.

        Args:
            message_data: Dictionary containing terminal message data
                Expected keys: time, status, stabilization_time (optional),
                pos_error, ang_error, thrusters, solve_time, next_update (optional)
        """
        self.terminal_log_data.append(message_data)

    def save_csv_data(self) -> bool:
        """
        Export all recorded data to CSV file for analysis.
        Also saves terminal log if available.

        Returns:
            True if save successful, False otherwise
        """
        if not self.data_save_path:
            return False

        success = True
        wrote_any_files = False

        # Save detailed data log
        if self.detailed_log_data:
            # Determine filename based on mode
            if self.mode == "simulation":
                csv_file_path = self.data_save_path / "simulation_data.csv"
                headers = self._get_simulation_headers()
            else:  # real
                csv_file_path = self.data_save_path / "real_test_data.csv"
                headers = self._get_real_headers()

            try:
                with open(csv_file_path, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(headers)

                    for log_entry in self.detailed_log_data:
                        row = [
                            self._format_value(header, log_entry.get(header, ""))
                            for header in headers
                        ]
                        writer.writerow(row)

                print(f" CSV data saved to: {csv_file_path}")
                wrote_any_files = True
            except Exception as e:
                print(f" Error saving CSV data: {e}")
                success = False

        # Save terminal log
        if self.terminal_log_data:
            terminal_log_path = self.data_save_path / f"{self.mode}_terminal_log.csv"
            try:
                with open(terminal_log_path, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(self._get_terminal_log_headers())

                    for log_entry in self.terminal_log_data:
                        row = [
                            self._format_terminal_value(header, log_entry.get(header, ""))
                            for header in self._get_terminal_log_headers()
                        ]
                        writer.writerow(row)

                print(f" Terminal log saved to: {terminal_log_path}")
                wrote_any_files = True
            except Exception as e:
                print(f" Error saving terminal log: {e}")
                success = False

        return success and wrote_any_files

    def _get_simulation_headers(self) -> List[str]:
        """Get CSV headers for simulation mode (identical to real hardware format)."""
        return [
            "Step",
            "MPC_Start_Time",
            "Control_Time",
            "Actual_Time_Interval",
            "CONTROL_DT",
            "Mission_Phase",
            "Waypoint_Number",
            "Telemetry_X_mm",
            "Telemetry_Z_mm",
            "Telemetry_Yaw_deg",
            "Current_X",
            "Current_Y",
            "Current_Yaw",
            "Current_VX",
            "Current_VY",
            "Current_Angular_Vel",
            "Target_X",
            "Target_Y",
            "Target_Yaw",
            "Target_VX",
            "Target_VY",
            "Target_Angular_Vel",
            "Error_X",
            "Error_Y",
            "Error_Yaw",
            "Error_VX",
            "Error_VY",
            "Error_Angular_Vel",
            "MPC_Computation_Time",
            "MPC_Status",
            "MPC_Solver",
            "MPC_Solver_Time_Limit",
            "MPC_Solve_Time",
            "MPC_Time_Limit_Exceeded",
            "MPC_Fallback_Used",
            "MPC_Objective",
            "MPC_Iterations",
            "MPC_Optimality_Gap",
            "Command_Vector",
            "Command_Hex",
            "Command_Sent_Time",
            "Total_Active_Thrusters",
            "Thruster_Switches",
            "Total_MPC_Loop_Time",
            "Timing_Violation",
        ]

    def _get_real_headers(self) -> List[str]:
        """Get CSV headers for real hardware mode."""
        return [
            "Step",
            "MPC_Start_Time",
            "Control_Time",
            "Actual_Time_Interval",
            "CONTROL_DT",
            "Mission_Phase",
            "Waypoint_Number",
            "Telemetry_X_mm",
            "Telemetry_Z_mm",
            "Telemetry_Yaw_deg",
            "Current_X",
            "Current_Y",
            "Current_Yaw",
            "Current_VX",
            "Current_VY",
            "Current_Angular_Vel",
            "Target_X",
            "Target_Y",
            "Target_Yaw",
            "Target_VX",
            "Target_VY",
            "Target_Angular_Vel",
            "Error_X",
            "Error_Y",
            "Error_Yaw",
            "Error_VX",
            "Error_VY",
            "Error_Angular_Vel",
            "MPC_Computation_Time",
            "MPC_Status",
            "MPC_Solver",
            "MPC_Solver_Time_Limit",
            "MPC_Solve_Time",
            "MPC_Time_Limit_Exceeded",
            "MPC_Fallback_Used",
            "MPC_Objective",
            "MPC_Iterations",
            "MPC_Optimality_Gap",
            "Command_Vector",
            "Command_Hex",
            "Command_Sent_Time",
            "Total_Active_Thrusters",
            "Thruster_Switches",
            "Total_MPC_Loop_Time",
            "Timing_Violation",
        ]

    def _get_terminal_log_headers(self) -> List[str]:
        """Get CSV headers for terminal log."""
        return [
            "Time",
            "Status",
            "Stabilization_Time",
            "Position_Error_m",
            "Angle_Error_deg",
            "Active_Thrusters",
            "Solve_Time_s",
            "Next_Update_s",
        ]

    def _format_value(self, header: str, value: Any) -> str:
        """
        Format numeric values with appropriate precision based on column type.

        Args:
            header: Column name
            value: Value to format

        Returns:
            Formatted string value
        """
        # Handle empty/None values
        if value is None or value == "":
            return ""

        # Boolean values
        if isinstance(value, bool):
            return str(value)

        # String values (includes Command_Vector, Command_Hex, Status)
        if isinstance(value, str):
            return value

        # Numeric formatting based on column type
        try:
            num_value = float(value)

            # Integer columns
            if header in ["Step", "Waypoint_Number", "Total_Active_Thrusters", "Thruster_Switches", "MPC_Iterations"]:
                return str(int(num_value))

            # Time values - 4 decimals (0.1ms precision)
            elif header in [
                "MPC_Start_Time",
                "Control_Time",
                "Actual_Time_Interval",
                "Command_Sent_Time",
                "MPC_Computation_Time",
                "MPC_Solve_Time",
                "Total_MPC_Loop_Time",
            ]:
                return f"{num_value:.4f}"

            # Configuration values - 3 decimals
            elif header in ["CONTROL_DT", "MPC_Solver_Time_Limit"]:
                return f"{num_value:.3f}"

            # Telemetry positions (mm) - 2 decimals (0.01mm precision)
            elif header in ["Telemetry_X_mm", "Telemetry_Z_mm"]:
                return f"{num_value:.2f}"

            # Telemetry angle (degrees) - 2 decimals (0.01 degree precision)
            elif header in ["Telemetry_Yaw_deg"]:
                return f"{num_value:.2f}"

            # Position values (meters) - 5 decimals (0.01mm precision)
            elif header in [
                "Current_X",
                "Current_Y",
                "Target_X",
                "Target_Y",
                "Error_X",
                "Error_Y",
            ]:
                return f"{num_value:.5f}"

            # Angle values (radians) - 5 decimals (0.01 degree precision)
            elif header in ["Current_Yaw", "Target_Yaw", "Error_Yaw"]:
                return f"{num_value:.5f}"

            # Velocity values - 5 decimals (0.01mm/s precision)
            elif header in [
                "Current_VX",
                "Current_VY",
                "Current_Angular_Vel",
                "Target_VX",
                "Target_VY",
                "Target_Angular_Vel",
                "Error_VX",
                "Error_VY",
                "Error_Angular_Vel",
            ]:
                return f"{num_value:.5f}"

            # Objective value and optimality gap - 3 decimals
            elif header in ["MPC_Objective", "MPC_Optimality_Gap"]:
                return f"{num_value:.3f}"

            # Default: 6 decimals for unknown numeric columns
            else:
                return f"{num_value:.6f}"

        except (ValueError, TypeError):
            # If conversion fails, return as-is
            return str(value)

    def _format_terminal_value(self, header: str, value: Any) -> str:
        """
        Format terminal log values with appropriate precision.

        Args:
            header: Column name
            value: Value to format

        Returns:
            Formatted string value
        """
        if value is None or value == "":
            return ""

        if isinstance(value, str):
            return value

        try:
            num_value = float(value)

            # Time values - 4 decimals (0.1ms precision)
            if header in ["Time", "Stabilization_Time", "Solve_Time_s", "Next_Update_s"]:
                return f"{num_value:.4f}"

            # Position error - 5 decimals (0.01mm precision)
            elif header == "Position_Error_m":
                return f"{num_value:.5f}"

            # Angle error - 2 decimals (0.01 degree precision)
            elif header == "Angle_Error_deg":
                return f"{num_value:.2f}"

            else:
                return f"{num_value:.4f}"

        except (ValueError, TypeError):
            return str(value)

    def get_log_count(self) -> int:
        """Get the number of logged entries."""
        return len(self.detailed_log_data)

    def clear_logs(self) -> None:
        """Clear all logged data."""
        self.detailed_log_data = []
        self.terminal_log_data = []
        self.current_step = 0

    def get_summary_stats(self) -> Dict[str, Any]:
        """
        Calculate summary statistics from logged data.

        Returns:
            Dictionary containing summary statistics
        """
        if not self.detailed_log_data:
            return {}

        stats = {
            "total_steps": len(self.detailed_log_data),
            "mode": self.mode,
        }

        # Calculate MPC solve time statistics
        solve_times = []
        timing_violations = 0
        time_limit_exceeded = 0

        for entry in self.detailed_log_data:
            if "MPC_Solve_Time" in entry and entry["MPC_Solve_Time"] != "":
                try:
                    solve_time = float(entry["MPC_Solve_Time"])
                    solve_times.append(solve_time)
                except (ValueError, TypeError):
                    pass

            if entry.get("Timing_Violation") == "YES":
                timing_violations += 1

            if entry.get("MPC_Time_Limit_Exceeded") == "YES":
                time_limit_exceeded += 1

        if solve_times:
            import numpy as np

            stats["avg_solve_time"] = float(np.mean(solve_times))
            stats["max_solve_time"] = float(np.max(solve_times))
            stats["min_solve_time"] = float(np.min(solve_times))
            stats["std_solve_time"] = float(np.std(solve_times))

        stats["timing_violations"] = timing_violations
        stats["time_limit_exceeded"] = time_limit_exceeded

        return stats

    def print_summary(self) -> None:
        """Print summary of logged data."""
        stats = self.get_summary_stats()

        if not stats:
            print("No data logged")
            return

        print("\n" + "=" * 60)
        print(f"DATA LOGGER SUMMARY ({self.mode.upper()} MODE)")
        print("=" * 60)
        print(f"Total steps logged: {stats['total_steps']}")

        if "avg_solve_time" in stats:
            print("\nMPC Solve Time Statistics:")
            print(f"  Average: {stats['avg_solve_time'] * 1000:.2f} ms")
            print(f"  Min:     {stats['min_solve_time'] * 1000:.2f} ms")
            print(f"  Max:     {stats['max_solve_time'] * 1000:.2f} ms")
            print(f"  Std Dev: {stats['std_solve_time'] * 1000:.2f} ms")

        print("\nTiming Performance:")
        print(f"  Timing violations:     {stats['timing_violations']}")
        print(f"  Time limits exceeded:  {stats['time_limit_exceeded']}")

        print("=" * 60 + "\n")


def create_data_logger(mode: str = "simulation") -> DataLogger:
    """
    Factory function to create a data logger.

    Args:
        mode: Either "simulation" or "real"

    Returns:
        Configured DataLogger instance
    """
    return DataLogger(mode=mode)
