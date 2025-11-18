#!/usr/bin/env python3
"""
Thruster Force Measurement and Calibration Tool

Automated thruster testing system for force measurement and characterization.
Uses precision load cell and synchronized serial communication for accurate calibration data.

Test procedure:
- Automated sequential testing of each thruster
- Multiple pulse tests per thruster for statistical analysis
- Real-time force measurement at high sample rate
- Configurable pulse duration and pause intervals
- CSV export with timestamp and force profiles

Hardware requirements:
- Precision load cell with serial output (e.g., HX711-based)
- Arduino or microcontroller for solenoid valve control
- Serial communication at 115200 baud

Data collection:
- Timestamp-synchronized force measurements
- Peak force, average force, impulse calculations
- Statistical analysis (mean, std dev, confidence intervals)
- CSV export for analysis and SET_EFFECTS matrix generation

Key features:
- MPC timing integration from config
- Configurable test parameters
- Real-time progress monitoring
- Automatic data directory management
- Thread-safe serial communication
"""
import csv, pathlib, serial, threading, queue, time, sys, re
import numpy as np
from typing import Tuple, Any
from config import SatelliteConfig

# ---------- USER CONFIG -------------------------------------------------
SCALE_PORT = "/dev/cu.usbmodem101"      # scale data
CMD_PORT   = "/dev/cu.usbmodem1101"     # solenoid commands
SCALE_BAUD = 115200
CMD_BAUD   = 115200
ACT_MS     = int(10 * 1000)  # Fixed action time for testing (10 seconds)
PAUSE_BETW = 5.0                        # pause between pulses (s)
LOG_DT     = SatelliteConfig.SIMULATION_DT  # Use Config simulation timestep
NUM_TESTS_PER_THRUSTER = 15             # Number of pulses per thruster
# -----------------------------------------------------------------------

# --- Target directory --------------------------------------------------
DATA_DIR = pathlib.Path("Data/Thruster_Data")
DATA_DIR.mkdir(parents=True, exist_ok=True)

def make_writer(solenoid_num: int) -> Tuple[Any, Any]:  # type: ignore[type-arg]
    """Return (file_handle, csv.writer) ready to log."""
    stamp = time.strftime("%Y%m%d_%H%M%S")
    file_path = DATA_DIR / f"MPC_Thruster({solenoid_num})_{stamp}.csv"
    f = file_path.open("w", newline="")
    w = csv.writer(f)
    w.writerow(("t_s", "weight_g", "bitmask", "pulse_number", "phase"))
    return f, w

def rx_scale(ser, q, stop):
    """Read scale data and queue it for the main thread."""
    while not stop.is_set():
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if line:
                match = re.search(r'-?\d+\.?\d*', line)
                if match:
                    value = float(match.group())
                    q.put(value)
        except (ValueError, AttributeError, UnicodeDecodeError):
            pass
        except Exception:
            time.sleep(0.01)

def analyze_pulse_data(data, pulse_start_time, pulse_duration_s):
    """Analyze a single pulse to extract thrust characteristics using steady-state analysis."""
    times = np.array([row[0] for row in data])
    weights = np.array([row[1] for row in data if row[1] != ""])

    if len(weights) < 10:
        return None

    baseline_mask = times < pulse_start_time - 0.1
    if np.sum(baseline_mask) > 5:
        baseline_weight = np.median(weights[baseline_mask])
    else:
        baseline_weight = weights[0]

    # Apply steady-state analysis: ignore first 3.0s and last 1.0s of firing
    steady_state_start_offset = 3.0  # Ignore first 3 seconds
    steady_state_end_offset = 1.0    # Ignore last 1 second

    # Initialize variables
    steady_start_time = pulse_start_time
    steady_end_time = pulse_start_time

    if pulse_duration_s >= (steady_state_start_offset + steady_state_end_offset + 0.5):  # Need at least 2.0s total
        # Define steady-state window
        steady_start_time = pulse_start_time + steady_state_start_offset
        steady_end_time = pulse_start_time + pulse_duration_s - steady_state_end_offset

        # Find data points within the steady-state window
        steady_mask = (times >= steady_start_time) & (times <= steady_end_time)
        steady_weights = weights[steady_mask]

        if len(steady_weights) > 5:  # Need sufficient steady-state data
            # Calculate thrust using only steady-state data
            steady_thrust_values = steady_weights - baseline_weight
            positive_thrust = steady_thrust_values[steady_thrust_values > 0]

            if len(positive_thrust) > 0:
                steady_thrust = np.median(positive_thrust)
                thrust_std = np.std(positive_thrust)
            else:
                steady_thrust = 0
                thrust_std = 0
        else:
            steady_thrust = 0
            thrust_std = 0
    else:
        steady_thrust = 0
        thrust_std = 0

    pulse_mask = (times >= pulse_start_time) & (times <= pulse_start_time + pulse_duration_s)
    pulse_weights = weights[pulse_mask]
    peak_thrust = np.max(pulse_weights) - baseline_weight if len(pulse_weights) > 0 else 0

    return {'baseline_weight': baseline_weight,
        'steady_thrust': steady_thrust,
        'thrust_std': thrust_std,
        'peak_thrust': peak_thrust,
        'steady_state_duration': steady_end_time - steady_start_time if pulse_duration_s >= 2.0 else 0
    }

def main():
    try:
        sol = int(input("Solenoid (1-8): "))
        if not 1 <= sol <= 8:
            raise ValueError

        auto_test = input("Test all thrusters automatically? (y/n): ").lower() == 'y'
        if auto_test:
            thrusters_to_test = list(range(1, 9))
        else:
            thrusters_to_test = [sol]

    except ValueError:
        print("Invalid input.")
        sys.exit(1)

    try:
        scale_ser = serial.Serial(SCALE_PORT, SCALE_BAUD, timeout=0.05)
        cmd_ser = serial.Serial(CMD_PORT, CMD_BAUD, timeout=0.05)
        print(f" Serial ports opened successfully")
    except serial.SerialException as e:
        print("Serial error:", e)
        sys.exit(1)

    # Results storage
    thruster_results = {}

    for thruster_num in thrusters_to_test:
        print(f"\n{'='*50}")
        print(f"TESTING THRUSTER {thruster_num}")
        print(f"{'='*50}")

        bitmask = 1 << (thruster_num - 1)
        csv_f, writer = make_writer(thruster_num)

        q_w = queue.Queue()
        stop = threading.Event()
        threading.Thread(target=rx_scale, args=(scale_ser, q_w, stop), daemon=True).start()

        latest = None
        pulse_data = []

        def get_w():
            nonlocal latest
            while not q_w.empty():
                latest = q_w.get()
            return latest

        t0 = time.perf_counter()
        next_log = 0.0
        now = lambda: time.perf_counter() - t0

        pulse_results = []

        try:
            # Initial stabilization
            print("Stabilizing baseline (3 seconds)...")
            while now() < 3.0:
                if now() >= next_log:
                    w = get_w()
                    writer.writerow((f"{next_log:.3f}", f"{w:.2f}" if w is not None else "", 0, 0, "baseline"))
                    pulse_data.append((next_log, w if w is not None else 0, 0, 0, "baseline"))
                    next_log += LOG_DT

            for pulse_num in range(NUM_TESTS_PER_THRUSTER):
                print(f"Pulse {pulse_num + 1}/{NUM_TESTS_PER_THRUSTER}")

                current_pulse_data = []

                pre_pulse_start = now()
                while now() < pre_pulse_start + 0.5:
                    if now() >= next_log:
                        w = get_w()
                        writer.writerow((f"{next_log:.3f}", f"{w:.2f}" if w is not None else "", 0, pulse_num + 1, "pre_pulse"))
                        pulse_data.append((next_log, w if w is not None else 0, 0, pulse_num + 1, "pre_pulse"))
                        current_pulse_data.append((next_log, w if w is not None else 0, 0, pulse_num + 1, "pre_pulse"))
                        next_log += LOG_DT

                # Fire thruster
                pulse_start = now()
                cmd_ser.write(f"CMD:{bitmask}:{ACT_MS}\n".encode())
                pulse_end = pulse_start + ACT_MS / 1000

                while now() < pulse_end:
                    if now() >= next_log:
                        w = get_w()
                        writer.writerow((f"{next_log:.3f}", f"{w:.2f}" if w is not None else "", bitmask, pulse_num + 1, "firing"))
                        pulse_data.append((next_log, w if w is not None else 0, bitmask, pulse_num + 1, "firing"))
                        current_pulse_data.append((next_log, w if w is not None else 0, bitmask, pulse_num + 1, "firing"))
                        next_log += LOG_DT

                # Stop thruster
                cmd_ser.write(b"CMD:0:0\n")

                while now() < pulse_end + 1.0:
                    if now() >= next_log:
                        w = get_w()
                        writer.writerow((f"{next_log:.3f}", f"{w:.2f}" if w is not None else "", 0, pulse_num + 1, "post_pulse"))
                        pulse_data.append((next_log, w if w is not None else 0, 0, pulse_num + 1, "post_pulse"))
                        current_pulse_data.append((next_log, w if w is not None else 0, 0, pulse_num + 1, "post_pulse"))
                        next_log += LOG_DT

                # Analyze this pulse using only current pulse data
                result = analyze_pulse_data(current_pulse_data, pulse_start, ACT_MS / 1000)
                if result:
                    pulse_results.append(result)
                    print(f"  Thrust: {result['steady_thrust']:.3f} ± {result['thrust_std']:.3f} g")

                # Pause between pulses
                if pulse_num < NUM_TESTS_PER_THRUSTER - 1:
                    pause_end = now() + PAUSE_BETW
                    while now() < pause_end:
                        if now() >= next_log:
                            w = get_w()
                            writer.writerow((f"{next_log:.3f}", f"{w:.2f}" if w is not None else "", 0, pulse_num + 1, "pause"))
                            pulse_data.append((next_log, w if w is not None else 0, 0, pulse_num + 1, "pause"))
                            next_log += LOG_DT

            print("Test complete")

        except KeyboardInterrupt:
            print("\nAborted by user")
            cmd_ser.write(b"CMD:0:0\n")

        finally:
            stop.set()
            time.sleep(0.2)
            csv_f.close()
            print(f"CSV saved → {csv_f.name}")

        # Statistical analysis
        if pulse_results:
            thrusts = [r['steady_thrust'] for r in pulse_results]
            mean_thrust = np.mean(thrusts)
            std_thrust = np.std(thrusts)
            median_thrust = np.median(thrusts)

            print(f"\nSTATISTICAL ANALYSIS - THRUSTER {thruster_num}")
            print(f"{'='*40}")
            print(f"Number of pulses: {len(thrusts)}")
            print(f"Mean thrust: {mean_thrust:.3f} g")
            print(f"Median thrust: {median_thrust:.3f} g")
            print(f"Standard deviation: {std_thrust:.3f} g")
            print(f"Coefficient of variation: {(std_thrust/mean_thrust) * 100:.1f}%")
            print(f"Thrust range: {np.min(thrusts):.3f} to {np.max(thrusts):.3f} g")

            thrust_N = median_thrust * 9.81 / 1000  # g to N
            print(f"Recommended thrust value: {thrust_N:.6f} N")

            thruster_results[thruster_num] = {'thrust_N': thrust_N,
                'thrust_g': median_thrust,
                'std_g': std_thrust,
                'cv_percent': (std_thrust/mean_thrust) * 100,
                'num_samples': len(thrusts)
            }

    # Final summary
    if len(thruster_results) > 1:
        print(f"\n{'='*60}")
        print("FINAL THRUSTER CHARACTERIZATION RESULTS")
        print(f"{'='*60}")

        for thruster_num, results in thruster_results.items():
            print(f"Thruster {thruster_num}: {results['thrust_N']:.6f} N ({results['thrust_g']:.3f} g) ± {results['cv_percent']:.1f}%")

        # Generate SET_EFFECTS recommendations
        print(f"\nSET_EFFECTS matrix recommendations:")
        print("# Update your Model.py with these values:")
        for thruster_num, results in thruster_results.items():
            print(f"# Thruster {thruster_num}: {results['thrust_N']:.6f} N")

    scale_ser.close()
    cmd_ser.close()

if __name__ == "__main__":
    main()
