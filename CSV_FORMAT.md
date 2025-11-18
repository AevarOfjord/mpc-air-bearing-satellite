# CSV Data Format Reference

Both simulation and real hardware produce identical CSV format for easy comparison.

## Output Files

Each mission run generates two CSV files:

1. **Detailed Data Log**: `simulation_data.csv` or `real_test_data.csv`
   - Full control loop data with all state variables, MPC info, and commands
   - 45 columns per row (see below)
   - **Identical format for both simulation and real hardware**

2. **Terminal Log**: `simulation_terminal_log.csv` or `real_terminal_log.csv`
   - Human-readable status messages that appear in terminal
   - 8 columns per row (see Terminal Log Format section)
  
# Detailed Data Log

## Column Definitions (45 columns total)

### Timing & Control (7 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Step | - | Control iteration number (0, 1, 2, ...) |
| MPC_Start_Time | seconds | Time when MPC computation started |
| Control_Time | seconds | Current control loop time |
| Actual_Time_Interval | seconds | Actual time elapsed since last control update |
| CONTROL_DT | seconds | Configured control interval (nominal) |
| Mission_Phase | - | Current phase: APPROACHING/STABILIZING (waypoint mode) or POSITIONING/PATH_STABILIZATION/TRACKING/RETURNING/STABILIZING (shape following) |
| Waypoint_Number | - | Current waypoint index (0 if not in waypoint mode) |

### Telemetry (3 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Telemetry_X_mm | millimeters | Raw X position from motion capture / simulation |
| Telemetry_Z_mm | millimeters | Raw Z position from motion capture / simulation (Y-axis in world frame) |
| Telemetry_Yaw_deg | degrees | Raw yaw angle from motion capture / simulation |

**Note:** In simulation, telemetry values mirror the current state (no measurement noise).
In real hardware, these are raw OptiTrack measurements.

### Current State (6 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Current_X | meters | X position in world frame (SI units) |
| Current_Y | meters | Y position in world frame (SI units) |
| Current_Yaw | radians | Yaw angle in world frame (SI units) |
| Current_VX | m/s | Velocity in X direction |
| Current_VY | m/s | Velocity in Y direction |
| Current_Angular_Vel | rad/s | Angular velocity |

### Target State (6 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Target_X | meters | Target X position |
| Target_Y | meters | Target Y position |
| Target_Yaw | radians | Target yaw angle |
| Target_VX | m/s | Target velocity in X direction |
| Target_VY | m/s | Target velocity in Y direction |
| Target_Angular_Vel | rad/s | Target angular velocity |

### Tracking Error (6 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Error_X | meters | Position error in X: Target_X - Current_X |
| Error_Y | meters | Position error in Y: Target_Y - Current_Y |
| Error_Yaw | radians | Yaw error (shortest angular distance) |
| Error_VX | m/s | Velocity error in X: Target_VX - Current_VX |
| Error_VY | m/s | Velocity error in Y: Target_VY - Current_VY |
| Error_Angular_Vel | rad/s | Angular velocity error |

### MPC Performance (10 columns)
| Column | Unit | Description |
|--------|------|-------------|
| MPC_Computation_Time | seconds | Total time for MPC optimization |
| MPC_Status | - | Solver status (OPTIMAL, SUBOPTIMAL, TIME_LIMIT, etc.) |
| MPC_Solver | - | Solver type (typically "Gurobi") |
| MPC_Solver_Time_Limit | seconds | Configured solver time limit |
| MPC_Solve_Time | seconds | Time spent in solver (subset of computation time) |
| MPC_Time_Limit_Exceeded | boolean | True if solver hit time limit |
| MPC_Fallback_Used | boolean | True if fallback controller was used |
| MPC_Objective | - | Optimization objective value achieved (3 decimals) |
| MPC_Iterations | - | Number of solver iterations (integer) |
| MPC_Optimality_Gap | - | MIP optimality gap (3 decimals) |

### Command Output (5 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Command_Vector | - | Thruster commands as array: [T1, T2, ..., T8] (0 or 1) |
| Command_Hex | hex | Command as hex string for hardware transmission |
| Command_Sent_Time | seconds | Time when command was sent to hardware/simulation |
| Total_Active_Thrusters | - | Count of thrusters firing this timestep (integer) |
| Thruster_Switches | - | Number of thrusters that changed state (integer) |

### Loop Performance (2 columns)
| Column | Unit | Description |
|--------|------|-------------|
| Total_MPC_Loop_Time | seconds | Total time from MPC start to command sent |
| Timing_Violation | boolean | True if MPC took longer than CONTROL_DT - 20ms |

## Unit Consistency

**SI Units (Internal):**
- Position: meters
- Angle: radians
- Velocity: m/s
- Angular velocity: rad/s
- Time: seconds

**Telemetry Units (Raw from sensors):**
- Position: millimeters (matches OptiTrack output)
- Angle: degrees (matches OptiTrack output)

## Mission Phases

### Waypoint Navigation Phases
1. **APPROACHING** - Moving toward waypoint target
2. **STABILIZING** - Holding at waypoint (3s for intermediate, 10s for final waypoint)

### Shape Following Phases

**Without Return Position:**
1. **POSITIONING** - Moving to closest point on path
2. **PATH_STABILIZATION** - Stabilizing at start waypoint before tracking begins (5s)
3. **TRACKING** - Following moving target along path
4. **STABILIZING** - Holding at final path position (15s)

**With Return Position:**
1. **POSITIONING** - Moving to closest point on path
2. **PATH_STABILIZATION** - Stabilizing at start waypoint before tracking begins (5s)
3. **TRACKING** - Following moving target along path
4. **PATH_STABILIZATION** - Stabilizing at final path waypoint (5s)
5. **RETURNING** - Moving to return position (not on path)
6. **STABILIZING** - Holding at return position (15s)

**Note:**
- All phases now display elapsed time in the format `PHASE_NAME (t=X.Xs)`
- PATH_STABILIZATION appears twice when a return position is configured - once at the start waypoint before the target begins moving, and once at the final waypoint after the path is complete
- The timer resets to 0 for each new phase (including the second PATH_STABILIZATION)

## Data Comparison Notes

When comparing simulation to real hardware:
1. **Telemetry columns** should be similar (simulation is noiseless, real has measurement noise)
2. **Current state** derived from telemetry will show filtering effects in real hardware
3. **MPC timing** will be faster in simulation (no hardware communication overhead)
4. **Command execution** is instantaneous in simulation but has valve delays in real hardware

---

# Terminal Log

The terminal log CSV captures all status messages displayed during mission execution.

### Columns (8 total)

| Column | Unit | Description |
|--------|------|-------------|
| Time | seconds | Control time when message was printed |
| Status | - | Mission status: APPROACHING/STABILIZING (waypoint) or POSITIONING/PATH_STABILIZATION/TRACKING/RETURNING/STABILIZING (shape following) |
| Stabilization_Time | seconds | Time spent stabilizing (empty if approaching) |
| Position_Error_m | meters | Distance error to target |
| Angle_Error_deg | degrees | Angular error to target orientation |
| Active_Thrusters | - | List of active thruster IDs [1-8] |
| Solve_Time_s | seconds | MPC computation time for this step |
| Next_Update_s | seconds | Next scheduled control update (simulation only, empty for real) |

### Example Terminal Log

**Waypoint Navigation:**
```csv
Time,Status,Stabilization_Time,Position_Error_m,Angle_Error_deg,Active_Thrusters,Solve_Time_s,Next_Update_s
238.3,APPROACHING,,0.015,1.2,[8],0.009,238.38
238.4,APPROACHING,,0.012,0.9,[4 8],0.007,238.44
238.5,STABILIZING (t=0.1s),0.1,0.004,0.5,[],0.006,238.56
238.6,STABILIZING (t=0.2s),0.2,0.003,0.4,[],0.005,238.62
```

**Shape Following (without return):**
```csv
Time,Status,Stabilization_Time,Position_Error_m,Angle_Error_deg,Active_Thrusters,Solve_Time_s,Next_Update_s
5.2,POSITIONING (t=0.1s),,0.123,2.3,[1 2 7 8],0.008,5.26
6.5,POSITIONING (t=1.4s),,0.015,0.8,[4 8],0.007,6.56
7.0,PATH_STABILIZATION (t=0.5s),0.5,0.008,0.5,[],0.006,7.06
9.5,PATH_STABILIZATION (t=2.8s),2.8,0.004,0.3,[],0.006,9.56
10.0,TRACKING (t=0.1s),,0.012,0.6,[1 5],0.009,10.06
15.3,TRACKING (t=5.4s),,0.010,0.7,[2 3],0.008,15.36
18.0,STABILIZING (t=0.2s),0.2,0.003,0.4,[],0.006,18.06
```

**Shape Following (with return position):**
```csv
Time,Status,Stabilization_Time,Position_Error_m,Angle_Error_deg,Active_Thrusters,Solve_Time_s,Next_Update_s
5.2,POSITIONING (t=0.1s),,0.123,2.3,[1 2 7 8],0.008,5.26
6.5,POSITIONING (t=1.4s),,0.015,0.8,[4 8],0.007,6.56
7.0,PATH_STABILIZATION (t=0.5s),0.5,0.008,0.5,[],0.006,7.06
9.5,PATH_STABILIZATION (t=2.8s),2.8,0.004,0.3,[],0.006,9.56
10.0,TRACKING (t=0.1s),,0.012,0.6,[1 5],0.009,10.06
15.3,TRACKING (t=5.4s),,0.010,0.7,[2 3],0.008,15.36
16.0,PATH_STABILIZATION (t=0.2s),0.2,0.003,0.4,[],0.006,16.06
18.5,PATH_STABILIZATION (t=2.7s),2.7,0.002,0.2,[],0.006,18.56
19.0,RETURNING (t=0.1s),,0.145,1.8,[1 7 8],0.007,19.06
24.2,RETURNING (t=5.3s),,0.052,0.6,[4],0.008,24.26
27.0,STABILIZING (t=0.1s),0.1,0.004,0.3,[],0.006,27.06
```

### Use Cases

- **Quick mission review**: Scan status progression without detailed data
- **Performance summary**: Track solve times and error convergence
- **Thruster activity**: Identify firing patterns during different phases
- **Timeline reconstruction**: Understand mission flow and phase transitions
