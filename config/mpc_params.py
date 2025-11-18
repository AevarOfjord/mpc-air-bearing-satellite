"""
MPC (Model Predictive Control) Configuration for Satellite Control

Comprehensive MPC tuning parameters for optimal control performance.
Includes prediction horizons, cost weights, constraints, and solver settings.

Configuration sections:
- Prediction and control horizons
- Cost function weights (position, velocity, angle, control effort)
- State and control constraints (velocity limits, thruster bounds)
- Solver settings (Gurobi parameters, time limits, tolerances)
- Adaptive control features (horizon scaling, weight adjustments)
- Performance tuning (warm starting, lazy updates)

Key features:
- Separated Q (state) and R (control) matrices
- Configurable prediction/control horizon lengths
- Adaptive horizon based on distance to target
- Solver timeout management for real-time performance
- Collision avoidance constraint parameters
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class MPCConfig:
    """
    Model Predictive Control tuning and limits.

    Attributes:
        prediction_horizon: Number of future steps to predict
        control_horizon: Number of control steps to optimize
        dt: Control timestep in seconds
        solver_time_limit: Maximum solver time in seconds
        solver_type: Optimization solver ('Gurobi', 'CVXPY', etc.)
        q_position: Position error cost weight
        q_velocity: Velocity error cost weight
        q_angle: Angle error cost weight
        q_ang_velocity: Angular velocity error cost weight
        r_thrust: Thrust magnitude penalty
        r_switch: Thruster switching penalty
        max_velocity: Maximum linear velocity in m/s
        max_angular_velocity: Maximum angular velocity in rad/s
        position_bounds: Workspace boundary in meters (±)
        damping_zone: Distance from target for velocity damping in meters
        velocity_threshold: Velocity threshold for adaptive weights in m/s
        max_velocity_weight: Maximum velocity weight when near target
        adaptive_horizons_enabled: Enable adaptive horizon adjustment
        solve_time_history_length: Number of solve times to track for averaging
    """

    # Horizons and timing
    prediction_horizon: int
    control_horizon: int
    dt: float
    solver_time_limit: float
    solver_type: str

    # Cost function weights
    q_position: float
    q_velocity: float
    q_angle: float
    q_ang_velocity: float
    r_thrust: float
    r_switch: float

    # Constraints
    max_velocity: float
    max_angular_velocity: float
    position_bounds: float

    # Adaptive control
    damping_zone: float
    velocity_threshold: float
    max_velocity_weight: float
    adaptive_horizons_enabled: bool
    solve_time_history_length: int


# DEFAULT MPC PARAMETERS
# ============================================================================

# Horizons and solver
MPC_PREDICTION_HORIZON = 12
MPC_CONTROL_HORIZON = 12
MPC_SOLVER_TIME_LIMIT = 0.05  # 50ms (CONTROL_DT - 0.01)
MPC_SOLVER_TYPE = "Gurobi"
VERBOSE_MPC = False
ADAPTIVE_HORIZONS_ENABLED = True
SOLVE_TIME_HISTORY_LENGTH = 10

# Cost weights
Q_POSITION = 1000.0
Q_VELOCITY = 1750.0
Q_ANGLE = 1000.0
Q_ANGULAR_VELOCITY = 1500.0
R_THRUST = 1.0
R_SWITCH = 0.0

# Fallback Controller Thresholds (when MPC solver fails)
FALLBACK_POSITION_ERROR_THRESHOLD = 0.1  # meters
FALLBACK_VELOCITY_ERROR_THRESHOLD = 0.05  # m/s
FALLBACK_ANGLE_ERROR_THRESHOLD = 0.1  # radians

# Constraints
MAX_VELOCITY = 0.25  # m/s
MAX_ANGULAR_VELOCITY = np.pi / 2  # rad/s
POSITION_BOUNDS = 3.0  # meters
ANGLE_BOUNDS = 2 * np.pi  # radians

# Adaptive control parameters
DAMPING_ZONE = 0.25  # meters
VELOCITY_THRESHOLD = 0.03  # m/s
MAX_VELOCITY_WEIGHT = 1000.0

# Performance tolerances
POSITION_TOLERANCE = 0.05  # meters
ANGLE_TOLERANCE = np.deg2rad(3)  # radians
VELOCITY_TOLERANCE = 0.05  # m/s
ANGULAR_VELOCITY_TOLERANCE = np.deg2rad(3)  # rad/s


def get_mpc_params() -> MPCConfig:
    """
    Get default MPC configuration.

    Returns:
        MPCConfig with default MPC parameters
    """
    return MPCConfig(
        prediction_horizon=MPC_PREDICTION_HORIZON,
        control_horizon=MPC_CONTROL_HORIZON,
        dt=0.06,  # Will be overridden by CONTROL_DT from timing
        solver_time_limit=MPC_SOLVER_TIME_LIMIT,
        solver_type=MPC_SOLVER_TYPE,
        q_position=Q_POSITION,
        q_velocity=Q_VELOCITY,
        q_angle=Q_ANGLE,
        q_ang_velocity=Q_ANGULAR_VELOCITY,
        r_thrust=R_THRUST,
        r_switch=R_SWITCH,
        max_velocity=MAX_VELOCITY,
        max_angular_velocity=MAX_ANGULAR_VELOCITY,
        position_bounds=POSITION_BOUNDS,
        damping_zone=DAMPING_ZONE,
        velocity_threshold=VELOCITY_THRESHOLD,
        max_velocity_weight=MAX_VELOCITY_WEIGHT,
        adaptive_horizons_enabled=ADAPTIVE_HORIZONS_ENABLED,
        solve_time_history_length=SOLVE_TIME_HISTORY_LENGTH,
    )


def validate_mpc_params(config: MPCConfig, control_dt: float) -> bool:
    """
    Validate MPC parameters for consistency.

    Args:
        config: MPCConfig to validate
        control_dt: Control interval in seconds (from TimingConfig)

    Returns:
        True if valid, False otherwise
    """
    issues = []

    # Horizon validation
    if config.control_horizon > config.prediction_horizon:
        issues.append(
            f"Control horizon ({config.control_horizon}) > "
            f"prediction horizon ({config.prediction_horizon})"
        )

    if config.prediction_horizon <= 0:
        issues.append(
            f"Prediction horizon must be positive: {config.prediction_horizon}"
        )

    if config.control_horizon <= 0:
        issues.append(f"Control horizon must be positive: {config.control_horizon}")

    # Timing validation
    if config.solver_time_limit >= control_dt:
        issues.append(
            f"Solver time limit ({config.solver_time_limit}s) >= "
            f"control interval ({control_dt}s)"
        )

    # Constraint validation
    if config.max_velocity <= 0:
        issues.append(f"Max velocity must be positive: {config.max_velocity}")

    if config.max_angular_velocity <= 0:
        issues.append(
            f"Max angular velocity must be positive: {config.max_angular_velocity}"
        )

    if config.position_bounds <= 0:
        issues.append(f"Position bounds must be positive: {config.position_bounds}")

    # Cost weight validation (should be non-negative)
    if config.q_position < 0:
        issues.append(f"Position weight must be non-negative: {config.q_position}")

    if config.r_thrust < 0:
        issues.append(f"Thrust penalty must be non-negative: {config.r_thrust}")

    # Report validation results
    if issues:
        print("MPC parameter validation failed:")
        for issue in issues:
            print(f"  - {issue}")
        return False

    return True


def print_mpc_config(config: MPCConfig) -> None:
    """
    Print MPC configuration for debugging.

    Args:
        config: MPCConfig to print
    """
    print("\nMPC CONTROLLER PARAMETERS:")
    print(f"   Prediction horizon:     {config.prediction_horizon} steps")
    print(f"   Control horizon:        {config.control_horizon} steps")
    print(f"   Solver time limit:      {config.solver_time_limit:.3f} s")
    print(f"   Solver type:            {config.solver_type}")
    print(f"   Adaptive horizons:      {config.adaptive_horizons_enabled}")

    print("\nCOST FUNCTION WEIGHTS:")
    print(f"   Position weight (Q):    {config.q_position:.1f}")
    print(f"   Velocity weight (Q):    {config.q_velocity:.1f}")
    print(f"   Angle weight (Q):       {config.q_angle:.1f}")
    print(f"   Angular vel weight (Q): {config.q_ang_velocity:.1f}")
    print(f"   Thrust penalty (R):     {config.r_thrust:.3f}")
    print(f"   Switch penalty (R):     {config.r_switch:.3f}")

    print("\nSYSTEM CONSTRAINTS:")
    print(f"   Max velocity:           {config.max_velocity:.2f} m/s")
    print(f"   Max angular velocity:   {config.max_angular_velocity:.2f} rad/s")
    print(f"   Position bounds:        ±{config.position_bounds:.1f} m")
    print()
