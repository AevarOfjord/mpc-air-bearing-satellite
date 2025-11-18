"""
Physical Parameters for Satellite Control System

Complete physical model parameters for satellite dynamics and thruster configuration.
Includes mass properties, thruster geometry, and realistic physics effects.

Configuration sections:
- Mass Properties: Total mass, moment of inertia, center of mass offset
- Thruster Configuration: Eight-thruster layout with positions and directions
- Thruster Forces: Individual force calibration per thruster
- Realistic Physics: Damping, friction, sensor noise
- Air Bearing System: Three-point support configuration

Thruster layout:
- Eight thrusters arranged around satellite body
- Individual position and direction vectors
- Configurable force magnitude per thruster
- Support for force calibration and testing

Key features:
- Individual thruster force calibration
- Realistic damping and friction modeling
- Sensor noise simulation for testing
- Center of mass calculation from air bearing
- Integration with testing_environment physics
"""

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass
class PhysicsConfig:
    """
    Physical properties and optional realism toggles.

    Attributes:
        total_mass: Total satellite mass in kg
        moment_of_inertia: Rotational inertia in kg·m²
        satellite_size: Characteristic dimension in meters
        com_offset: Center of mass offset [x, y] in meters
        thruster_positions: Dict mapping thruster ID (1-8) to (x, y) position in meters
        thruster_directions: Dict mapping thruster ID to unit direction vector
        thruster_forces: Dict mapping thruster ID to force magnitude in Newtons
        use_realistic_physics: Enable realistic physics modeling
        linear_damping_coeff: Linear drag coefficient in N/(m/s)
        rotational_damping_coeff: Rotational drag coefficient in N*m/(rad/s)
        position_noise_std: Position measurement noise std dev in meters
        velocity_noise_std: Velocity estimation noise std dev in m/s
        angle_noise_std: Orientation noise std dev in radians
        angular_velocity_noise_std: Angular velocity noise std dev in rad/s
        thruster_valve_delay: Solenoid valve opening delay in seconds
        thruster_rampup_time: Time for thrust to reach full force in seconds
        thruster_force_noise_std: Fractional thrust force variation (std dev)
        enable_random_disturbances: Enable random environmental disturbances
        disturbance_force_std: Random disturbance force std dev in Newtons
        disturbance_torque_std: Random disturbance torque std dev in N*m
    """

    # Core physical properties
    total_mass: float
    moment_of_inertia: float
    satellite_size: float
    com_offset: np.ndarray

    # Thruster configuration
    thruster_positions: Dict[int, Tuple[float, float]]
    thruster_directions: Dict[int, np.ndarray]
    thruster_forces: Dict[int, float]

    # Realistic physics modeling
    use_realistic_physics: bool = True
    linear_damping_coeff: float = 1.8
    rotational_damping_coeff: float = 0.3

    # Sensor noise
    position_noise_std: float = 0.000
    velocity_noise_std: float = 0.000
    angle_noise_std: float = 0.0
    angular_velocity_noise_std: float = 0.0

    # Actuator dynamics
    thruster_valve_delay: float = 0.04
    thruster_rampup_time: float = 0.01
    thruster_force_noise_std: float = 0.00

    # Environmental disturbances
    enable_random_disturbances: bool = True
    disturbance_force_std: float = 0.4
    disturbance_torque_std: float = 0.1


# DEFAULT PHYSICAL PARAMETERS
# ============================================================================

# Air bearing weight distribution
AIR_BEARING_1_WEIGHT = 7.614  # kg
AIR_BEARING_1_RADIAL = 0.214  # m
AIR_BEARING_1_ANGULAR = 0.0  # degrees

AIR_BEARING_2_WEIGHT = 7.724  # kg
AIR_BEARING_2_RADIAL = 0.193  # m
AIR_BEARING_2_ANGULAR = -120.0  # degrees

AIR_BEARING_3_WEIGHT = 7.752  # kg
AIR_BEARING_3_RADIAL = 0.195  # m
AIR_BEARING_3_ANGULAR = 120.0  # degrees

TOTAL_MASS = AIR_BEARING_1_WEIGHT + AIR_BEARING_2_WEIGHT + AIR_BEARING_3_WEIGHT
SATELLITE_SIZE = 0.29  # m

MOMENT_OF_INERTIA = (1 / 6) * TOTAL_MASS * SATELLITE_SIZE**2

# Thruster configuration
THRUSTER_POSITIONS = {
    1: (0.145, 0.06),  # Right-top
    2: (0.145, -0.06),  # Right-bottom
    3: (0.06, -0.145),  # Bottom-right
    4: (-0.06, -0.145),  # Bottom-left
    5: (-0.145, -0.06),  # Left-bottom
    6: (-0.145, 0.06),  # Left-top
    7: (-0.06, 0.145),  # Top-left
    8: (0.06, 0.145),  # Top-right
}

THRUSTER_DIRECTIONS = {
    1: np.array([-1, 0]),  # Left
    2: np.array([-1, 0]),  # Left
    3: np.array([0, 1]),  # Up
    4: np.array([0, 1]),  # Up
    5: np.array([1, 0]),  # Right
    6: np.array([1, 0]),  # Right
    7: np.array([0, -1]),  # Down
    8: np.array([0, -1]),  # Down
}

THRUSTER_FORCES = {
    1: 0.441450,  # N - Measured thruster forces
    2: 0.430659,
    3: 0.427716,
    4: 0.438017,
    5: 0.468918,
    6: 0.446846,
    7: 0.466956,
    8: 0.484124,
}

GRAVITY_M_S2 = 9.81  # m/s²


def calculate_com_offset() -> np.ndarray:
    """
    Calculate center of mass offset from air bearing parameters.

    Returns:
        2D numpy array [x, y] representing COM offset in meters
    """
    radii = np.array([AIR_BEARING_1_RADIAL, AIR_BEARING_2_RADIAL, AIR_BEARING_3_RADIAL])

    angles_deg = np.array(
        [AIR_BEARING_1_ANGULAR, AIR_BEARING_2_ANGULAR, AIR_BEARING_3_ANGULAR]
    )

    masses = np.array(
        [AIR_BEARING_1_WEIGHT, AIR_BEARING_2_WEIGHT, AIR_BEARING_3_WEIGHT]
    )

    angles_rad = np.deg2rad(angles_deg)
    x_positions = radii * np.cos(angles_rad)
    y_positions = radii * np.sin(angles_rad)

    com_x = np.sum(masses * x_positions) / TOTAL_MASS
    com_y = np.sum(masses * y_positions) / TOTAL_MASS

    return np.array([com_x, com_y])


COM_OFFSET = calculate_com_offset()


def get_physics_params() -> PhysicsConfig:
    """
    Get default physics configuration.

    Returns:
        PhysicsConfig with default physical parameters
    """
    return PhysicsConfig(
        total_mass=TOTAL_MASS,
        moment_of_inertia=MOMENT_OF_INERTIA,
        satellite_size=SATELLITE_SIZE,
        com_offset=COM_OFFSET.copy(),
        thruster_positions=THRUSTER_POSITIONS.copy(),
        thruster_directions={k: v.copy() for k, v in THRUSTER_DIRECTIONS.items()},
        thruster_forces=THRUSTER_FORCES.copy(),
        use_realistic_physics=True,
        linear_damping_coeff=1.8,
        rotational_damping_coeff=0.3,
        position_noise_std=0.000,
        velocity_noise_std=0.000,
        angle_noise_std=np.deg2rad(0.0),
        angular_velocity_noise_std=np.deg2rad(0.0),
        thruster_valve_delay=0.04,
        thruster_rampup_time=0.01,
        thruster_force_noise_std=0.00,
        enable_random_disturbances=True,
        disturbance_force_std=0.4,
        disturbance_torque_std=0.1,
    )


def set_thruster_force(thruster_id: int, force: float) -> None:
    """
    Set individual thruster force for calibration.

    Args:
        thruster_id: Thruster ID (1-8)
        force: Force magnitude in Newtons

    Raises:
        ValueError: If thruster_id invalid or force non-positive
    """
    if thruster_id not in range(1, 9):
        raise ValueError(f"Thruster ID must be 1-8, got {thruster_id}")
    if force <= 0:
        raise ValueError(f"Force must be positive, got {force}")

    THRUSTER_FORCES[thruster_id] = force
    print(f"Thruster {thruster_id} force set to {force:.3f} N")


def set_all_thruster_forces(force: float) -> None:
    """
    Set all thruster forces to the same value.

    Args:
        force: Force magnitude in Newtons for all thrusters

    Raises:
        ValueError: If force is non-positive
    """
    if force <= 0:
        raise ValueError(f"Force must be positive, got {force}")

    for thruster_id in range(1, 9):
        THRUSTER_FORCES[thruster_id] = force
    print(f"All thruster forces set to {force:.3f} N")


def get_thruster_force(thruster_id: int) -> float:
    """
    Get individual thruster force.

    Args:
        thruster_id: Thruster ID (1-8)

    Returns:
        Force magnitude in Newtons

    Raises:
        ValueError: If thruster_id is invalid
    """
    if thruster_id not in range(1, 9):
        raise ValueError(f"Thruster ID must be 1-8, got {thruster_id}")
    return THRUSTER_FORCES[thruster_id]


def print_thruster_forces() -> None:
    """Print current thruster force configuration."""
    print("\nCURRENT THRUSTER FORCE CONFIGURATION:")
    for thruster_id in range(1, 9):
        force = THRUSTER_FORCES[thruster_id]
        print(f"  Thruster {thruster_id}: {force:.3f} N")
    print()


def validate_physics_params(config: PhysicsConfig) -> bool:
    """
    Validate physical parameters for consistency.

    Args:
        config: PhysicsConfig to validate

    Returns:
        True if valid, False otherwise
    """
    issues = []

    # Mass validation
    if config.total_mass <= 0:
        issues.append(f"Invalid mass: {config.total_mass}")

    # Inertia validation
    if config.moment_of_inertia <= 0:
        issues.append(f"Invalid moment of inertia: {config.moment_of_inertia}")

    # Thruster configuration validation
    if len(config.thruster_positions) != 8:
        issues.append(f"Expected 8 thrusters, got {len(config.thruster_positions)}")

    if len(config.thruster_directions) != 8:
        issues.append(
            f"Expected 8 thruster directions, got {len(config.thruster_directions)}"
        )

    if len(config.thruster_forces) != 8:
        issues.append(f"Expected 8 thruster forces, got {len(config.thruster_forces)}")

    # Validate thruster force values are positive
    for thruster_id, force in config.thruster_forces.items():
        if force <= 0:
            issues.append(f"Thruster {thruster_id} force must be positive, got {force}")

    # Report validation results
    if issues:
        print("Physics parameter validation failed:")
        for issue in issues:
            print(f"  - {issue}")
        return False

    return True
