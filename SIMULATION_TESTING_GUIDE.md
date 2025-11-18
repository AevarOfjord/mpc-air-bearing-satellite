# Simulation Testing Guide

Complete guide for running simulation tests without hardware requirements.

**Good for**: Testing control algorithms, mission planning, parameter tuning, and validation before hardware deployment.

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Installation](#installation)
3. [Running Your First Simulation](#running-your-first-simulation)
4. [Mission Types](#mission-types)
5. [Configuration & Tuning](#configuration--tuning)
6. [Output & Visualization](#output--visualization)
7. [Advanced Testing](#advanced-testing)
8. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Minimal Setup

```bash
# Install dependencies
pip install -r requirements.txt

# Get Gurobi academic license (free for students)
# Place gurobi.lic in project root

# Run simulation
python3 simulation.py
```

---

## Installation

### 1. Python Dependencies

```bash
pip install -r requirements.txt
```

**Required packages:**
- numpy, scipy, matplotlib
- gurobipy (optimization solver)
- PyQt5 (for visualization)
- opencv-python
- ezdxf, shapely (for DXF missions)

### 2. Gurobi License Setup

Gurobi is **free for academic use**:

1. Register at: https://www.gurobi.com/academia/academic-program-and-licenses/
2. Generate license (use university email)
3. Download `gurobi.lic`
4. Place in project root directory

**Verify installation:**
```bash
python3 -c "import gurobipy as gp; m = gp.Model(); print('Gurobi working!')"
```

### 3. FFmpeg for Animation

**macOS:**
```bash
brew install ffmpeg
```

**Linux:**
```bash
sudo apt install ffmpeg
```

**Windows:**
- Download from https://ffmpeg.org
- Add to PATH or update `FFMPEG_PATH` in `satellite_config.py`

---

## Running Your First Simulation

### Basic Command

```bash
python3 simulation.py
```

### What Happens

1. **Mission Selection** - Interactive prompt asks which mission to run
2. **Configuration** - Set start/target positions, mission parameters
3. **Simulation** - Real-time physics simulation with MPC control
4. **Data Logging** - Saves CSV with all state/control data
5. **Visualization** - Auto-generates MP4 animation and plots

### Example Output

```
Satellite MPC Simulation (Linearized)
=========================================

Select mission mode:
1. Waypoint Mission
2. Shape Following

Enter mode (1-2): 1

--- Configuration ---
Start position: (-1.0, -1.0) m, angle: 90°
Target position: (0.0, 0.0) m, angle: 90°

Running simulation...
Step 100/500 | Position error: 0.15m | MPC: 23ms
Step 200/500 | Position error: 0.08m | MPC: 19ms
Step 300/500 | Position error: 0.03m | MPC: 21ms
Mission complete! Reached target.

Results saved to: Data/Simulation/10-11-2025_14-30-45/
```

---

## Mission Types

The simulation supports 2 primary mission types that test different aspects of satellite control and navigation.

### 1. Waypoint Navigation

Navigate to single or multiple waypoints in sequence with configurable stabilization times.

**Use case:** Path planning, sequential target navigation, inspection missions

**Configuration:**
```python
python3 simulation.py
# Select: 1 (Waypoint Navigation)
# Enter waypoints: (0, 0), (1, 0), (1, 1), (0, 1)
# Specify target angles for each waypoint
# Optional: Configure obstacles for avoidance
```

**Mission behavior:**
- Navigates to each waypoint sequentially
- Stabilizes at each waypoint for configurable time (default: 5s) before proceeding
- Supports obstacle configuration for path planning
- Each waypoint includes position (x, y) and orientation angle
- Final waypoint holds for 10s by default

**What it tests:**
- Waypoint-to-waypoint navigation accuracy
- Waypoint sequencing logic
- Stabilization at each target
- Path planning between points
- Obstacle avoidance (if configured)

**Timing Configuration** (in `config/timing.py`):
- `TARGET_HOLD_TIME`: 5.0 seconds (intermediate waypoints)
- `WAYPOINT_FINAL_STABILIZATION_TIME`: 10.0 seconds (final waypoint)

**Termination:** Completes when all waypoints are visited and satellite stabilizes at final waypoint

---

### 2. Shape Following

Follow a moving target along predefined geometric paths (circles, rectangles, triangles, hexagons, or custom DXF files).

**Use case:** Orbital surveillance, perimeter inspection, formation flying, custom trajectory following

**Configuration:**
```python
python3 simulation.py
# Select: 2 (Shape Following)
# Choose shape: circle, rectangle, triangle, hexagon, or custom DXF
# Shape center: (0, 0) (configurable)
# Rotation angle: 0 degrees (configurable)
# Offset distance: 0.6m (default, satellite path distance from shape)
# Target speed: 0.05 m/s (default)
# Optional: Return position after completing path
```

**Mission behavior:**
- Three execution phases:
  1. **Positioning phase**: Move to closest point on shape and stabilize (5s)
  2. **Tracking phase**: Follow moving target along the entire shape path
  3. **Stabilization phase**: Final hold at completion (15s) or return to starting position
- Target moves along path at configurable speed
- Satellite maintains specified offset distance while following target

**Available Shapes:**
- **Circle** - Smooth circular orbit (36 points, configurable radius)
- **Rectangle** - 0.4m × 0.3m rectangular path
- **Triangle** - Equilateral triangle with 0.4m sides
- **Hexagon** - Regular hexagon with 0.2m radius
- **Custom DXF** - Import any CAD shape from DXF files

**What it tests:**
- Shape trajectory tracking
- Dynamic target following
- Continuous motion control
- Moving reference frame control
- Multi-phase mission execution

**Timing Configuration** (in `config/timing.py`):
- `SHAPE_POSITIONING_STABILIZATION_TIME`: 5.0 seconds (positioning phase)
- `SHAPE_FINAL_STABILIZATION_TIME`: 15.0 seconds (final stabilization)

**Termination:** Completes when shape path is finished and satellite stabilizes for configured time

---

## Configuration & Tuning

### Key Configuration File

**All parameters in**: `satellite_config.py` (or `config/` package)

### Important Parameters

#### Physics
```python
SIMULATION_DT = 0.02        # 20ms - Physics timestep
CONTROL_DT = 0.06           # 60ms - MPC update rate
TOTAL_MASS = 23.09          # kg
MOMENT_OF_INERTIA = 0.312   # kg⋅m²
```

#### MPC Controller
```python
MPC_PREDICTION_HORIZON = 15  # Steps (~0.9s lookahead)
MPC_CONTROL_HORIZON = 12     # Control freedom
MPC_SOLVER_TIME_LIMIT = 0.05 # 50ms max solve time
```

#### Cost Weights
```python
Q_POSITION = 1000.0         # Position tracking
Q_VELOCITY = 1750.0         # Velocity damping
Q_ANGLE = 1000.0            # Angle tracking
Q_ANGULAR_VELOCITY = 1500.0 # Angular damping
R_THRUST = 1.0              # Thrust penalty
```

### Realistic vs Idealized Physics

#### Enable Realistic Physics
```python
# In satellite_config.py
ENABLE_REALISTIC_PHYSICS = True
```

**Includes:**
- Air resistance damping
- Sensor noise
- Thruster delays
- Random disturbances

#### Idealized Physics
```python
ENABLE_REALISTIC_PHYSICS = False
```

**Perfect conditions:**
- No damping
- No noise
- Instant thruster response
- No disturbances

### Tuning Guidelines

**For faster response:**
- ↑ Increase `Q_POSITION`
- ↑ Increase `MPC_PREDICTION_HORIZON`
- ↓ Decrease `R_THRUST`

**For smoother motion:**
- ↑ Increase `Q_VELOCITY`
- ↑ Increase `R_SWITCH` (if thruster switching too frequent)
- ↓ Decrease `MPC_CONTROL_HORIZON`

**For better settling:**
- ↑ Increase `Q_VELOCITY` and `Q_ANGULAR_VELOCITY`
- ↑ Increase `VELOCITY_THRESHOLD`
- ↑ Increase `DAMPING_ZONE`

**Always verify:**
```python
from config import SatelliteConfig
SatelliteConfig.validate_parameters()
```

---

## Output & Visualization

### Automatic Data Saving

After each simulation, data is saved to:
```
Data/Simulation/DD-MM-YYYY_HH-MM-SS/
├── simulation_data.csv       # All state/control data
├── simulation_animation.mp4  # Visual playback
└── simulation_plots.png      # Performance graphs
```

### CSV Data Format

Contains timestamped data:
- Current state (position, velocity, angle)
- Target state
- Errors (position, angle)
- MPC computation time
- Thruster commands
- Timing violations

### Viewing Results

**Auto-generated:**
- Animation created automatically after simulation
- Plots generated automatically

**Manual visualization:**
```bash
# Visualize latest simulation
python3 visualize.py

# Visualize specific run
python3 visualize.py Data/Simulation/10-11-2025_14-30-45/
```

### Data Analysis

```python
import pandas as pd
import numpy as np

# Load data
data = pd.read_csv('Data/Simulation/[timestamp]/simulation_data.csv')

# Calculate metrics
pos_error = np.sqrt(data['Error_X']**2 + data['Error_Y']**2)
avg_error = pos_error.mean()
max_error = pos_error.max()

# MPC performance
avg_solve_time = data['MPC_Computation_Time'].mean() * 1000  # ms
violations = data['Timing_Violation'].sum()

print(f"Average position error: {avg_error:.4f}m")
print(f"Max position error: {max_error:.4f}m")
print(f"Average MPC solve: {avg_solve_time:.1f}ms")
print(f"Timing violations: {violations}")
```

---

## Advanced Testing

### Comparison Testing

Compare simulation vs real hardware tests using the comparison tool:

```bash
python3 comparison.py --real Data/Real_Test/[timestamp]/ --sim Data/Simulation/[timestamp]/
```

Generates side-by-side comparison showing:
- Mission duration
- Thruster activation count
- Position error
- Control effort

### Interactive Testing

Manual thruster control for experimentation:

```bash
python3 testing_environment.py
```

**Controls:**
- **1-8**: Fire individual thrusters
- **Arrow keys**: Directional movement
- **Space**: Stop all thrusters
- **R**: Reset to origin
- **ESC**: Exit

**Use case:** Understanding thruster effects, COM offset testing

### Batch Testing

Run multiple simulations with different parameters:

```python
import subprocess
import numpy as np

# Test different Q_POSITION values
for q_pos in [500, 1000, 2000, 5000]:
    # Update config
    # ... (modify satellite_config.py)

    # Run simulation
    result = subprocess.run(['python3', 'simulation.py'])

    # Analyze results
    # ...
```

### Parameter Sweep

Test range of MPC parameters:

```python
from config import SatelliteConfig
import itertools

horizons = [10, 15, 20]
weights = [500, 1000, 2000]

for pred_h, q_pos in itertools.product(horizons, weights):
    SatelliteConfig.MPC_PREDICTION_HORIZON = pred_h
    SatelliteConfig.Q_POSITION = q_pos

    # Run simulation and log results
    # ...
```

---

## Troubleshooting

### Gurobi License Issues

**Problem:** "Model too large for license"

**Solution:**
- Get academic license (free, larger limits)
- OR reduce `MPC_PREDICTION_HORIZON`

```python
MPC_PREDICTION_HORIZON = 12  # Reduce from 15
MPC_CONTROL_HORIZON = 9      # Reduce from 12
```

### MPC Solver Too Slow

**Problem:** "Solver time limit exceeded" warnings

**Solution:**
```python
# Reduce problem size
MPC_PREDICTION_HORIZON = 12
MPC_CONTROL_HORIZON = 9

# OR increase time limit
MPC_SOLVER_TIME_LIMIT = 0.1  # 100ms
```

### Simulation Unstable

**Problem:** Satellite spinning out of control

**Possible causes:**
- Q_VELOCITY too low (increase damping)
- R_THRUST too high (allow more control)
- MPC_CONTROL_HORIZON too small

**Solution:**
```python
Q_VELOCITY = 2500.0          # More damping
Q_ANGULAR_VELOCITY = 2000.0  # More angular damping
R_THRUST = 0.5               # Less thrust penalty
```

### Can't Reach Target

**Problem:** Simulation never converges

**Possible causes:**
- Target too far
- Thrusters too weak
- Velocity limits too restrictive

**Solution:**
```python
# Check constraints
MAX_VELOCITY = 0.20          # Increase from 0.15
POSITION_THRESHOLD = 0.03    # Relax from 0.02
ANGLE_THRESHOLD = 0.1        # Relax from 0.05
```

### Animation Not Generated

**Problem:** No MP4 file created

**Possible causes:**
- FFmpeg not installed
- Wrong FFmpeg path

**Solution:**
```bash
# Install FFmpeg (see Installation section)

# OR update path in config
FFMPEG_PATH = '/usr/local/bin/ffmpeg'  # macOS
FFMPEG_PATH = '/usr/bin/ffmpeg'        # Linux
FFMPEG_PATH = 'C:/ffmpeg/bin/ffmpeg.exe'  # Windows
```

### Import Errors

**Problem:** `ModuleNotFoundError` or `ImportError`

**Solution:**
```bash
# Reinstall dependencies
pip install --upgrade -r requirements.txt

# Check Python version
python3 --version  # Should be 3.7+

# Use virtual environment
python3 -m venv venv
source venv/bin/activate  # Linux/macOS
venv\Scripts\activate     # Windows
pip install -r requirements.txt
```

---

## Quick Reference

### Common Commands

```bash
# Run simulation
python3 simulation.py

# Visualize data manually
python3 visualize.py

# Interactive virtual environment testing
python3 testing_environment.py

# Comparison test
python3 comparison.py --real Data/Real_Test/<timestamp>/ --sim Data/Simulation/<timestamp>/

# Validate config
python3 -c "from config import SatelliteConfig; SatelliteConfig.validate_parameters()"
```

### Key Files

| Purpose | File | Description |
|---------|------|-------------|
| Run simulation | `simulation.py` | Main simulation script |
| Configure | `satellite_config.py` | All parameters |
| Visualize | `visualize.py` | Generate animations/plots |
| Test manually | `testing_environment.py` | Interactive control |
| Compare | `comparison.py` | Side-by-side comparison |

### Default Parameters

- Simulation timestep: **20ms**
- Control interval: **60ms**
- MPC horizon: **15 steps** (~0.9s)
- Solver limit: **50ms**
- Position threshold: **2cm**
- Angle threshold: **3°**
