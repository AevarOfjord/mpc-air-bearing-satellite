# Mission Architecture

## Overview

The satellite control system supports **two primary mission types**, each with flexible configuration options:

## Mission Types

### Mission 1: Waypoint Navigation
**Navigate to single or multiple waypoints in sequence**

**Features:**
- Single point-to-point navigation
- Multiple waypoint sequential navigation
- Customizable stabilization time at each waypoint (default: 5 seconds)
- Final stabilization time (default: 10 seconds)

**Use Cases:**
- Simple point-to-point maneuvers
- Multi-stop inspection routes
- Sequential target acquisition

**Configuration:**
- Starting position and orientation
- Target waypoint(s) with positions and orientations
- Intermediate stabilization time
- Final stabilization time

---

### Mission 2: Shape Following
**Follow a moving target along predefined shape paths**

**Available Shapes:**
1. **Circle** - Smooth circular path (0.25m radius)
2. **Rectangle** - 0.4m × 0.3m rectangular path
3. **Triangle** - Equilateral triangle with 0.4m sides
4. **Hexagon** - Regular hexagon with 0.2m radius
5. **Custom DXF** - Import any CAD shape from DXF files

**Features:**
- Moving target follows the shape path at configurable speed
- Configurable shape center, rotation, and offset distance
- Three-phase mission:
  1. **Positioning** - Move to closest point on shape (configurable stabilization)
  2. **Tracking** - Follow moving target along entire shape path
  3. **Stabilization** - Final hold at completion (configurable time)
- Optional return to starting position after completion

**Use Cases:**
- Orbital surveillance around targets
- Perimeter inspection
- Formation flying patterns
- Custom path following from CAD designs

**Configuration:**
- Shape selection (circle, rectangle, triangle, hexagon, or DXF file)
- Shape center position
- Shape rotation angle
- Offset distance (satellite path distance from shape)
- Target speed along path
- Optional return position

---

## Timing Configuration

All mission timing values are centralized in `config/timing.py` for easy customization:

### Waypoint Mission Timing
- `TARGET_HOLD_TIME`: 5.0 seconds (intermediate waypoint hold)
- `WAYPOINT_FINAL_STABILIZATION_TIME`: 10.0 seconds (final hold)

### Shape Following Mission Timing
- `SHAPE_POSITIONING_STABILIZATION_TIME`: 5.0 seconds (positioning phase)
- Tracking phase: Variable (depends on path length and speed)
- `SHAPE_FINAL_STABILIZATION_TIME`: 15.0 seconds (final stabilization)

### Configuration Location
See `config/timing.py` to modify these values. All constants are defined in one central location.

---

## Running Missions

### Simulation Mode
Run missions in a virtual environment without hardware:

```bash
python3 simulation.py
```

The simulator will display a menu to select:
1. **Waypoint Navigation** - Navigate to single or multiple waypoints
2. **Shape Following** - Follow a predefined shape (circle, rectangle, triangle, hexagon, or custom DXF)

### Real Hardware Mode
Run missions on the actual satellite hardware:

```bash
python3 real.py
```

**IMPORTANT**: Follow [HARDWARE_TEST_PROCEDURE.md](HARDWARE_TEST_PROCEDURE.md) safety procedures before running real hardware missions.

---

## Code Organization

### Key Files
- `mission.py` - Mission configuration and menu system
- `config/timing.py` - Timing parameters for both missions
- `satellite_config.py` - Mission state and configuration
- `mission_state_manager.py` - Runtime state management

### Mission Methods
- `run_multi_point_mode()` - Waypoint navigation (Mission 1)
- `run_dxf_shape_mode()` - Shape following (Mission 2)
- `get_demo_shape()` - Generate circle, rectangle, triangle, or hexagon

---

## Custom DXF File Configuration

To use a custom shape from a CAD drawing:

1. **Export your CAD design** as a DXF file
2. **Place the DXF file** in the project root or a `/dxf_files/` directory
3. **In simulation.py or real.py**, select "Shape Following" → "Custom DXF"
4. **Provide the file path** when prompted (e.g., `my_shape.dxf`)

The system will:
- Load the DXF geometry
- Extract the boundary/path
- Configure the satellite to follow the shape at your specified speed
- Center the path at your target location

---

## Mission Safety Bounds

The satellite operates within these physical constraints:

- **Work Area**: 6×6 meter air-bearing table
- **Position Bounds**: ±3.0 meters from origin in X and Y
- **Velocity Limits**: ±0.5 m/s per axis (MPC-controlled)
- **Angular Rate Limits**: ±1.0 rad/s (orientation control)
- **Thruster Limits**: 0-100% PWM per thruster

If the satellite approaches bounds during a mission:
- **Simulation**: Mission aborts with warning
- **Real Hardware**: Control system reduces speed and redirects trajectory

---

## Performance Metrics

After each mission completes, the system generates:

- **CSV data file** in `Data/` with timestamped position, velocity, and control inputs
- **Performance plots** showing trajectory, error, and control effort
- **Mission report** in `Data/` with success/failure status and statistics

View results:
```bash
python3 visualize.py
```
