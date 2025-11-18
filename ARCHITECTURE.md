# Satellite Thruster Control System - Architecture Documentation

This document provides a complete file listing and description of the system architecture. For project overview and features, see [README.md](README.md).

---

## Directory Structure

```
mpc-air-bearing-satellite/
│
├── Main Entry Points (root level)
│   ├── simulation.py                 # Simulation environment launcher
│   ├── real.py                       # Real hardware control launcher
│   ├── visualize.py                  # Unified visualization & animation
│   ├── comparison.py                 # Compare simulation vs real results
│   └── testing_environment.py        # Interactive physics testing
│
├── Core Control & Physics
│   ├── mpc.py                        # Model Predictive Control optimizer
│   ├── model.py                      # Satellite physics model
│   ├── mission.py                    # Mission logic & waypoint handling
│   ├── mission_state_manager.py      # Track mission state
│   └── mission_report_generator.py   # Generate mission reports
│
├── Hardware & Communication
│   ├── satellite_hardware_interface.py   # Serial communication
│   ├── telemetry_client.py              # Motion capture data polling
│   └── camera_manager.py                # Multi-camera streaming
│
├── Navigation & Planning
│   ├── path_planning_manager.py     # Obstacle avoidance
│   └── navigation_utils.py          # Geometric utilities
│
├── Data & Logging
│   ├── data_logger.py               # CSV export
│   ├── extract_initial_velocities.py # Velocity extraction
│   └── real_data_simulated.py       # Data playback
│
├── UI & Visualization
│   ├── simulation_visualization.py   # Animation generation
│   ├── real_ui_components.py        # Real-time dashboard
│   ├── gui_components.py            # Shared GUI components
│   └── visualization_manager.py     # Visualization orchestration
│
├── Modes & Setup
│   ├── simulation_test_modes.py      # Simulation mode menu
│   ├── real_test_modes.py           # Hardware mode menu
│   ├── mode_setup_manager.py        # Unified setup logic
│   └── simulation_state_validator.py # State validation
│
├── Utilities & Infrastructure
│   ├── exceptions.py                # Custom exceptions
│   ├── logging_config.py            # Logging configuration
│   └── satellite_config.py          # Central config wrapper interface
│
├── Configuration System (config/)
│   ├── __init__.py                  # Package initialization
│   ├── physics.py                   # Physical parameters
│   ├── timing.py                    # Timing parameters
│   ├── mpc_params.py                # MPC tuning parameters
│   ├── camera.py                    # Camera configuration
│   ├── constants.py                 # System constants
│   ├── mission_state.py             # Mission state
│   └── obstacles.py                 # Obstacle definitions
│
├── Motion Capture Integration (Motive/)
│   ├── nat_net_client.py            # NatNet protocol client
│   ├── motive_data_server.py        # HTTP data server
│   ├── mocap_data.py                # Motion capture data structures
│   └── data_descriptions.py         # NatNet format definitions
│
├── Shape & DXF Processing (DXF/)
│   ├── __init__.py                  # Package initialization
│   ├── dxf_viewer.py                # DXF file viewer
│   ├── dxf_shape_maker.py           # DXF file creator
│   └── DXF_Files/                   # Pre-defined shape files
│       ├── Shape1.dxf
│       ├── Shape2.dxf
│       ├── Shape3.dxf
│       ├── Shape4.dxf
│       └── Shape5.dxf
│
├── Thruster Testing & Calibration (Thruster_Test/)
│   ├── thruster_test.py             # Sequential thruster verification
│   ├── thrust_calibration.py        # Thruster force calibration
│   ├── analyze_thruster_data.py     # Calibration data analysis
│   └── Thruster_Data/               # Calibration test results
│
├── Testing Infrastructure (tests/)
│   ├── __init__.py                  # Package initialization
│   ├── README.md                    # Testing documentation
│   ├── conftest.py                  # pytest configuration
│   ├── test_config.py               # Configuration tests
│   ├── test_mpc_controller.py       # MPC tests
│   ├── test_model.py                # Physics model tests
│   ├── test_mission_state_manager.py # Mission state tests
│   ├── test_path_planning_manager.py # Path planning tests
│   ├── test_navigation_utils.py     # Navigation utility tests
│   ├── test_data_logger.py          # Data logging tests
│   ├── test_camera_manager.py       # Camera manager tests
│   ├── test_simulation_state_validator.py # State validation tests
│   ├── test_integration_basic.py    # Basic integration tests
│   └── test_integration_missions.py # Mission integration tests
│
├── Data Output (Data/)
│   ├── Simulation/                  # Simulation results
│   ├── Real_Test/                   # Hardware test results
│   └── Comparison/                  # Comparative analysis
│
├── Project Files
│   ├── README.md                    # Project overview & quick start
│   ├── ARCHITECTURE.md              # This file
│   ├── MISSION_ARCHITECTURE.md      # Mission system architecture
│   ├── HANDOFF_GUIDE.md             # Handoff documentation
│   ├── QUICK_START_CHECKLIST.md     # First week checklist
│   ├── HARDWARE_TEST_PROCEDURE.md   # Hardware testing guide
│   ├── THRUSTER_CALIBRATION.md      # Thruster calibration guide
│   ├── DEVELOPMENT_GUIDE.md         # Development guidelines
│   ├── TESTING_GUIDE.md             # Testing documentation
│   ├── SIMULATION_TESTING_GUIDE.md  # Simulation testing guide
│   ├── TROUBLESHOOTING.md           # Troubleshooting guide
│   ├── CSV_FORMAT.md                # CSV data format documentation
│   ├── CODE_OF_CONDUCT.md           # Code of conduct
│   ├── SECURITY.md                  # Security policy
│   ├── LESSONS_LEARNED.md           # Development lessons learned
│   ├── requirements.txt             # Python dependencies
│   ├── pyproject.toml               # Project metadata
│   ├── .gitignore                   # Git ignore rules
│   └── LICENSE                      # License
│
├── GitHub Configuration (.github/)
│   ├── CONTRIBUTING.md              # Contribution guidelines
│   ├── SUPPORT.md                   # Support resources
│   └── ISSUE_TEMPLATE/              # Issue templates
│       ├── config.yml               # Issue template configuration
│       ├── bug_report.md            # Bug report template
│       ├── feature_request.md       # Feature request template
│       ├── hardware_issue.md        # Hardware issue template
│       └── documentation.md         # Documentation issue template
│
├── Images (images/)                 # Project images and diagrams
│
└── Virtual Environment (venv/)      # Python virtual environment

```

---

## Complete File Listing

### Root Level - Main Entry Points

| File | Purpose |
|------|---------|
| [simulation.py](simulation.py) | Main entry point for physics-based simulation of satellite control with MPC optimization. |
| [real.py](real.py) | Main entry point for real hardware control system that interfaces with microcontroller and motion capture. |
| [visualize.py](visualize.py) | Unified visualization system that generates MP4 animations and performance plots from CSV data. |
| [comparison.py](comparison.py) | Compares simulation results against real hardware test results side-by-side. |
| [testing_environment.py](testing_environment.py) | Interactive testing interface for virtual satellite simulation and physics validation. |

### Core Control & Physics

| File | Purpose |
|------|---------|
| [mpc.py](mpc.py) | Model Predictive Control optimizer using Gurobi to generate thruster commands. |
| [model.py](model.py) | Satellite physics model with linearized dynamics for simulation and visualization. |
| [mission.py](mission.py) | Unified mission module supporting waypoint navigation and shape-following behaviors. |
| [mission_state_manager.py](mission_state_manager.py) | Tracks mission state, waypoint progress, and determines when targets are reached. |
| [mission_report_generator.py](mission_report_generator.py) | Generates text summaries and performance analysis after missions complete. |

### Hardware & Communication

| File | Purpose |
|------|---------|
| [satellite_hardware_interface.py](satellite_hardware_interface.py) | Abstracts serial communication with microcontroller for thruster commands. |
| [telemetry_client.py](telemetry_client.py) | HTTP client that polls motion capture data from Motive data server. |
| [camera_manager.py](camera_manager.py) | Manages multi-camera streaming and recording from ESP32-CAM modules. |

### Navigation & Planning

| File | Purpose |
|------|---------|
| [path_planning_manager.py](path_planning_manager.py) | Generates safe paths around obstacles using intermediate waypoint insertion. |
| [navigation_utils.py](navigation_utils.py) | Geometric utilities for distance calculations, angle conversions, and path validation. |

### Data & Logging

| File | Purpose |
|------|---------|
| [data_logger.py](data_logger.py) | Exports mission state and control data to CSV format for analysis. |
| [extract_initial_velocities.py](extract_initial_velocities.py) | Utility to extract initial velocity estimates from CSV data for controller initialization. |
| [real_data_simulated.py](real_data_simulated.py) | Playback mode that replays real hardware data through simulation.|

### UI & Visualization

| File | Purpose |
|------|---------|
| [simulation_visualization.py](simulation_visualization.py) | Generates real-time animation of satellite trajectory with thrust vectors during simulation. |
| [real_ui_components.py](real_ui_components.py) | PyQt5 components for real-time monitoring dashboard during hardware missions. |
| [gui_components.py](gui_components.py) | Shared GUI components and widgets used across both simulation and hardware modes. |
| [visualization_manager.py](visualization_manager.py) | Orchestrates animation generation, plot creation, and media export. |

### Configuration System

| File | Purpose |
|------|---------|
| [config/__init__.py](config/__init__.py) | Configuration package exports all dataclass modules. |
| [config/physics.py](config/physics.py) | Physical parameters: satellite mass, inertia, thruster positions, forces, drag coefficients. |
| [config/timing.py](config/timing.py) | Timing parameters: simulation timestep, control rate, stabilization delays. |
| [config/mpc_params.py](config/mpc_params.py) | MPC tuning: prediction horizon, control horizon, cost weights, constraints. |
| [config/camera.py](config/camera.py) | Camera configuration: ESP32-CAM URLs, recording settings, frame rates. |
| [config/constants.py](config/constants.py) | System constants: serial ports, network parameters, data paths, UI dimensions. |
| [config/mission_state.py](config/mission_state.py) | Runtime mission state tracking: waypoints, targets, phase information. |
| [config/obstacles.py](config/obstacles.py) | Obstacle definitions: circular obstacles with safety radii for path planning. |

### Modes & Setup

| File | Purpose |
|------|---------|
| [simulation_test_modes.py](simulation_test_modes.py) | Interactive menu for selecting simulation scenarios (waypoint, shapes, DXF). |
| [real_test_modes.py](real_test_modes.py) | Interactive menu for real hardware missions with manual parameter input. |
| [mode_setup_manager.py](mode_setup_manager.py) | Unified setup logic for initializing simulation or hardware modes. |
| [simulation_state_validator.py](simulation_state_validator.py) | Validation checks for simulation state consistency and physics constraints. |

### Utilities & Infrastructure

| File | Purpose |
|------|---------|
| [exceptions.py](exceptions.py) | Custom exception classes for mission, hardware, and control errors. |
| [logging_config.py](logging_config.py) | Centralized logging configuration with file and console handlers. |
| [satellite_config.py](satellite_config.py) | Unified wrapper interface for accessing configuration modules with thruster calibration support. |

### Motion Capture Integration (Motive)

| File | Purpose |
|------|---------|
| [Motive/nat_net_client.py](Motive/nat_net_client.py) | OptiTrack NatNet protocol client for receiving UDP motion capture packets. |
| [Motive/motive_data_server.py](Motive/motive_data_server.py) | HTTP server exposing latest motion capture data as JSON endpoints. |
| [Motive/mocap_data.py](Motive/mocap_data.py) | Data structures for rigid body tracking, skeletons, markers, and force plates. |
| [Motive/data_descriptions.py](Motive/data_descriptions.py) | NatNet packet format definitions and unpacking utilities. |

### Shape & DXF Processing

| File | Purpose |
|------|---------|
| [DXF/dxf_viewer.py](DXF/dxf_viewer.py) | Loads and visualizes DXF shape files with customizable offset and smoothing. |
| [DXF/dxf_shape_maker.py](DXF/dxf_shape_maker.py) | Creates DXF shape files for defining custom mission paths and obstacles. |
| [DXF/__init__.py](DXF/__init__.py) | DXF module package initialization. |

### Thruster Testing & Calibration

| File | Purpose |
|------|---------|
| [Thruster_Test/thruster_test.py](Thruster_Test/thruster_test.py) | Quick sequential verification that fires each of 8 thrusters for 1 second. |
| [Thruster_Test/thrust_calibration.py](Thruster_Test/thrust_calibration.py) | Measures actual thruster forces and updates physics configuration with calibrated values. |
| [Thruster_Test/analyze_thruster_data.py](Thruster_Test/analyze_thruster_data.py) | Analyzes thrust calibration data and generates performance statistics. |

### Testing Infrastructure

| File | Purpose |
|------|---------|
| [tests/__init__.py](tests/__init__.py) | Test package initialization. |
| [tests/README.md](tests/README.md) | Testing documentation and guidelines. |
| [tests/conftest.py](tests/conftest.py) | pytest fixtures and configuration for unit and integration tests. |
| [tests/test_config.py](tests/test_config.py) | Tests for configuration system and parameter validation. |
| [tests/test_mpc_controller.py](tests/test_mpc_controller.py) | Tests for MPC optimization, constraint handling, and solver accuracy. |
| [tests/test_model.py](tests/test_model.py) | Tests for physics model linearization and state updates. |
| [tests/test_mission_state_manager.py](tests/test_mission_state_manager.py) | Tests for mission state tracking and waypoint detection. |
| [tests/test_path_planning_manager.py](tests/test_path_planning_manager.py) | Tests for obstacle avoidance and path validation. |
| [tests/test_navigation_utils.py](tests/test_navigation_utils.py) | Tests for geometric calculations and angle conversions. |
| [tests/test_data_logger.py](tests/test_data_logger.py) | Tests for CSV export and data format correctness. |
| [tests/test_camera_manager.py](tests/test_camera_manager.py) | Tests for camera streaming and recording functionality. |
| [tests/test_simulation_state_validator.py](tests/test_simulation_state_validator.py) | Tests for physics constraint validation. |
| [tests/test_integration_basic.py](tests/test_integration_basic.py) | Integration tests for basic mission execution. |
| [tests/test_integration_missions.py](tests/test_integration_missions.py) | Integration tests for complex multi-waypoint and shape-following missions. |

---

## Architecture Layers

The system is organized into logical layers:

1. **Entry Points** - `simulation.py`, `real.py`, `visualize.py` coordinate the system
2. **Control Layer** - `mpc.py` generates commands, `mission_state_manager.py` tracks state
3. **Hardware/Telemetry Layer** - `satellite_hardware_interface.py`, `telemetry_client.py` handle I/O
4. **Navigation Layer** - `path_planning_manager.py`, `mission.py` plan trajectories
5. **Data Layer** - `data_logger.py`, `real_data_simulated.py` manage persistence
6. **Visualization Layer** - `visualize.py`, UI components generate outputs
7. **Configuration Layer** - `config/` provides unified parameter management

---

### **Key Architectural Differences**

| Aspect | Simulation Mode | Hardware Mode |
|--------|-----------------|---------------|
| **State Source** | Physics model (`model.py`) | Motion capture system (OptiTrack) |
| **Loop Rate** | CONTROL_DT = 0.06 s | CONTROL_DT = 0.06s |
| **Target Update** | Before MPC call | During MPC call |
| **Sensor Input** | Simulated with noise | Real hardware|
| **Telemetry** | Internal state | HTTP polling from motive_data_server |
| **Control Output** | Applied to physics model | Sent to onboard microcontroller via serial |

In simulation the target is updated right before each MPC call because the entire loop runs inside `update_simulation()` once the physics step finishes, MissionStateManager selects the next waypoint/DXF phase and the controller immediately uses that target. On hardware the control thread first gathers fresh OptiTrack measurements, then calls `MissionStateManager.update_target_state()` inside the same iteration so the MPC solver works with the newest telemetry before issuing serial commands to the microcontroller.

---

### **Common Elements**

Both simulation and hardware modes use the same components for core functionality:

1. **MPC Controller** (`mpc.py`) - Identical optimization engine for both
2. **Mission Logic** (`mission.py`) - Same waypoint/shape-following logic
3. **Data Logging** (`data_logger.py`) - Identical CSV recording format
4. **Path Planning** (`path_planning_manager.py`) - Shared
5. **Post-Processing** (`visualize.py`) - Same animation/plot generation

---

## Key Design Principles

- **Modularity:** Each file has a single, clear responsibility
- **Configuration-driven:** All parameters in `config/` dataclasses
- **Simulation/Hardware parity:** Same `mpc.py` and `mission.py` code used for both modes
- **Testing:** Comprehensive unit and integration tests for all core components
- **Documentation:** Each file has docstring describing its purpose
