# Satellite Thruster Control System

[![Python 3.9-3.12](https://img.shields.io/badge/python-3.9--3.12-blue.svg)](https://www.python.org/downloads/)
[![Master's Thesis](https://img.shields.io/badge/Master's-Thesis-green.svg)](ARCHITECTURE.md)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> **📌 Read-Only Reference Implementation**
>
> This repository is provided as a **complete reference implementation** for educational and research use. It is **not actively maintained** for external contributions.
>
> - ✅ **Download, fork, and modify** for your own research
> - ✅ **Use GitHub Discussions** for questions and comments
> - ❌ **Pull requests are not accepted** (see [CONTRIBUTING.md](.github/CONTRIBUTING.md))
> - 📧 **For urgent matters**: ofjord99@gmail.com

---

## What is This?

An **autonomous satellite control system** that uses **Model Predictive Control (MPC)** to navigate a satellite testbed on a 6×6 meter air-bearing table.

**In plain terms**: A small satellite floats frictionlessly on air bearings. It uses 8 compressed air thrusters and advanced control algorithms to move to targets and follow paths. Everything runs in software (simulation) or on real hardware.

---

## Quick Start

### 1. Install Python Dependencies
```bash
pip install -r requirements.txt  # Includes gurobipy (requires free academic license)
```

### 2. Get Gurobi License (Free for Academia)
- Visit: https://www.gurobi.com/academia/academic-program-and-licenses/
- Register with `.edu` email → download `gurobi.lic`
- Place `gurobi.lic` in project root

### 3. Run Simulation
```bash
python3 simulation.py
```
Follow the prompts to navigate to waypoints or follow shapes.

---

## What's Included

| What | What it Does | Command |
|------|-------------|---------|
| **simulation.py** | Run MPC control on virtual satellite | `python3 simulation.py` |
| **real.py** | Run MPC control on real hardware | `python3 real.py` |
| **mpc.py** | MPC solver & optimization logic | Imported by sim/real |
| **visualize.py** | Creates animations & performance plots (done automatically after each run, or manually) | `python3 visualize.py` |
| **comparison.py** | Compare sim vs real hardware results | `python3 comparison.py` |
| **testing_environment.py** | Manual control interface | `python3 testing_environment.py` |
| **config/** | All system parameters (physics, timing, MPC) | Import as needed |

---

## Documentation

### I'm taking over this project (handoff)
→ See [HANDOFF_GUIDE.md](HANDOFF_GUIDE.md) - Week 1 checklist, system health checks, maintenance schedule, known quirks

### I want to understand the system architecture
→ See [ARCHITECTURE.md](ARCHITECTURE.md) - Deep dive into control algorithms, physics model, module design

### I want to run hardware tests
→ See [HARDWARE_TEST_PROCEDURE.md](HARDWARE_TEST_PROCEDURE.md) - Safety checks, calibration, real-world testing

### I want to understand the mission types
→ See [MISSION_ARCHITECTURE.md](MISSION_ARCHITECTURE.md) - Waypoint navigation, shape following, DXF support

### I want to extend or modify the code
→ See [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) - Adding features, code style, testing your changes

### I want comprehensive simulation testing
→ See [SIMULATION_TESTING_GUIDE.md](SIMULATION_TESTING_GUIDE.md) - Simulation modes, parameter tuning, validation

### I want to understand data formats
→ See [CSV_FORMAT.md](CSV_FORMAT.md) - Data structure, analysis examples

### I want to write or understand tests
→ See [TESTING_GUIDE.md](TESTING_GUIDE.md) - Test structure, running tests, writing new tests

### I encountered a problem
→ See [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Installation, runtime, hardware, and data issues

---

## Key Features

- **Full Simulation**: Physics-based satellite dynamics with realistic thruster behavior
- **Real Hardware Control**: MPC on actual 6×6m air-bearing testbed
- **8 Thruster Control**: Individual calibration for each thruster
- **Multiple Mission Types**: Waypoint navigation, shape following (circle, rectangle, triangle, hexagon, custom DXF)
- **Visualization**: Animated results + performance plots
- **Comprehensive Tests**: 300+ unit and integration tests
- **Research Foundation**: Modular, extensible architecture for new control strategies

---

## Installation

### Prerequisites
- **Python 3.9, 3.10, 3.11, or 3.12** (3.13+ not supported yet)
- Check your version: `python3 --version`

### Steps
```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Get free academic Gurobi license
# - Visit: https://www.gurobi.com/academia/academic-program-and-licenses/
# - Register with .edu email → download gurobi.lic
# - Place gurobi.lic in project root

# 3. Install FFmpeg
# macOS: brew install ffmpeg
# Linux: sudo apt install ffmpeg
# Windows: https://ffmpeg.org/download.html

# 4. Verify everything works
python3 -c "import gurobipy; print('Gurobi OK')"
python3 simulation.py
```

---

## Common Tasks

| Task | Command | Output |
|------|---------|--------|
| Run simulation | `python3 simulation.py` | CSV data + MP4 animation + plots |
| Run hardware control | `python3 real.py` | Live hardware control + CSV data + MP4 animation + plots + Live video |
| Generate visualizations | `python3 visualize.py` | Manually creates MP4 animation + PNG plots from selected CSV data file|
| Compare sim vs real | `python3 comparison.py` | Side-by-side analysis |
| Manual control in simulated environment| `python3 testing_environment.py` | Live interactive interface to test simulation environment |
| Run tests | `pytest` | Test results + coverage |

---

## Troubleshooting

**Python 3.13+ not supported?**
→ Use Python 3.9-3.12 instead. Install from https://www.python.org/downloads/

**"No module named gurobipy"?**
→ Install Gurobi (free academic license) and place `gurobi.lic` in project root

**Serial port errors?**
→ Update `SERIAL_PORT` in config/constants.py. Check Device Manager (Windows) or `ls /dev/tty.*` (Mac/Linux)

**MPC solver too slow?**
→ Reduce `MPC_PREDICTION_HORIZON` in config/mpc_params.py

**FFmpeg not found?**
→ Install FFmpeg or update `FFMPEG_PATH` in config

For more troubleshooting, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

## About This Project

Master's thesis project from University of Kentucky Mechanical Engineering. The system demonstrates Model Predictive Control for autonomous satellite navigation on a 6×6m air-bearing testbed.

**Why it exists**: Research platform for studying MPC-based attitude and position control with real hardware validation.

**Who should use it**: Researchers, students, and educators interested in control systems and autonomous satellite operations.

### Vision for Others

This project was designed with **extensibility and replicability** in mind. If you're interested in MPC research or small satellite testing:

- **Download and adapt**: Fork this repo and customize for your hardware
- **Reference implementation**: Use as a starting point so you don't have to build from scratch
- **MPC controller**: The core `mpc.py` works with any air-bearing testbed (adjust physics parameters)
- **Simulation**: Validate algorithms in software before hardware investment

**Note**: I want to maintain this as a reference implementation. You're welcome to fork and adapt it for your own research, but modifications stay on your own copy. This ensures the canonical version remains stable for others.

---

## Contributing

This is a personal research project released for educational and reference use. I do **not accept pull requests**, but you can fork and modify it for your own work.

### If you find issues:

- **Report bugs**: [Create a bug report](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=bug_report.md) - I'll fix them when I can
- **Hardware issues**: [Report problems](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=hardware_issue.md) - Helps improve reliability
- **Documentation**: [Suggest improvements](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=documentation.md) - Clarifications welcome

### Want to modify the code?

**Fork and adapt for your research!** See [CONTRIBUTING.md](.github/CONTRIBUTING.md) for details.

**Note**: This project is released as-is for educational and reference use. Bug fixes and improvements are made on a best-effort basis.

---

## Credits and Acknowledgments

**Author**: Aevar Amundinusarson Oefjoerd (Master's Thesis Project)

**Advisors**:
- Dr. Poonawala
- Dr. Seigler

**Institution**: University of Kentucky, Department of Mechanical Engineering

**Key Technologies**:
- **Gurobi Optimization**: Academic license for MPC solver
- **Python Scientific Stack**: NumPy, SciPy, Matplotlib
- **PyQt5**: User interface framework
- **OptiTrack Motive**: Motion capture system
- **FFmpeg**: Video generation

**Acknowledgments**:
- Dr. Poonawala, Dr. Seigler, UK Mechanical Engineering faculty and staff for facility access and support
- Open source community for excellent tools and libraries

---

## Support

**Repository**: https://github.com/AevarOfjord/mpc-air-bearing-satellite

**Issues**: https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues

Need help? Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md) or open an issue.

---

**Ready to get started?**
```bash
python3 simulation.py
```
