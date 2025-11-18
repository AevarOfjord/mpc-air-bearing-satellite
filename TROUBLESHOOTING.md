# Troubleshooting Guide

Common issues and solutions for the Satellite Thruster Control System.

---

## Table of Contents

- [Installation Issues](#installation-issues)
- [Runtime Errors](#runtime-errors)
- [Hardware Issues](#hardware-issues)
- [Performance Issues](#performance-issues)
- [Data Analysis Issues](#data-analysis-issues)
- [Getting Help](#getting-help)

---

## Installation Issues

### Python Version Incompatibility

**Problem**: "Python 3.13+ not supported" or packages fail to install

**Solution**:
```bash
# Check your Python version
python3 --version

# Must be Python 3.9, 3.10, 3.11, or 3.12
# Download from https://www.python.org/downloads/

# Alternative: Use pyenv for version management
pyenv install 3.12.0
pyenv local 3.12.0

# Alternative: Use conda
conda create -n satellite python=3.12
conda activate satellite
```

### "ModuleNotFoundError: No module named 'gurobipy'"

**Problem**: Gurobi not installed or license not found

**Solution**:
```bash
# 1. Install Gurobi
pip install gurobipy

# 2. Get academic license (free)
# - Visit: https://www.gurobi.com/academia/academic-program-and-licenses/
# - Register with .edu email
# - Download gurobi.lic file

# 3. Place license file in project root
cp ~/Downloads/gurobi.lic /path/to/mpc-air-bearing-satellite/

# 4. Verify installation
python3 -c "import gurobipy as gp; print('Gurobi OK')"
```

### "ModuleNotFoundError" for other packages

**Problem**: Missing dependencies (numpy, scipy, etc.)

**Solution**:
```bash
# Reinstall all requirements
pip install --upgrade -r requirements.txt

# Or individually:
pip install numpy scipy matplotlib pyqt5 opencv-python pandas seaborn
pip install gurobipy pyserial ezdxf shapely flask flask-cors
```

### FFmpeg Not Found

**Problem**: "ffmpeg not found" when generating videos

**Solution (macOS)**:
```bash
brew install ffmpeg
```

**Solution (Linux - Ubuntu/Debian)**:
```bash
sudo apt update
sudo apt install ffmpeg
```

**Solution (Linux - Fedora/RHEL)**:
```bash
sudo dnf install ffmpeg
```

**Solution (Windows)**:
1. Download from: https://ffmpeg.org/download.html
2. Extract to a folder (e.g., `C:\ffmpeg`)
3. Add to PATH:
   - Search "Edit environment variables"
   - Add `C:\ffmpeg\bin` to PATH
4. Verify: `ffmpeg -version`

### Virtual Environment Issues

**Problem**: Dependencies installed but imports still fail

**Solution**:
```bash
# Create fresh virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install from requirements
pip install -r requirements.txt
```

---

## Runtime Errors

### "GurobiError: Model too large for license"

**Problem**: Trial license doesn't support problem size

**Solution**:
1. Get academic license (free, larger limits)
   - Follow Gurobi installation steps above
2. Reduce problem size temporarily:
   ```python
   # In config/mpc_params.py
   MPC_PREDICTION_HORIZON = 10  # Reduce from default
   MPC_CONTROL_HORIZON = 8
   ```

### "Solver time limit exceeded"

**Problem**: MPC solver takes too long to run

**Solutions**:
```python
# Reduce prediction horizon
# config/mpc_params.py
MPC_PREDICTION_HORIZON = 12  # Reduce from 15

# Reduce control horizon
MPC_CONTROL_HORIZON = 8  # Reduce from 10

# Increase time limit (short-term fix)
MPC_SOLVER_TIME_LIMIT = 0.05  # 50ms instead of 40ms
# (But keep < control timestep!)
```

### "Timing violation" in simulation

**Problem**: "Timing violation" messages in log, or simulation pauses

**Solution**:
```python
# config/timing.py
CONTROL_DT = 0.02  # Increase timestep from 0.01
# or reduce prediction horizon (see above)
```

### Serial Port Errors on Linux

**Problem**: "Permission denied" or "Serial port not found"

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in (or restart)
exit  # Log out
# Log back in

# Verify
ls -l /dev/ttyUSB*  # Should show permissions
```

### Serial Port Errors on macOS

**Problem**: "Serial port not found"

**Solution**:
```bash
# Find port name
ls /dev/tty.*
# Output might be: /dev/tty.usbserial-14110

# Update config in config/constants.py (line 55)
SERIAL_PORT = '/dev/tty.usbserial-14110'
```

### Serial Port Errors on Windows

**Problem**: "COM port not found" or "Invalid port"

**Solution**:
1. Open Device Manager:
   - Search "Device Manager"
   - Look under "Ports (COM & LPT)"
   - Note the COM port number (e.g., COM20)

2. Update config:
   ```python
   # In satellite_config.py
   SERIAL_PORT = 'COM20'
   ```

3. Check permissions:
   - Run Python as Administrator if access denied

### Import Errors with PyQt5

**Problem**: Qt conflicts between conda and pip

**Solution**:
```bash
# Use only conda for PyQt5
conda install pyqt5

# Don't mix conda and pip for Qt packages
pip uninstall pyqt5
```

---

## Hardware Issues

### Hardware Tests Fail Immediately

**Problem**: "Real.py fails with safety check error"

**Solutions**:
1. **Check Motive Server**:
   ```bash
   cd Motive
   python3 motive_data_server.py
   # Should output: "Listening on port 1111"
   ```

2. **Verify Satellite Visible in Motive**:
   - Open Motive GUI
   - Confirm markers show in 3D view
   - Confirm orbit/pose estimation is active

3. **Check Serial Connection**:
   ```bash
   # Test serial port
   python3 -c "
   import serial
   ser = serial.Serial('/dev/ttyUSB0', 115200)
   print('Connected!')
   ser.close()
   "
   ```

4. **Check Air Supply**:
   - Verify compressed air is on
   - Check pressure gauge (should be 60-80 PSI)

5. **Check Starting Position**:
   - Satellite must be in workspace bounds
   - See config/mpc_params.py: WORKSPACE_X, WORKSPACE_Y

### Satellite Won't Hover (Air Bearing Issue)

**Problem**: Satellite tilts excessively or won't float

**Solutions**:
1. **Check Floor Levelness**:
   ```bash
   # Use spirit level
   # Acceptable: < 0.5° tilt
   ```

2. **Adjust Air Bearing Pressure**:
   ```bash
   # Increase pressure gradually (10 → 80 PSI)
   # Adjust needle valves for balanced height
   ```

3. **Check for Air Leaks**:
   ```bash
   # Listen for hissing
   # Check all connections
   # Inspect for debris
   ```

4. **Clean Air Bearings**:
   ```bash
   # Turn off air supply
   # Inspect for dirt/particles
   # Clean gently with cloth
   ```

### Thrusters Don't Fire

**Problem**: "Thruster command sent but no movement"

**Solutions**:
1. **Verify Solenoid Power**:
   - Check 24V power to manifold
   - Use multimeter to verify voltage

2. **Test Solenoid Valve**:
   - Connect 24V directly to valve
   - Listen for clicking sound
   - Check for air flow at output

3. **Check Serial Command**:
   ```bash
   # Monitor serial with logic analyzer
   # Verify bitmask is correct
   # Check baud rate is 115200
   ```

4. **Verify Air Supply**:
   - Check pressure gauge
   - Check for kinks in tubing
   - Test manifold input directly

5. **Test Microcontroller**:
   ```cpp
   // In Arduino sketch, test directly:
   digitalWrite(THRUSTER_PIN, HIGH);
   delay(500);
   digitalWrite(THRUSTER_PIN, LOW);
   ```

### Motion Capture Tracking Issues

**Problem**: "Motive reports low tracking quality or no markers"

**Solutions**:
1. **Check Marker Cleanliness**:
   - Clean markers with soft cloth
   - Remove any dirt/dust

2. **Verify Camera Focus**:
   - Check cameras are in focus
   - Adjust zoom if needed

3. **Check Lighting**:
   - Increase LED ring brightness
   - Ensure adequate ambient light

4. **Recalibrate Motive**:
   - Run wand calibration
   - Check calibration quality score > 0.5

5. **Check Camera Sync**:
   - Verify sync box is connected
   - Verify sync LED is on

---

## Performance Issues

### Slow Simulation

**Problem**: Simulation runs slowly or lags

**Solutions**:
```python
# Reduce simulation fidelity
# config/timing.py
SIMULATION_TIMESTEP = 0.01  # Increase from 0.005

# Reduce MPC horizon
# config/mpc_params.py
MPC_PREDICTION_HORIZON = 10  # Reduce from 15

# Disable visualization during run
# In simulation.py, disable live plotting
```

### High CPU Usage

**Problem**: Python process uses 100% CPU

**Solutions**:
1. Close other applications
2. Reduce simulation fidelity (see above)
3. Use task manager/Activity Monitor to find bottleneck
   ```bash
   # Profile code
   python3 -m cProfile -s cumulative simulation.py
   ```

### Network Latency with Motion Capture

**Problem**: "Motion capture data delayed" or "Timing violations"

**Solutions**:
1. Check network quality:
   ```bash
   ping <motive_server_ip>
   # Should be < 5ms latency
   ```

2. Use wired connection (not WiFi)

3. Reduce data rate if possible

---

## Data Analysis Issues

### CSV File Not Generated

**Problem**: "Simulation runs but no CSV file created"

**Solution**:
```bash
# Check output directory
ls Data/Simulation/
# Should contain timestamped folders

# Check permissions
# Data/ directory should be writable
chmod 755 Data/

# Check disk space
df -h
# Should have > 1GB free
```

### Video Generation Fails

**Problem**: "ffmpeg error" or no MP4 created

**Solutions**:
```bash
# Verify FFmpeg is installed
ffmpeg -version

# Check output permission
chmod 755 Data/

# Check disk space
df -h
```

### Can't Load CSV Data

**Problem**: "Error reading CSV" in data analysis

**Solution**:
```python
# Check CSV format
import pandas as pd
data = pd.read_csv('Data/Simulation/.../simulation_data.csv')
print(data.head())
print(data.info())

# Common issues:
# - Wrong delimiter (comma vs semicolon)
# - Wrong encoding (UTF-8 vs Latin-1)
# - Missing values
```

### Plotting Fails

**Problem**: "Matplotlib error" when visualizing

**Solution**:
```bash
# Verify Matplotlib installed
pip install --upgrade matplotlib

# Check display backend
python3 -c "
import matplotlib
print(matplotlib.get_backend())
# Should output: 'TkAgg' or similar
"
```

---

## Getting Help

### Check Documentation

1. **README.md**: Quick start and overview
2. **ARCHITECTURE.md**: System design details
3. **HARDWARE_TEST_PROCEDURE.md**: Hardware setup and testing
4. **DEVELOPMENT_GUIDE.md**: Extending the code
5. **TESTING_GUIDE.md**: Test suite
6. **CSV_FORMAT.md**: Data format

### Debug Steps

1. **Enable debug logging**:
   ```python
   import logging
   logging.basicConfig(level=logging.DEBUG)
   ```

2. **Check logs**:
   ```bash
   tail -f Data/simulation.log
   tail -f Data/real_control.log
   ```

3. **Run tests**:
   ```bash
   pytest -v
   ```

4. **Check configuration**:
   ```python
   python3 -c "
   from config import SatelliteConfig
   SatelliteConfig.validate_parameters()
   SatelliteConfig.print_thruster_forces()
   "
   ```

### Open an Issue

1. Go to: https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues
2. Check if issue already exists
3. Provide:
   - Python version
   - Operating system
   - Steps to reproduce
   - Error message (full traceback)
   - Expected vs actual behavior

### Contact

- **Author**: Aevar Ofjord
- **Institution**: University of Kentucky
- **Email**: Check GitHub profile

---

**Last Updated**: November 2024

See also: [README.md](README.md), [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)
