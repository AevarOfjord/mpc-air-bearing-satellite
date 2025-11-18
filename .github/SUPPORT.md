# Getting Help

If you're stuck, here's how to get support for the Satellite Thruster Control System.

---

## 📖 Documentation First

**Most questions are answered in the documentation:**

### Quick Start
- [README.md](../README.md) - Quick start and overview
- [QUICK_START_CHECKLIST.md](../QUICK_START_CHECKLIST.md) - First week checklist (printable)

### Tutorials & Guides
- [HANDOFF_GUIDE.md](../HANDOFF_GUIDE.md) - Takeover guide for next project lead
- [HARDWARE_TEST_PROCEDURE.md](../HARDWARE_TEST_PROCEDURE.md) - Step-by-step hardware testing
- [THRUSTER_CALIBRATION.md](../THRUSTER_CALIBRATION.md) - Thruster calibration procedure
- [MISSION_ARCHITECTURE.md](../MISSION_ARCHITECTURE.md) - Mission types and configuration

### Technical Details
- [ARCHITECTURE.md](../ARCHITECTURE.md) - System design and algorithms
- [DEVELOPMENT_GUIDE.md](../DEVELOPMENT_GUIDE.md) - Code structure and extensions
- [CSV_FORMAT.md](../CSV_FORMAT.md) - Data format specification

### Problem Solving
- [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - Common issues and solutions
- [TESTING_GUIDE.md](../TESTING_GUIDE.md) - Running and writing tests

---

## 🔧 Troubleshooting Checklist

Before opening an issue, try these steps:

### Installation Issues
```bash
# Verify Python version
python3 --version  # Should be 3.9-3.12

# Verify Gurobi
python3 -c "import gurobipy; print('✓ Gurobi OK')"

# Verify dependencies
pip list | grep -E "numpy|scipy|pytest"

# Run tests
pytest -q
```

### Simulation Not Running
```bash
# Check if it's a missing file
python3 simulation.py

# If import error, reinstall packages
pip install -r requirements.txt

# Try a simple test
python3 -c "from config import SatelliteConfig; print('Config OK')"
```

### Hardware Issues
1. Check [HARDWARE_TEST_PROCEDURE.md](../HARDWARE_TEST_PROCEDURE.md) safety checklist
2. Verify air pressure: Main (80 PSI), Bearings (60 PSI), Thrusters (40 PSI)
3. Test thrusters: `python3 Thruster_Test/thruster_test.py`
4. Check serial port: `ls /dev/tty.*` (macOS/Linux)
5. Review [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) hardware section

### Performance Degradation
1. Run `pytest -v` to check system health
2. Check thruster calibration date
3. Verify Motive tracking quality (in 3D view)
4. Check for air bearing debris or dust
5. Review [HANDOFF_GUIDE.md](../HANDOFF_GUIDE.md) maintenance section

---

## 🐛 Report a Bug

Found a genuine bug? [Create a bug report](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=bug_report.md).

**Include:**
- Clear description of the problem
- Your environment (OS, Python version, etc.)
- Exact steps to reproduce
- Error messages or logs
- CSV data files if applicable
- What you've already tried

---

## ❓ Ask a Question

Have a question about the system? Check first:
1. Search [existing issues](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues)
2. Read relevant documentation
3. If still unclear, [open a discussion](https://github.com/AevarOfjord/mpc-air-bearing-satellite/discussions) (if enabled)

---

## 🔗 External Support

### Gurobi Issues
- **Issue**: "Model too large for license" or license errors
- **Help**: https://support.gurobi.com/
- **Also**: Verify academic license from https://www.gurobi.com/academia/

### OptiTrack / Motive Issues
- **Issue**: Tracking lost, camera connection, calibration
- **Help**: https://v21.optitrack.com/support/
- **Docs**: Check Motive manual in help menu

### Python / NumPy / SciPy Issues
- **Help**: Stack Overflow (tag with `python`, `numpy`, `scipy`)
- **Docs**: Individual package documentation

### FFmpeg Issues
- **Issue**: Video generation failed
- **Help**: https://ffmpeg.org/
- **Also**: Check `FFMPEG_PATH` in config

---

## 📋 Make a Feature Request

Have an idea? [Create a feature request](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=feature_request.md).

**Include:**
- Use case: Why do you need this?
- Proposed solution: How should it work?
- Alternatives: Other approaches considered?
- Impact: Which components affected?

---

## 📝 Improve Documentation

Notice something unclear or wrong in the docs? [Create a documentation issue](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=documentation.md).

**Include:**
- Which document
- What's unclear or wrong
- Suggested fix
- Context (when did you encounter this?)

---

## 💬 Contact Project Lead

**For research collaboration or thesis-related questions:**
- See [HANDOFF_GUIDE.md](../HANDOFF_GUIDE.md) "Important Contacts" section
- Faculty advisors can provide research direction
- Original author (if available) can discuss design decisions

---

## 🎓 Learning Resources

### Understanding MPC
- Read [ARCHITECTURE.md](../ARCHITECTURE.md) → "Control Algorithm" section
- Study `mpc.py` to see implementation
- Run `python3 simulation.py` with different parameters to see effects

### Understanding Hardware
- Read [HARDWARE_TEST_PROCEDURE.md](../HARDWARE_TEST_PROCEDURE.md)
- Study [THRUSTER_CALIBRATION.md](../THRUSTER_CALIBRATION.md)
- Check [HANDOFF_GUIDE.md](../HANDOFF_GUIDE.md) → "Hardware Quirks & Workarounds"

### Understanding Missions
- Read [MISSION_ARCHITECTURE.md](../MISSION_ARCHITECTURE.md)
- Study `mission.py` and `mission_state_manager.py`
- Try different missions in simulation

### Understanding Data
- Read [CSV_FORMAT.md](../CSV_FORMAT.md)
- Run `python3 visualize.py` to see plots
- Use Jupyter notebooks for data analysis

---

## 🔍 Debugging Tips

### Enable Logging
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

### Check System State
```bash
# Monitor resources
top  # macOS/Linux
# or Task Manager on Windows

# Check processes
ps aux | grep python

# Find open ports
lsof -i -P -n | grep LISTEN
```

### Analyze Data
```bash
# View CSV structure
head -5 Data/Real_Test/*/real_test_data.csv

# Count rows
wc -l Data/Real_Test/*/real_test_data.csv

# Extract specific column
awk -F',' '{print $2}' Data/Real_Test/*/real_test_data.csv | head
```

### Test Individual Components
```bash
# Test MPC solver
python3 -c "from mpc import MPCController; print('MPC OK')"

# Test Gurobi
python3 -c "from gurobipy import *; print('Gurobi OK')"

# Test data logging
python3 -c "import csv; print('CSV OK')"
```

---

## 📞 Support Options Summary

| Issue Type | Best Action |
|-----------|------------|
| How do I...? | Check [README.md](../README.md) or relevant guide |
| Found a bug | [Bug report](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=bug_report.md) |
| Hardware problem | [Hardware issue](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=hardware_issue.md) |
| Docs unclear | [Documentation issue](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=documentation.md) |
| Feature idea | [Feature request](https://github.com/AevarOfjord/mpc-air-bearing-satellite/issues/new?template=feature_request.md) |
| Gurobi issue | https://support.gurobi.com/ |
| OptiTrack issue | https://v21.optitrack.com/support/ |
| General Python help | Stack Overflow |

---

## 👥 Community Guidelines

When seeking help, please:
- ✅ Be respectful and patient
- ✅ Provide specific details
- ✅ Share what you've already tried
- ✅ Follow-up on your own progress
- ✅ Share solutions if you find them

- ❌ Don't demand immediate responses
- ❌ Don't post duplicate issues
- ❌ Don't be rude to volunteers
- ❌ Don't share credentials or sensitive info

---

**Still stuck?** Check the [documentation index](../README.md#documentation) or create an issue with all the information above.

We're here to help! 🚀
