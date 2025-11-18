# Development Guide

This guide is for developers who want to extend or modify this project for their own research.

**Note**: This is a personal research project, and pull requests are not accepted. If you want to make modifications, please fork the repository and work on your own copy. See [CONTRIBUTING.md](.github/CONTRIBUTING.md) for details.

---

## Table of Contents

- [Quick Start for Development](#quick-start-for-development)
- [Project Structure](#project-structure)
- [Code Style Guidelines](#code-style-guidelines)
- [Working with Data and CSV Logging](#working-with-data-and-csv-logging)
- [Testing Your Changes](#testing-your-changes)
- [Adding New Features](#adding-new-features)
- [Modifying Parameters](#modifying-parameters)
- [Debugging](#debugging)
- [Git Workflow](#git-workflow)
- [Contributing](#contributing)

---

## Quick Start for Development

### Setup

```bash
# Clone YOUR FORK of the repository
# (Fork the repo on GitHub first if you haven't already)
git clone https://github.com/YOUR-USERNAME/mpc-air-bearing-satellite.git
cd mpc-air-bearing-satellite

# Create virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install runtime dependencies
pip install -r requirements.txt

# Install development tools
pip install pytest pytest-cov black isort flake8 mypy
```

### Verify Everything Works

```bash
# Run tests
pytest -v

# Test simulation
python3 simulation.py

# Check code quality
flake8 .
mypy .
```

---

## Project Structure

For a complete project structure and design overview, see [ARCHITECTURE.md](ARCHITECTURE.md).

### Key Design Principles

1. **Centralized Configuration**: All parameters in `config/` package
   - Never hardcode parameters in other files
   - Import configuration where needed

2. **Modular Architecture**: Each file has a single responsibility
   - `mpc.py`: Optimization only
   - `model.py`: Physics only
   - `simulation.py` / `real.py`: Control loop only

3. **Type Hints**: All functions should have type hints
   - Enables static analysis and IDE support
   - Catch errors before runtime

4. **Docstrings**: Google-style docstrings for all classes and functions
   ```python
   def my_function(x: float, y: float) -> float:
       """Brief description.

       Longer description if needed.

       Args:
           x: Description of x
           y: Description of y

       Returns:
           Description of return value

       Raises:
           ValueError: If x is negative
       """
       return x + y
   ```

---

## Code Style Guidelines

### Python Style (PEP 8)

```bash
# Auto-format code
black .

# Sort imports
isort .

# Check style
flake8 .
```

### Guidelines

- **Line length**: Maximum 100 characters
- **Naming**:
  - Classes: `PascalCase` (e.g., `SatelliteConfig`)
  - Functions: `snake_case` (e.g., `calculate_thrust`)
  - Constants: `UPPER_CASE` (e.g., `CONTROL_DT`)
  - Private: Leading underscore (e.g., `_helper_function`)

- **Imports**: Group in order:
  1. Standard library (`import os`, `import sys`)
  2. Third-party (`import numpy`, `import pandas`)
  3. Local (`from config import SatelliteConfig`)

- **Functions**: Keep < 50 lines when possible
  - Break into smaller functions
  - Each function does one thing

- **Comments**: Explain *why*, not *what*
  ```python
  # Good
  # MPC requires linearization around current state for optimization
  A_lin = compute_linearization(x_current)

  # Avoid
  # Compute linearization
  A_lin = compute_linearization(x_current)
  ```

---

## Working with Data and CSV Logging

### CSV Data Format

Both simulation and real hardware produce identical CSV output with 45 columns per row:

**Detailed Data Log**: `simulation_data.csv` or `real_test_data.csv`
- Contains full control loop data with all state variables, MPC info, and commands
- 45 columns of timestep data
- Organized into categories: Timing & Control, Telemetry, Current State, Target State, Tracking Error, MPC Performance, Command Output, Loop Performance

**Terminal Log**: `simulation_terminal_log.csv` or `real_terminal_log.csv`
- Human-readable status messages
- 8 columns per row

For complete column definitions and descriptions, see [CSV_FORMAT.md](CSV_FORMAT.md).

### Adding Data to Logs

Use the `DataLogger` class in `data_logger.py`:

```python
from data_logger import DataLogger

# Create logger for simulation or real mode
logger = DataLogger(mode="simulation")
logger.set_save_path(Path("./Data/Simulation/test_run"))

# Log each control loop iteration
entry = {
    "Step": step_count,
    "Control_Time": current_time,
    "Current_X": x_position,
    "Current_Y": y_position,
    # ... other 41 columns ...
}
logger.log_entry(entry)

# Save to CSV when done
logger.save_csv_data()
```

**Important**: Always include all expected columns in log entries. Missing columns will appear as empty fields in the CSV.

### Modifying CSV Format

To change CSV format:

1. **Update column definitions**: Edit `_get_simulation_headers()` and `_get_real_headers()` in `data_logger.py`
   - Both must remain identical for consistent comparison between sim and hardware

2. **Update column formatting**: Modify `_format_value()` for precision/format rules

3. **Update documentation**: Reflect changes in [CSV_FORMAT.md](CSV_FORMAT.md)

4. **Update tests**: Modify `tests/test_data_logger.py` to verify new format

5. **Update data collection**: Ensure `simulation.py` and `real.py` populate new columns

---

## Testing Your Changes

### Running Tests

```bash
# Run all tests
pytest

# Run specific test file
pytest tests/test_model.py

# Run specific test function
pytest tests/test_model.py::test_physics_conservation

# Run with verbose output
pytest -v

# Run with coverage report
pytest --cov=. --cov-report=html
# Open htmlcov/index.html to view coverage
```

### Writing Tests

Create a new test file following existing patterns:

```python
# tests/test_myfeature.py
import pytest
from config import SatelliteConfig
from myfeature import my_function

class TestMyFeature:
    """Test suite for my new feature."""

    def test_basic_functionality(self):
        """Test basic case."""
        result = my_function(1, 2)
        assert result == 3

    def test_edge_case(self):
        """Test edge case."""
        with pytest.raises(ValueError):
            my_function(-1, 2)

    @pytest.fixture
    def sample_data(self):
        """Provide sample data for tests."""
        return SatelliteConfig.get_satellite_params()

    def test_with_fixture(self, sample_data):
        """Test using fixture."""
        assert sample_data['mass'] > 0
```

### Testing Workflow

```bash
# 1. Make changes to code
# (Edit simulation.py, model.py, etc.)

# 2. Run tests to catch regressions
pytest

# 3. Test in simulation before hardware
python3 simulation.py

# 4. Hardware test if applicable
python3 real.py
```

---

## Adding New Features

### Example: Adding a New Mission Type

**Goal**: Add "Spiral Navigation" mission

#### Step 1: Define Mission in `mission.py`

```python
def configure_spiral_mission():
    """Configure spiral navigation mission.

    Returns:
        dict: Mission configuration
    """
    mission = {
        'type': 'spiral',
        'start': (0, 0),
        'center': (2, 2),
        'radius_min': 0.5,
        'radius_max': 2.0,
        'revolutions': 2,
        'speed': 0.5,  # m/s target speed
    }
    return mission
```

#### Step 2: Add Logic in `simulation.py` and `real.py`

```python
def update_simulation():
    # ... existing code ...

    if mission_config['type'] == 'spiral':
        target = compute_spiral_target(
            mission_config,
            current_state,
            elapsed_time
        )
        # Rest of control loop with new target
```

#### Step 3: Add Helper Function in `mission.py`

```python
def compute_spiral_target(config, state, time):
    """Compute target position for spiral trajectory.

    Args:
        config: Spiral mission configuration
        state: Current satellite state
        time: Elapsed time in mission

    Returns:
        tuple: (x, y, yaw) target position
    """
    # Implement spiral math here
    pass
```

#### Step 4: Update `visualize.py`

```python
def visualize_spiral_mission(data):
    """Visualize spiral mission results."""
    # Add spiral-specific overlays
    # Plot trajectory over time
    pass
```

#### Step 5: Add Tests in `tests/test_mission.py`

```python
def test_spiral_target_computation():
    """Test spiral target calculation."""
    config = {'center': (2, 2), 'radius_min': 0.5, 'radius_max': 2.0}
    state = {'x': 0, 'y': 0}

    target = compute_spiral_target(config, state, time=1.0)
    assert isinstance(target, tuple)
    assert len(target) == 3  # x, y, yaw
```

#### Step 6: Add Configuration (if needed)

Edit `config/` as needed:

```python
# config/mission_state.py
SPIRAL_SPEED = 0.5  # m/s
SPIRAL_REVOLUTIONS = 2
```

#### Step 7: Update Menu

Edit `simulation.py` to add menu option:

```python
print("3. Spiral Navigation")
# ... handle selection ...
```

#### Step 8: Document

Update [README.md](README.md) mission types section with new mission description.

#### Step 9: Test

```bash
# Run new tests
pytest tests/test_mission.py

# Test in simulation
python3 simulation.py
# Select: 3. Spiral Navigation
```

---

## Modifying Parameters

### Where Parameters Live

All parameters should be in `config/` package:

```
config/
├── physics.py        # Mass, inertia, thruster positions, forces
├── timing.py         # Control rate, simulation step, stabilization times
├── mpc_params.py     # Horizons, cost weights, constraints
├── camera.py         # Camera settings (if used)
├── mission_state.py  # Mission-specific parameters
├── constants.py      # System constants
└── obstacles.py      # Obstacle definitions
```

### Guidelines

1. **Never hardcode values** in other files
   ```python
   # Good
   from config import SatelliteConfig
   mass = SatelliteConfig.get_satellite_params()['mass']

   # Bad
   mass = 12.5  # Hardcoded!
   ```

2. **Group related parameters**
   ```python
   # Good - related parameters together
   MPC_PREDICTION_HORIZON = 15
   MPC_CONTROL_HORIZON = 10
   MPC_SOLVER_TIME_LIMIT = 0.04

   # Bad - scattered around
   ```

3. **Use descriptive names**
   ```python
   # Good
   VELOCITY_THRESHOLD_FOR_DAMPING = 0.03  # m/s

   # Unclear
   V_THRESH = 0.03
   ```

4. **Include units in comments**
   ```python
   MASS = 12.5  # kg
   CONTROL_DT = 0.01  # seconds
   MAX_VELOCITY = 2.0  # m/s
   ```

### Adding New Parameters

```python
# In config/physics.py
NEW_PARAMETER = 1.5  # units

# Then use everywhere:
from config import SatelliteConfig
value = SatelliteConfig.NEW_PARAMETER
```

### Validation

The `SatelliteConfig` class provides parameter validation:

```python
# Validate all configuration parameters
is_valid = SatelliteConfig.validate_parameters()

# This checks:
# - Timing parameters are consistent
# - MPC parameters are reasonable
# - Physics parameters are valid
# - Camera settings are correct
```

You can also add custom validation in your code:

```python
def validate_mission_params(mission_config):
    """Validate mission-specific parameters."""
    params = SatelliteConfig.get_satellite_params()

    assert params['mass'] > 0, "Mass must be positive"
    assert params['inertia'] > 0, "Inertia must be positive"
    assert len(params['thruster_forces']) == 8, "Must have 8 thrusters"

    print("✓ Mission parameters valid")
```

---

## Debugging

### Using Print Statements

```python
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# Debug output
logger.debug(f"Current state: x={x}, y={y}, yaw={yaw}")
logger.info("Starting MPC solve")
logger.warning(f"Timing violation: {solve_time} > {time_limit}")
logger.error(f"Failed to solve: {error_msg}")
```

### Interactive Debugging

```python
# Add breakpoint in code
import pdb; pdb.set_trace()

# Or use breakpoint() in Python 3.7+
breakpoint()

# Commands
# n: next line
# s: step into function
# c: continue
# p variable: print variable
# l: list code
# h: help
```

### Inspect Test Behavior

```bash
# Run test with detailed output
pytest tests/test_model.py::test_physics -v -s

# -s: show print/debug output
# -v: verbose
```

### Performance Profiling

```python
import cProfile
import pstats

# Profile a function
cProfile.run('my_function()', 'stats')

# View results
p = pstats.Stats('stats')
p.sort_stats('cumulative').print_stats(10)  # Top 10 functions
```

### Simulation Debugging

```bash
# Enable debug logging
python3 -c "
import logging
logging.basicConfig(level=logging.DEBUG)
exec(open('simulation.py').read())
"
```

---

## Git Workflow

### Working with Your Fork

Since this is a personal research project, **pull requests are not accepted**. If you want to make modifications:

1. **Fork the repository** on GitHub
2. **Clone your fork** locally and make changes
3. **Commit your changes** to your fork
4. **Report bugs** back as issues if you find problems in the original code

### Making Changes in Your Fork

```bash
# 1. Work on your fork
# (You already have cloned and created a branch)
git checkout -b feature/my-new-feature

# 2. Make changes and commit
git add .
git commit -m "Add new feature: describe what and why"

# 3. Push to your fork
git push -u origin feature/my-new-feature

# 4. To report bugs to the original project:
# - Create an issue on GitHub with:
#   - Clear description of the problem
#   - Steps to reproduce
#   - Suggested solution (if you have one)
# - The project maintainer will fix verified bugs when available
```

### Commit Message Guidelines

```
Prefix: brief description

Longer explanation if needed. Describe:
- What changed
- Why it changed
- How to test

Examples:
- "feat: add spiral navigation mission"
- "fix: correct thruster calibration formula"
- "refactor: simplify MPC solver interface"
- "test: add unit tests for physics model"
- "docs: update hardware setup guide"
```

### Before Committing

```bash
# Format code
black .
isort .

# Run tests
pytest

# Check style
flake8 .
mypy .

# Then commit
git add .
git commit -m "Your message"
```

---

## Contributing

### For Your Own Fork

When working on your own fork of this project, follow these best practices:

### Code Quality Checklist

Maintain code quality by ensuring:

- [ ] Code follows PEP 8 style guidelines
- [ ] All functions have docstrings (Google style)
- [ ] Type hints on all functions
- [ ] Tests written for new code
- [ ] All tests pass (`pytest`)
- [ ] No new warnings from `flake8` or `mypy`
- [ ] Documentation updated if needed
- [ ] Commit messages follow guidelines

### Adding Tests

For every new feature or bug fix:

1. Write test that fails (TDD approach)
2. Implement feature/fix
3. Verify test passes
4. Add edge case tests

### Documentation

When modifying your fork:

- Update [README.md](README.md) for user-facing changes
- Update [ARCHITECTURE.md](ARCHITECTURE.md) for design changes
- Update docstrings for code changes
- Update this guide for development process changes

### Reporting Bugs Back

If you discover bugs in the original project:

1. Create an issue on GitHub with:
   - Clear description of the problem
   - Exact steps to reproduce
   - Your environment (OS, Python version, etc.)
   - Error messages or logs if applicable
2. The project maintainer will evaluate and fix verified bugs when available
3. You'll be credited in the issue discussion and commit messages

---

## Common Development Tasks

### Running Simulation with Debug Output

```bash
# In simulation.py, add:
logging.basicConfig(level=logging.DEBUG)

# Then run
python3 simulation.py
```

### Testing New Physics Parameters

```python
# In tests/test_model.py
def test_new_mass():
    from config.physics import MASS
    # Temporarily override for testing
    original_mass = MASS

    # ... test with new value ...

    # Restore
    # (Better: use fixtures)
```

### Comparing Changes

```bash
# See what changed
git diff

# See staged changes
git diff --staged

# Compare branches
git diff main..feature/my-feature
```

### Reverting Changes

```bash
# Undo last commit (keep changes)
git reset --soft HEAD~1

# Undo last commit (discard changes)
git reset --hard HEAD~1

# Undo specific file
git checkout HEAD -- filename.py
```

---

## Troubleshooting Development

### Import Errors

```python
# Problem: "ModuleNotFoundError: No module named 'config'"
# Solution 1: Run from project root
cd /path/to/mpc-air-bearing-satellite
python3 script.py

# Solution 2: Add to path
import sys
sys.path.insert(0, '/path/to/mpc-air-bearing-satellite')
```

### Test Discovery Issues

```bash
# Ensure tests/ has __init__.py
touch tests/__init__.py

# Run with explicit path
pytest tests/

# Run with verbose discovery
pytest --collect-only
```

### Type Checking Issues

```bash
# Install type stubs if needed
pip install types-numpy types-PyYAML

# Check specific file
mypy mpc.py

# Ignore specific line
x = func()  # type: ignore
```

---

## Resources

- **PEP 8**: https://www.python.org/dev/peps/pep-0008/
- **Google Python Style Guide**: https://google.github.io/styleguide/pyguide.html
- **Pytest Documentation**: https://docs.pytest.org/
- **Type Hints**: https://docs.python.org/3/library/typing.html

---

## Related Documentation

This guide complements several other important documents:

- **[README.md](README.md)** - Project overview and quick start
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System design and component relationships
- **[CSV_FORMAT.md](CSV_FORMAT.md)** - Complete specification of data logging format (45 columns)
- **[SIMULATION_TESTING_GUIDE.md](SIMULATION_TESTING_GUIDE.md)** - Detailed simulation testing procedures
- **[HARDWARE_TEST_PROCEDURE.md](HARDWARE_TEST_PROCEDURE.md)** - Hardware setup and testing procedures
