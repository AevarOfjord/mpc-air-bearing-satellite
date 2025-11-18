# Testing Guide

Comprehensive guide for running, writing, and understanding the test suite.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Test Structure](#test-structure)
- [Running Tests](#running-tests)
- [Understanding Test Results](#understanding-test-results)
- [Writing Tests](#writing-tests)
- [Test Fixtures](#test-fixtures)
- [Integration Testing](#integration-testing)
- [Debugging Tests](#debugging-tests)
- [Test Coverage](#test-coverage)

---

## Quick Start

### Install Testing Tools

```bash
pip install pytest pytest-cov
```

### Run All Tests

```bash
pytest
```

### Run Tests with Coverage Report

```bash
pytest --cov=. --cov-report=html
open htmlcov/index.html  # View report in browser
```

---

## Test Structure

### Directory Layout

```
tests/
├── __init__.py                     # Makes tests a package
├── conftest.py                     # Pytest configuration & fixtures
├── test_config.py                  # Configuration module tests
├── test_model.py                   # Physics model tests
├── test_mpc_controller.py          # MPC solver tests
├── test_mission.py                 # Mission logic tests
├── test_navigation_utils.py        # Navigation helper tests
├── test_integration_basic.py       # Integration: basic setup
├── test_integration_missions.py    # Integration: full missions
└── [other test files]
```

### Test Categories

#### 1. Unit Tests (`test_*.py`)

Test individual modules in isolation.

**Examples**:
- `test_config.py`: Verify configuration validation
- `test_model.py`: Verify physics calculations
- `test_mpc_controller.py`: Verify optimization logic

**Characteristics**:
- Test one function/class at a time
- Use mocks to isolate dependencies
- Fast to run (< 1 second per test)
- Catch bugs early

#### 2. Integration Tests (`test_integration_*.py`)

Test component interactions.

**Examples**:
- `test_integration_basic.py`: System initialization
- `test_integration_missions.py`: Full mission workflows

**Characteristics**:
- Test multiple components together
- Use real (or minimal mocked) dependencies
- Slower than unit tests (1-10 seconds per test)
- Catch integration bugs

#### 3. Test Fixtures (`conftest.py`)

Reusable test data and mocks.

**Examples**:
- `satellite_params`: Standard satellite configuration
- `mock_gurobi_model`: Mocked optimization solver
- `mock_serial_port`: Simulated hardware interface

---

## Running Tests

### Basic Commands

```bash
# Run all tests
pytest

# Run specific test file
pytest tests/test_config.py

# Run specific test class
pytest tests/test_model.py::TestPhysicsModel

# Run specific test function
pytest tests/test_model.py::TestPhysicsModel::test_mass_is_positive

# Run tests matching pattern
pytest -k "physics"  # Run tests with "physics" in name
```

### Verbose Output

```bash
# Show test names and results
pytest -v

# Show print statements
pytest -s

# Show both
pytest -vs
```

### Filtering Tests

```bash
# Run only fast tests (skip slow)
pytest -m "not slow"

# Run only unit tests
pytest tests/test_*.py

# Run only integration tests
pytest tests/test_integration_*.py

# Run tests by keyword
pytest -k "mpc"  # Run tests with "mpc" in name
pytest -k "not slow"  # Skip slow tests
```

### Output Formats

```bash
# Minimal output
pytest -q

# Detailed output with line numbers
pytest -v --tb=long

# Stop at first failure
pytest -x

# Stop after N failures
pytest --maxfail=3

# Show local variables in tracebacks
pytest -l
```

---

## Understanding Test Results

### Test Output

```
tests/test_config.py::test_mass_is_positive PASSED          [10%]
tests/test_config.py::test_inertia_is_positive PASSED       [20%]
tests/test_model.py::TestPhysicsModel::test_thrust PASSED   [30%]
...

====== 47 passed in 1.23s ======
```

### Interpreting Symbols

- ✓ `PASSED`: Test succeeded
- ✗ `FAILED`: Test failed (assertion or exception)
- ⊘ `SKIPPED`: Test was skipped (e.g., missing dependency)
- ⚠ `XFAIL`: Expected failure (test expected to fail)
- ⊙ `XPASS`: Unexpected pass (test expected to fail but passed)
- `ERROR`: Setup or teardown failed

### Common Failures

#### AssertionError

```
AssertionError: assert 3.14159 == 3.14159265
```
**Cause**: Assertion failed. Check the condition.

#### TypeError

```
TypeError: unsupported operand type(s) for /: 'str' and 'float'
```
**Cause**: Wrong type passed to function. Check inputs.

#### ModuleNotFoundError

```
ModuleNotFoundError: No module named 'config'
```
**Cause**: Missing dependency or import path. Check installation.

#### TimeoutError (from pytest.timeout)

```
TimeoutError: Test took too long
```
**Cause**: Test hangs (infinite loop or waiting for external resource).

---

## Writing Tests

### Basic Test Structure

```python
# tests/test_example.py
import pytest
from config import SatelliteConfig
from model import compute_acceleration

class TestPhysics:
    """Test suite for physics calculations."""

    def test_acceleration_is_positive_for_positive_force(self):
        """Test that positive force produces positive acceleration."""
        mass = 10.0
        force = 5.0
        acceleration = force / mass

        assert acceleration > 0
        assert acceleration == pytest.approx(0.5)

    def test_acceleration_raises_for_zero_mass(self):
        """Test that zero mass raises error."""
        with pytest.raises(ZeroDivisionError):
            acceleration = 5.0 / 0.0
```

### Naming Conventions

```python
# Good test names describe what is tested
def test_mass_is_positive():
    """Test that satellite mass is positive."""
    pass

def test_thruster_force_increases_with_pressure():
    """Test that higher pressure produces more force."""
    pass

# Avoid unclear names
def test_physics():  # Too vague
    pass

def test_1():  # No meaning
    pass
```

### Assertion Examples

```python
# Equality
assert result == 5

# Approximate equality (for floats)
assert result == pytest.approx(3.14159, abs=1e-5)

# Comparisons
assert value > 0
assert value >= threshold

# Membership
assert element in collection
assert 'key' in dictionary

# Identity
assert obj is None
assert obj is not None

# Type checking
assert isinstance(obj, MyClass)

# Exception handling
with pytest.raises(ValueError):
    function_that_should_raise()

with pytest.raises(ValueError, match="error message"):
    function_that_raises_with_message()
```

### Test Phases

```python
def test_complete_workflow():
    """Example test with all phases."""
    # Arrange: Set up test data
    x = 1.0
    y = 2.0

    # Act: Execute code being tested
    result = add(x, y)

    # Assert: Verify result
    assert result == 3.0

    # Cleanup: Happens automatically (or use fixtures)
```

---

## Test Fixtures

### Built-in Fixtures

```python
# Temporary directory
def test_with_temp_dir(tmp_path):
    file = tmp_path / "test.txt"
    file.write_text("hello")
    assert file.read_text() == "hello"

# Monkeypatch (mock objects)
def test_with_monkeypatch(monkeypatch):
    import os
    monkeypatch.setenv("MY_VAR", "test_value")
    assert os.environ["MY_VAR"] == "test_value"
```

### Custom Fixtures

```python
# conftest.py
import pytest
from config import SatelliteConfig

@pytest.fixture
def satellite_params():
    """Provide standard satellite parameters."""
    return SatelliteConfig.get_satellite_params()

@pytest.fixture
def mpc_config():
    """Provide standard MPC configuration."""
    return SatelliteConfig.get_mpc_params()

# Use in test
def test_with_fixture(satellite_params):
    assert satellite_params['mass'] > 0
```

### Fixture Scopes

```python
# Function scope: New fixture for each test
@pytest.fixture(scope="function")
def new_each_time():
    return create_resource()

# Class scope: One fixture per test class
@pytest.fixture(scope="class")
def shared_in_class():
    return create_expensive_resource()

# Module scope: One fixture per module
@pytest.fixture(scope="module")
def expensive_setup():
    return very_expensive_resource()

# Session scope: One fixture for entire test run
@pytest.fixture(scope="session")
def database_connection():
    return setup_database()
```

### Fixture Cleanup

```python
@pytest.fixture
def resource_with_cleanup():
    """Fixture that cleans up after use."""
    resource = create_resource()
    yield resource  # Provide to test
    cleanup_resource(resource)  # Clean up after

def test_with_cleanup(resource_with_cleanup):
    # Use resource
    assert resource_with_cleanup is not None
    # Cleanup happens automatically after test
```

---

## Integration Testing

### Testing Full Mission Workflow

```python
# tests/test_integration_missions.py
import pytest
from simulation import run_simulation_step
from mission import configure_waypoint_mission
from config import SatelliteConfig

def test_waypoint_navigation_completes():
    """Test complete waypoint navigation mission."""
    # Configure mission
    mission = configure_waypoint_mission(
        waypoints=[(0, 0), (1, 0), (1, 1)]
    )

    # Initialize state
    state = {
        'x': 0, 'y': 0, 'yaw': 0,
        'vx': 0, 'vy': 0, 'angular_vel': 0
    }

    # Run simulation
    completed = False
    for step in range(10000):  # Max steps
        state = run_simulation_step(state, mission)

        if state['mission_complete']:
            completed = True
            break

    assert completed, "Mission did not complete"
    assert state['x'] == pytest.approx(1.0, abs=0.1)
    assert state['y'] == pytest.approx(1.0, abs=0.1)
```

### Testing Component Integration

```python
def test_mpc_produces_valid_commands(satellite_params, mpc_config):
    """Test that MPC solver produces valid thruster commands."""
    from mpc import MPCController

    controller = MPCController(satellite_params, mpc_config)

    # Set a target
    current_state = [0, 0, 0, 0, 0, 0]  # x, y, yaw, vx, vy, w
    target_state = [1, 1, 0, 0, 0, 0]   # Go to (1, 1)

    # Solve
    commands = controller.solve(current_state, target_state)

    # Verify commands are valid
    assert len(commands) == 8  # 8 thrusters
    for cmd in commands:
        assert cmd in [0, 1]  # Binary command
```

---

## Debugging Tests

### Using print() and logging

```python
def test_with_debug_output():
    """Test with debug output."""
    x = compute_something()
    print(f"Debug: x = {x}")  # Use -s flag to see
    assert x > 0
```

Run with:
```bash
pytest -s test_example.py
```

### Using pytest.set_trace()

```python
def test_with_breakpoint():
    """Test with interactive debugger."""
    x = 5
    pytest.set_trace()  # Debugger pauses here
    y = x * 2
    assert y == 10
```

Run with:
```bash
pytest --pdb test_example.py
```

### Using pdb

```python
def test_with_pdb():
    """Test with pdb debugger."""
    import pdb
    x = 5
    pdb.set_trace()  # Debugger pauses here
    y = x * 2
    assert y == 10
```

### Pytest Options for Debugging

```bash
# Stop at first failure and drop into debugger
pytest --pdb

# Show local variables on failure
pytest -l

# Show print statements
pytest -s

# Verbose output
pytest -v

# Last failed tests
pytest --lf

# Failed tests first
pytest --ff
```

---

## Test Coverage

### Generate Coverage Report

```bash
# Generate coverage
pytest --cov=. --cov-report=html

# View report
open htmlcov/index.html
```

### Understanding Coverage

```
Name                  Stmts   Miss  Cover   Missing
--------------------------------------------------
config/__init__.py       45      2    96%   12-13
model.py               120     10    92%   45-48, 67
mpc.py                 200      5    98%   150-151
simulation.py          250     20    92%   100-120
--------------------------------------------------
TOTAL                  615     37    94%
```

- **Stmts**: Total statements
- **Miss**: Statements not executed
- **Cover**: Percentage covered
- **Missing**: Line numbers not covered

### Coverage Goals

- **Overall**: Aim for > 80% coverage
- **Critical**: MPC, physics model > 95%
- **Utilities**: Test helpers > 70%

### Excluding Code from Coverage

```python
# Exclude function from coverage
def rarely_used_function():  # pragma: no cover
    pass

# Exclude block
if debug_mode:  # pragma: no cover
    print("Debug info")
```

### Coverage Commands

```bash
# Minimum coverage threshold
pytest --cov=. --cov-fail-under=80

# Coverage per module
pytest --cov=. --cov-report=term-missing

# XML report (for CI/CD)
pytest --cov=. --cov-report=xml
```

---

## Common Testing Patterns

### Testing with Mocks

```python
from unittest.mock import Mock, patch

def test_with_mock():
    """Test using mocked dependency."""
    # Create mock
    mock_solver = Mock()
    mock_solver.solve.return_value = [1, 0, 1, 0, 0, 0, 1, 0]

    # Use mock
    result = mock_solver.solve([0, 0, 0, 0, 0, 0])

    # Verify calls
    mock_solver.solve.assert_called_once()
    assert result[0] == 1
```

### Parametrized Tests

```python
import pytest

@pytest.mark.parametrize("mass,expected_inertia", [
    (10.0, 0.45),
    (12.5, 0.56),
    (15.0, 0.67),
])
def test_inertia_scales_with_mass(mass, expected_inertia):
    """Test that inertia scales correctly with mass."""
    from config import SatelliteConfig
    # Mock calculation
    calculated = mass * 0.045  # Simple scaling
    assert calculated == pytest.approx(expected_inertia, rel=0.01)
```

### Skipping Tests

```python
@pytest.mark.skip(reason="Not implemented yet")
def test_future_feature():
    pass

@pytest.mark.skipif(True, reason="Disabled for now")
def test_disabled_feature():
    pass

# Conditional skip
import sys
@pytest.mark.skipif(sys.platform == "win32", reason="Unix only")
def test_unix_feature():
    pass
```

### Expected Failures

```python
@pytest.mark.xfail(reason="Known bug in library")
def test_known_issue():
    assert False  # Expected to fail

@pytest.mark.xfail(strict=True)
def test_should_fail_strictly():
    """Test that must fail."""
    assert False
```

---

## Test Examples

### Configuration Tests

```python
# tests/test_config.py
def test_satellite_params_valid():
    """Verify satellite parameters are physically reasonable."""
    from config import SatelliteConfig
    params = SatelliteConfig.get_satellite_params()

    assert params['mass'] > 0
    assert params['inertia'] > 0
    assert len(params['thruster_forces']) == 8
    assert all(f > 0 for f in params['thruster_forces'])
```

### Physics Model Tests

```python
# tests/test_model.py
def test_force_produces_acceleration():
    """Test Newton's second law: F = ma."""
    from model import compute_acceleration

    mass = 10.0
    force = 25.0
    expected = force / mass  # 2.5 m/s^2

    actual = compute_acceleration(force, mass)
    assert actual == pytest.approx(expected)
```

### MPC Tests

```python
# tests/test_mpc_controller.py
def test_mpc_converges_to_target():
    """Test that MPC moves toward target."""
    from mpc import MPCController

    controller = MPCController(satellite_params, mpc_config)

    current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # At origin
    target = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]   # Move to (1, 1)

    commands = controller.solve(current, target)

    # Should produce valid commands
    assert len(commands) == 8
    assert all(cmd in [0, 1] for cmd in commands)
```

---

## Continuous Integration

### GitHub Actions Example

```yaml
# .github/workflows/tests.yml
name: Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        python-version: [3.9, 3.10, 3.11, 3.12]

    steps:
    - uses: actions/checkout@v2
    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: ${{ matrix.python-version }}

    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-cov

    - name: Run tests
      run: pytest --cov=. --cov-report=xml

    - name: Upload coverage
      uses: codecov/codecov-action@v2
```

---

## Troubleshooting Tests

### Tests Won't Run

```bash
# Issue: ModuleNotFoundError
# Solution: Run from project root
cd /path/to/mpc-air-bearing-satellite
pytest

# Issue: Can't find conftest
# Solution: Ensure tests/ has __init__.py
touch tests/__init__.py
```

### Test Takes Forever

```bash
# Issue: Test hangs
# Solution: Set timeout
pytest --timeout=5

# Check for infinite loops or blocking I/O
pytest -s --tb=short  # See where it hangs
```

### Flaky Tests

```python
# Issue: Test passes sometimes, fails sometimes
# Solution: Check for timing/randomness issues

# Retry decorator
@pytest.mark.flaky(reruns=3)
def test_flaky_behavior():
    # Test that might fail randomly
    pass

# Control randomness
import random
def test_with_seed():
    random.seed(42)  # Reproducible
```

---

## Resources

- **Pytest Documentation**: https://docs.pytest.org/
- **Pytest Fixtures**: https://docs.pytest.org/how-to/fixtures.html
- **Testing Best Practices**: https://docs.pytest.org/goodpractices.html

---

**See also**: [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md), [README.md](README.md)
