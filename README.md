# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

Velocity-only motor control for exosuit tendon systems (CubeMars AK60-6).

Position, current, and duty cycle modes are removed -- they cause oscillations
when combined with the firmware's high acceleration trap parameters.

## Install

```bash
uv install motor_python
```

## Quick Start

```python
from motor_python.cube_mars_motor import CubeMarsAK606v3

motor = CubeMarsAK606v3()

# Direct velocity control
motor.set_velocity(velocity_erpm=10000)   # Pull tendon
motor.set_velocity(velocity_erpm=-8000)   # Release tendon
motor.set_velocity(velocity_erpm=0)       # Stop

# Or use the tendon helper
motor.control_exosuit_tendon(action="pull", velocity_erpm=10000)
motor.control_exosuit_tendon(action="release", velocity_erpm=8000)
motor.control_exosuit_tendon(action="stop")
```

## API

| Method | Description |
|--------|-------------|
| `set_velocity(velocity_erpm, allow_low_speed=False)` | Primary control -- set speed in ERPM |
| `control_exosuit_tendon(action, velocity_erpm)` | Helper for pull / release / stop |
| `stop()` | Stop the motor (velocity = 0) |
| `get_status()` | Query all motor parameters |
| `check_communication()` | Verify motor is responding |

## Velocity Safety

**Minimum safe velocity: 5000 ERPM** (enforced by default).

Below 5000 ERPM the firmware acceleration settings cause current oscillations,
audible noise, and motor instability.

```python
motor.set_velocity(velocity_erpm=10000)              # OK
motor.set_velocity(velocity_erpm=100)                 # Raises ValueError
motor.set_velocity(velocity_erpm=100, allow_low_speed=True)  # Bypass
motor.set_velocity(velocity_erpm=0)                   # Stop always allowed
```

## Run

```bash
uv run python -m motor_python
```

## Testing

```bash
make test            # Unit tests only (no hardware, uses mocks)
make test-hardware   # All tests (unit + hardware, requires motor connected)
```

Hardware tests are skipped automatically if the motor is not available.

## Project Structure

```
src/motor_python/
    __init__.py
    __main__.py           # Entry point (make app)
    cube_mars_motor.py    # Motor controller class
    definitions.py        # Constants and limits
    examples.py           # Demo functions
    motor_status_parser.py
    utils.py
tests/
    conftest.py
    cube_mars_motor_test.py   # Unit tests (mocked serial)
    hardware_test.py          # Integration tests (real motor)
    motor_status_parser_test.py
    utils_test.py
```
