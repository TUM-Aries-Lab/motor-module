# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

Velocity and position motor control for exosuit tendon systems (CubeMars AK60-6).

**Communication Interfaces:**
- **UART:** Serial communication via RS-485 or TTL (default)
- **CAN:** CAN bus communication with SN65HVD230 transceiver (Jetson Orin Nano)

Current and duty cycle modes are removed -- they cause oscillations when combined
with the firmware's high acceleration trap parameters.

## Install

```bash
uv install motor_python
```

## Quick Start

### UART Communication (Default)

```python
from motor_python.cube_mars_motor import CubeMarsAK606v3

motor = CubeMarsAK606v3()

# Velocity control
motor.set_velocity(velocity_erpm=10000)   # Pull tendon
motor.set_velocity(velocity_erpm=-8000)   # Release tendon
motor.stop()                              # Stop (releases windings)

# Position control (hip controller calculates target from IMU angle)
motor.set_position(position_degrees=90.0)   # Go to 90 degrees
motor.set_position(position_degrees=0.0)    # Return to home
motor.get_position()                        # Read current position

# Combined: move to position at a given speed
motor.move_to_position_with_speed(target_degrees=180.0, motor_speed_erpm=6000)

# Tendon helper
motor.control_exosuit_tendon(action="pull", velocity_erpm=10000)
motor.control_exosuit_tendon(action="release", velocity_erpm=8000)
motor.control_exosuit_tendon(action="stop")
```

### CAN Bus Communication (Jetson Orin Nano)

```python
from motor_python.cube_mars_motor_can import CubeMarsAK606v3CAN

# Initialize with motor CAN ID (configured in CubeMars software)
motor = CubeMarsAK606v3CAN(motor_can_id=0x03, interface="can0", bitrate=1000000)

# Same API as UART version
motor.set_velocity(velocity_erpm=10000)
motor.stop()

# Multi-motor support (each motor has unique CAN ID)
motor_left = CubeMarsAK606v3CAN(motor_can_id=0x03)
motor_right = CubeMarsAK606v3CAN(motor_can_id=0x04)
```

**CAN Setup:** See [docs/CAN_SETUP_GUIDE.md](docs/CAN_SETUP_GUIDE.md) for complete hardware wiring and Jetson configuration instructions.

## API

| Method | Description |
|--------|-------------|
| `set_velocity(velocity_erpm, allow_low_speed=False)` | Set speed in ERPM |
| `set_position(position_degrees)` | Set target position in degrees (+/-360) |
| `get_position()` | Read current position from motor |
| `move_to_position_with_speed(target_degrees, motor_speed_erpm)` | Move to position via velocity then hold |
| `control_exosuit_tendon(action, velocity_erpm)` | Helper for pull / release / stop |
| `stop()` | Stop the motor (current = 0, releases windings) |
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

## Position Limits

**Range: -360.0 to 360.0 degrees** (one full rotation each direction).

Values outside this range are clamped with a warning.

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
    __main__.py                # Entry point (make app)
    cube_mars_motor.py         # UART motor controller class
    cube_mars_motor_can.py     # CAN motor controller class
    definitions.py             # Constants and limits
    examples.py                # Demo functions
    motor_status_parser.py
    utils.py
tests/
    conftest.py
    cube_mars_motor_test.py        # Unit tests (mocked serial)
    hardware_test.py               # Integration tests (real motor)
    motor_status_parser_test.py
    utils_test.py
docs/
    CAN_SETUP_GUIDE.md             # CAN hardware setup for Jetson
```
