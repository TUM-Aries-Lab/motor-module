# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)

This package provides Python control for CubeMars AK60-6 motors with support for both **UART** and **CAN** communication interfaces.

## Features

- **Dual Interface Support**: Control motors via UART or CAN bus
- **UART Interface**: Full-featured protocol with detailed status feedback
- **CAN Interface**: High-speed control with MCP2515 CAN controller (Waveshare board)
- **Multiple Control Modes**: Duty cycle, current, velocity, position, MIT mode
- **Real-time Feedback**: Position, velocity, current, temperature monitoring
- **Type-safe**: Full type hints and dataclass-based API
- **Well-tested**: Comprehensive unit and integration tests
- **Logging**: Structured logging with loguru

## Communication Interfaces

### UART Interface
- Uses RS485/UART communication (default: `/dev/ttyTHS1` at 921600 baud)
- Full protocol support with detailed status queries
- See main README for UART usage

### CAN Interface  
- Uses Waveshare RS485 CAN expansion board with MCP2515 controller
- 500 Kbps CAN bus communication
- Extended CAN frames with 8-byte data
- Timed motor feedback at 1-500 Hz
- **See [CAN_SETUP.md](CAN_SETUP.md) for detailed CAN setup instructions**

## Install
To install the library run:

```bash
uv install motor_python
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/motor_python.git@<specific-tag>  
```

For CAN support, also install:
```bash
pip install python-can
```

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:
1. Update the version number in pyproject.toml and imu_module/__init__.py
2. Commit your changes and add a git tag "<new.version.number>"
3. Push the tag `git push --tag`

The package can then be found at: https://pypi.org/project/motor_python

## Module Usage

### UART Interface (Default)
```python
from motor_python import CubeMarsAK606v3
from loguru import logger

# Initialize motor with UART interface
with CubeMarsAK606v3() as motor:
    if motor.connected and motor.check_communication():
        # Control motor
        motor.set_position(90.0)  # Move to 90 degrees
        motor.get_status()        # Get detailed status
        motor.stop()              # Stop motor
```

### CAN Interface
```python
from motor_python import CubeMarsAK606CAN

# Initialize motor with CAN interface (ID 0x68)
with CubeMarsAK606CAN(motor_id=0x68) as motor:
    if motor.connected:
        # Receive feedback
        status = motor.receive_feedback()
        logger.info(f"Position: {status.position:.2f}°")

        # Control motor
        motor.set_position(180.0)   # Move to 180 degrees
        motor.set_velocity(1000.0)  # Set velocity to 1000 RPM
        motor.set_current(2.0)      # Set current to 2A
```

See [CAN_SETUP.md](CAN_SETUP.md) for complete CAN setup and usage guide.

## Program Usage

### UART Interface (Default)
```bash
uv run python -m motor_python
```

### CAN Interface
```bash
# Run with CAN interface (default motor ID 0x68)
uv run python -m motor_python --interface can

# Specify custom motor ID
uv run python -m motor_python --interface can --motor-id 0x69

# Run CAN examples
uv run python -m motor_python.examples_can
```

## Testing

### Unit Tests (No Hardware Required)
Run unit tests without hardware (using mocks):
```bash
make test          # Run unit tests only (no hardware required)
make test-hardware # Run hardware integration tests (requires motor connected)
make test-all      # Run all tests including hardware tests (full coverage)
```

**Note:** Hardware tests are skipped automatically if motor hardware is not available.

## Structure
<!-- TREE-START -->
```
├── src
│   └── motor_python
│       ├── config
│       ├── __init__.py
│       ├── __main__.py
│       ├── cube_mars_motor.py
│       ├── definitions.py
│       ├── examples.py
│       ├── motor_status_parser.py
│       └── utils.py
├── tests
│   ├── __init__.py
│   ├── conftest.py
│   ├── cube_mars_motor_test.py
│   ├── hardware_test.py
│   ├── motor_status_parser_test.py
│   └── utils_test.py
├── .dockerignore
├── .gitignore
├── .pre-commit-config.yaml
├── .python-version
├── CONTRIBUTING.md
├── Dockerfile
├── LICENSE
├── Makefile
├── README.md
├── pyproject.toml
├── repo_tree.py
└── uv.lock
```
<!-- TREE-END -->
