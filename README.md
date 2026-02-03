# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor-module/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor-module?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor-module/actions/workflows/ci.yml/badge.svg)



## Install
To install the library run:

```bash
uv install motor_python
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/motor_python.git@<specific-tag>  
```

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:
1. Update the version number in pyproject.toml and imu_module/__init__.py
2. Commit your changes and add a git tag "<new.version.number>"
3. Push the tag `git push --tag`

The package can then be found at: https://pypi.org/project/motor_python

## Module Usage
```python
"""Basic docstring for my module."""

from loguru import logger

from motor_python.cube_mars_motor import CubeMarsAK606v3

def main() -> None:
    """Run a simple demonstration."""
    motor = CubeMarsAK606v3()
    motor.get_status()
    motor.set_position(position_degrees=0.0)

if __name__ == "__main__":
    main()
```

## Program Usage
```bash
uv run python -m motor_python
```

## Testing

### Unit Tests (No Hardware Required)
Run tests with or without hardware:
```bash
make test          # Run unit tests only (no hardware required, using mocks)
make test-hardware # Run ALL tests with hardware integration (requires motor connected)
```

**Note:** `make test-hardware` runs all tests (unit + hardware integration) with full coverage reporting. Hardware tests are skipped automatically if motor hardware is not available.

## Structure
<!-- TREE-START -->
```
├── Test Rig CAD files
│   ├── Foot.3mf
│   ├── Jetson_Mount.3mf
│   ├── Leg_with_2_IMU's.3mf
│   ├── Motor_Case.3mf
│   ├── Rod_Adapter_Jetson.3mf
│   ├── Rod_Adapter_Motor.3mf
│   └── Spool_V2.3mf
├── docs
│   ├── AK60-6 Manual.pdf
│   └── Jetson-Orin-Nano-DevKit.pdf
├── src
│   └── motor_python
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
