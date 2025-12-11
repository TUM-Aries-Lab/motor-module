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

from motor_python import definitions

def main() -> None:
    """Run a simple demonstration."""
    logger.info("Hello World!")

if __name__ == "__main__":
    main()
```

## Program Usage
```bash
uv run python -m motor_python
```

## Testing

### Unit Tests (No Hardware Required)
Run unit tests without hardware (using mocks):
```bash
make test
# or
uv run pytest -m "not hardware"
```

### Hardware Tests
Run hardware integration tests (requires motor connected):
```bash
make test-hardware
# or
uv run pytest -m hardware
```

### All Tests (Full Coverage)
Run all tests including hardware tests:
```bash
make test-all
# or
uv run pytest
```

**Note:** Hardware tests are skipped automatically if motor hardware is not available.

## Structure
<!-- TREE-START -->
```
├── src
│   └── motor_python
│       ├── __init__.py
│       ├── __main__.py
│       ├── cube_mars_motor.py
│       ├── defintions.py
│       └── utils.py
├── tests
│   ├── __init__.py
│   ├── conftest.py
│   ├── main_test.py
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
