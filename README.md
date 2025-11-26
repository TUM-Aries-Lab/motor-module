# Motor Control Software for Soft Exoskeleton
[![Coverage Status](https://coveralls.io/repos/github/TUM-Aries-Lab/motor_python/badge.svg?branch=main)](https://coveralls.io/github/TUM-Aries-Lab/motor_python?branch=main)
![Docker Image CI](https://github.com/TUM-Aries-Lab/motor_python/actions/workflows/ci.yml/badge.svg)



## Install
To install the library run:

```bash
uv install motor_python
```

OR

```bash
uv install git+https://github.com/TUM-Aries-Lab/motor_python.git@<specific-tag>  
```


1. ```git clone git@github.com:TUM-Aries-Lab/motor-module.git```
2. `make init` to create the virtual environment and install dependencies
3. `make format` to format the code and check for errors
4. `make test` to run the test suite
5. `make clean` to delete the temporary files and directories

## Publishing
It's super easy to publish your own packages on PyPI. To build and publish this package run:

```bash
uv build
uv publish  # make sure your version in pyproject.toml is updated
```
The package can then be found at: https://pypi.org/project/motor_python

## Module Usage
```python
"""Basic docstring for my module."""

from loguru import logger

from motor_python.config import definitions

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
